#include "precomp.hpp"

namespace cv { namespace av {

class AVFrameAllocator : public MatAllocator
{
public:
  UMatData* allocate(int /*dims*/, const int* /*sizes*/, int /*type*/,
                     void* /*data0*/, size_t* /*step*/, AccessFlag /*flags*/, UMatUsageFlags /*usageFlags*/) const
  {
    return NULL;
  }

  bool allocate(UMatData* u, AccessFlag /*accessFlags*/, UMatUsageFlags /*usageFlags*/) const
  {
    if(!u) return false;
    return true;
  }

  void deallocate(UMatData* u) const
  {
    if (!u) return;

    CV_Assert(u->urefcount == 0);
    CV_Assert(u->refcount == 0);
    if (u->frame && !(u->flags & UMatData::AVFRAME_ATTACHED))
    {
      av_frame_free(&u->frame);
      u->frame = NULL;
    }
    delete u;
  }
};

namespace
{
  MatAllocator* volatile g_matAllocator = NULL;
}

MatAllocator* getAVAllocator()
{
  CV_SINGLETON_LAZY_INIT(MatAllocator, new AVFrameAllocator())
}

MatAllocator* getAllocator()
{
  if (g_matAllocator == NULL)
  {
    cv::AutoLock lock(cv::getInitializationMutex());
    if (g_matAllocator == NULL)
    {
      g_matAllocator = getAVAllocator();
    }
  }
  return g_matAllocator;
}

//////////////////////////////////////////////////////////////////////////////////

static void av_buffer_release(void *opaque, uint8_t* /*data*/)
{
  UMatOpaque *avOpaque = (UMatOpaque *)opaque;
  if (avOpaque && avOpaque->magic_number == MAGIC_MAT){
    UMatData *u = avOpaque->data;
    if (u) {
      MatAllocator *allocator = hal::getAllocator();
      allocator->deallocate(u);
    }
    delete avOpaque;
  }
}

int get_scale_and_plane(int color_format, int wscale[], int hscale[])
{
	int plane = 0;

	switch(color_format){
        case AV_PIX_FMT_GRAY8:
            wscale[0] = 1;
            hscale[0] = 1;
            plane = 1;
            break;
        case AV_PIX_FMT_NV12:
            wscale[0] = 1;
            hscale[0] = 1;
            wscale[1] = 1;
            hscale[1] = 2;
            plane = 2;
            break;
        case AV_PIX_FMT_YUVJ420P:
        case AV_PIX_FMT_YUV420P:
            wscale[0] = 1;
            hscale[0] = 1;
            wscale[1] = 2;
            hscale[1] = 2;
            wscale[2] = 2;
            hscale[2] = 2;
            plane = 3;
            break;
        case AV_PIX_FMT_YUVJ422P:
        case AV_PIX_FMT_YUV422P: // horizontal downsample
            wscale[0] = 1;
            hscale[0] = 1;
            wscale[1] = 2;
            hscale[1] = 1;
            wscale[2] = 2;
            hscale[2] = 1;
            plane = 3;
            break;
        case AV_PIX_FMT_NV16:
            wscale[0] = 1;
            hscale[0] = 1;
            wscale[1] = 1;
            hscale[1] = 1;
            plane = 2;
            break;
        case AV_PIX_FMT_YUVJ444P:
        case AV_PIX_FMT_YUV444P:
        case AV_PIX_FMT_GBRP:
            wscale[0] = 1;
            hscale[0] = 1;
            wscale[1] = 1;
            hscale[1] = 1;
            wscale[2] = 1;
            hscale[2] = 1;
            plane = 3;
            break;
        default:
            printf("unsupported format, only gray,nv12,yuv420p,nv16,yuv422p horizontal,yuv444p,rgbp supported\n");
            return 0;
	}

	return plane;
}

AVFrame *create(int height, int width, int color_format,
                void* data, bm_int64 addr, int fd, int* plane_stride,
                int* plane_size, int color_space, int color_range, int id)
{
    if (fd>=0 && (!plane_stride || !plane_size)) return NULL;

    int step = (fd >= 0) ? plane_stride[0] : (((width + 63) >> 6) << 6);
    int ylen = (fd >= 0) ? plane_size[0] : (((height + 15) >> 4) << 4) * step;
    int size[3], stride[3];
    int plane, hscale[3], wscale[3];
    int total;
    MatAllocator *allocator = hal::getAllocator();
    UMatData *u;

    if (fd < 0 && data) return NULL;  // device memory is required for avFrame
    plane = get_scale_and_plane(color_format, wscale, hscale);
    memset(size, 0, sizeof(size));
    memset(stride, 0, sizeof(stride));
    if (fd >= 0) {
        for (int i = 0; i < plane; i++){
            size[i] = plane_size[i];
            stride[i] = plane_stride[i];
        }
    } else {
        for (int i = 0; i < plane; i++){
            size[i] = ylen / wscale[i] / hscale[i];
            stride[i] = step / wscale[i];
        }
    }

    total = size[0] + size[1] + size[2];
    u = allocator->allocate(total, data, addr, fd, id);
    if (!u) {
        printf("hal Allocate ERR\n");
        return NULL;
    }

    UMatOpaque *avOpaque = new UMatOpaque;
    avOpaque->magic_number = MAGIC_MAT;
    avOpaque->data = u;

    AVFrame *f = av_frame_alloc();
    f->format = color_format;
    f->height = height;
    f->width = width;
    f->opaque = avOpaque;
    f->colorspace = (AVColorSpace)color_space;
    f->color_range = (AVColorRange)color_range;

    f->buf[0] = av_buffer_create((uchar *)u->addr, u->size, av_buffer_release, avOpaque, AV_BUFFER_FLAG_READONLY);
    f->data[4] = (uchar *)u->addr;
    f->data[0] = u->data;
    f->linesize[4] = stride[0];
    f->linesize[0] = stride[0];

    f->data[5] = f->data[4] + size[0];
    f->data[1] = f->data[0] + size[0];
    f->linesize[5] = stride[1];
    f->linesize[1] = stride[1];

    f->data[6] = f->data[5] + size[1];
    f->data[2] = f->data[1] + size[1];
    f->linesize[6] = stride[2];
    f->linesize[2] = stride[2];

    return f;
}

AVFrame *create(int height, int width, int id) // 420P
{
  // 64 bytes alignment for vpp
  int step = (((width + 63) >> 6) << 6);
  int ylen = (((height + 15) >> 4) << 4) * step;
  int uvlen = ylen >> 2;

  int total = ylen + uvlen + uvlen;
  MatAllocator *allocator = hal::getAllocator();
  UMatData *u = allocator->allocate(total, id);
  if (!u) {
    printf("ION Allocate ERR\n");
    return NULL;
  }
  UMatOpaque *avOpaque = new UMatOpaque;
  avOpaque->magic_number = MAGIC_MAT;
  avOpaque->data = u;

  AVFrame *f = av_frame_alloc();
  f->format = AV_PIX_FMT_YUV420P;
  f->height = height;
  f->width = width;
  f->opaque = avOpaque;

  f->buf[0] = av_buffer_create((uchar *)u->addr, u->size, av_buffer_release, avOpaque, AV_BUFFER_FLAG_READONLY);
  f->data[4] = (uchar *)u->addr;
  f->data[0] = u->data;
  f->linesize[4] = step;
  f->linesize[0] = step;

  f->data[5] = f->data[4] + ylen;
  f->data[1] = f->data[0] + ylen;
  f->linesize[5] = step / 2;
  f->linesize[1] = step / 2;

  f->data[6] = f->data[5] + uvlen;
  f->data[2] = f->data[1] + uvlen;
  f->linesize[6] = step / 2;
  f->linesize[2] = step / 2;

  return f;
}

void flush(AVFrame *f)
{
  if (f->opaque) {
    UMatOpaque *avOpaque = (UMatOpaque *)f->opaque;
    if (avOpaque && avOpaque->magic_number == MAGIC_MAT){
      UMatData *u = (UMatData *)avOpaque->data;
      MatAllocator *allocator = hal::getAllocator();
      allocator->flush(u, 0);
    }
  }
}

int copy(AVFrame *src, AVFrame *dst, int id)
{
	int ret = 0;

#ifdef HAVE_BMCV
	if (!src->width || !src->height || !dst->width || !dst->height ||
		src->width != dst->width || src->height != dst->height ||
		src->format != dst->format)
		return ret;

	bm_handle_t handle = bmcv::getCard(id);
	int width = src->width;
	int height = src->height;
	int plane = 0;
	int wscale[3], hscale[3];
	int src_ysize = src->linesize[0] * height;
	int dst_ysize = dst->linesize[0] * height;

	memset(wscale, 0, sizeof(wscale));
	memset(hscale, 0, sizeof(hscale));
	plane = get_scale_and_plane(src->format, wscale, hscale);

	/* copy device memory only */
	for (int i = 0; i < plane; i++){
		int src_size = src_ysize / wscale[i] / hscale[i];
		int dst_size = dst_ysize / wscale[i] / hscale[i];

#ifdef USING_SOC
		bm_device_mem_t mem = bm_mem_from_device((bm_uint64)dst->data[4+i], dst_size);
		bm_mem_invalidate_device_mem(handle, &mem);
#endif
		if (src->linesize[4+i] == dst->linesize[4+i]){
			bm_device_mem_t src_mem = bm_mem_from_device((bm_uint64)src->data[4+i], src_size);
			bm_device_mem_t dst_mem = bm_mem_from_device((bm_uint64)dst->data[4+i], dst_size);

			//bm_memcpy_d2d_byte(handle, dst_mem, 0, src_mem, 0, src_size);
			bm_memcpy_c2c(handle, handle, src_mem, dst_mem, false);
			ret += src_size;
		}
		else {
			for (int j = 0; j < height / hscale[i]; j++){
				bm_device_mem_t src_mem =
					bm_mem_from_device((bm_uint64)src->data[4+i] + j*src->linesize[4+i], width/wscale[i]);
				bm_device_mem_t dst_mem =
					bm_mem_from_device((bm_uint64)dst->data[4+i] + j*dst->linesize[4+i], width/wscale[i]);

				//bm_memcpy_d2d_byte(handle, dst_mem, j * dst->linesize[4+i],
				//		src_mem, j * src->linesize[4+i], width / wscale[i]);
				bm_memcpy_c2c(handle, handle, src_mem, dst_mem, false);
				ret += width / wscale[i];
			}
		}
	}
#endif

	return ret;
}

}}
