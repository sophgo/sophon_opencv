#include "precomp.hpp"

#include <iostream>
#include <fstream>
#include <map>

using namespace std;

#ifdef HAVE_BMCV

namespace cv { namespace bmcv {

#define CARD_MAX 64
static bm_handle_t mCards[CARD_MAX] = { 0 };

static void init()
{
  bmlib_log_set_level(BMLIB_LOG_ERROR);
}

static void exit()
{
  for (int i = 0; i < CARD_MAX; i++) {
    bm_handle_t handle = mCards[i];
    if (handle) {
      bm_dev_free(handle);
    }
  }
}

class BmcvInstance
{
public:
   BmcvInstance() { init(); }
   ~BmcvInstance() { exit(); }
};

static BmcvInstance instance;  // init instanse
static cv::Mutex _initBmcv_mutex;

static std::map<std::pair<AVColorSpace, AVColorRange>, std::pair<csc_type_t, csc_type_t>> colorspace_map_csc = {
    {std::pair<AVColorSpace, AVColorRange>(AVCOL_SPC_BT709, AVCOL_RANGE_MPEG),     std::pair<csc_type_t, csc_type_t>(CSC_YCbCr2RGB_BT709, CSC_RGB2YCbCr_BT709)},
    {std::pair<AVColorSpace, AVColorRange>(AVCOL_SPC_BT709, AVCOL_RANGE_JPEG),     std::pair<csc_type_t, csc_type_t>(CSC_YPbPr2RGB_BT709, CSC_RGB2YPbPr_BT709)},
    {std::pair<AVColorSpace, AVColorRange>(AVCOL_SPC_BT470BG, AVCOL_RANGE_MPEG),   std::pair<csc_type_t, csc_type_t>(CSC_YCbCr2RGB_BT601, CSC_RGB2YCbCr_BT601)},
    {std::pair<AVColorSpace, AVColorRange>(AVCOL_SPC_SMPTE170M, AVCOL_RANGE_MPEG), std::pair<csc_type_t, csc_type_t>(CSC_YCbCr2RGB_BT601, CSC_RGB2YCbCr_BT601)},
    {std::pair<AVColorSpace, AVColorRange>(AVCOL_SPC_SMPTE240M, AVCOL_RANGE_MPEG), std::pair<csc_type_t, csc_type_t>(CSC_YCbCr2RGB_BT601, CSC_RGB2YCbCr_BT601)},
    {std::pair<AVColorSpace, AVColorRange>(AVCOL_SPC_BT470BG, AVCOL_RANGE_JPEG),   std::pair<csc_type_t, csc_type_t>(CSC_YPbPr2RGB_BT601, CSC_RGB2YPbPr_BT601)},
    {std::pair<AVColorSpace, AVColorRange>(AVCOL_SPC_SMPTE170M, AVCOL_RANGE_JPEG), std::pair<csc_type_t, csc_type_t>(CSC_YPbPr2RGB_BT601, CSC_RGB2YPbPr_BT601)},
    {std::pair<AVColorSpace, AVColorRange>(AVCOL_SPC_SMPTE240M, AVCOL_RANGE_JPEG), std::pair<csc_type_t, csc_type_t>(CSC_YPbPr2RGB_BT601, CSC_RGB2YPbPr_BT601)},
};

static csc_type_t get_csc_type_by_colorinfo(AVColorSpace space, AVColorRange range, int inRGB)
{
    csc_type_t csc = CSC_MAX_ENUM;
    auto key = std::pair<AVColorSpace, AVColorRange>(space, range);

    if (colorspace_map_csc.find(key) == colorspace_map_csc.end())
        csc = CSC_MAX_ENUM;
    else{
        if (inRGB)  csc = colorspace_map_csc[key].second;
        else csc = colorspace_map_csc[key].first;
    }
    return csc;
}

static int map_bmformat_to_avformat(int bmformat)
{
    int format;
    switch(bmformat){
        case FORMAT_YUV420P: format = AV_PIX_FMT_YUV420P; break;
        case FORMAT_YUV422P: format = AV_PIX_FMT_YUV422P; break;
        case FORMAT_YUV444P: format = AV_PIX_FMT_YUV444P; break;
        case FORMAT_NV12:    format = AV_PIX_FMT_NV12; break;
        case FORMAT_NV16:    format = AV_PIX_FMT_NV16; break;
        case FORMAT_GRAY:    format = AV_PIX_FMT_GRAY8; break;
        case FORMAT_RGBP_SEPARATE: format = AV_PIX_FMT_GBRP; break;
        default: printf("unsupported image format %d\n", bmformat); return -1;
    }
    return format;
}

static int map_avformat_to_bmformat(int avformat)
{
    int format;
    switch(avformat){
        case AV_PIX_FMT_YUV420P: format = FORMAT_YUV420P; break;
        case AV_PIX_FMT_YUV422P: format = FORMAT_YUV422P; break;
        case AV_PIX_FMT_YUV444P: format = FORMAT_YUV444P; break;
        case AV_PIX_FMT_NV12:    format = FORMAT_NV12; break;
        case AV_PIX_FMT_NV16:    format = FORMAT_NV16; break;
        case AV_PIX_FMT_GRAY8:   format = FORMAT_GRAY; break;
        case AV_PIX_FMT_GBRP:    format = FORMAT_RGBP_SEPARATE; break;
        default: printf("unsupported av_pix_format %d\n", avformat); return -1;
    }

    return format;
}

bm_handle_t getCard(int id)
{
#ifdef USING_SOC
  id = 0;
#endif
  // avoid thread competition to request device more
  cv::AutoLock lock(_initBmcv_mutex);

  id = BM_CARD_ID(id);
  if (mCards[id]) return mCards[id];

  bm_handle_t handle;
  bm_status_t ret = bm_dev_request(&handle, id);
  if (ret != BM_SUCCESS) {
    printf("Create Bm Handle Failed\n");
    ::exit(0);
  }

  mCards[id] = handle;
  return handle;
}

int getId(bm_handle_t handle)
{
#ifdef USING_SOC
  return 0;
#else
  int ret = bm_get_devid(handle);
  if (ret < 0){
	printf("Invalid Card Handle\n");
	::exit(0);
	return 0;
  }

  return ret;
#endif
}

bm_device_mem_t getDeviceMem(bm_uint64 addr, int len)
{
  return bm_mem_from_device(addr, len);
}


static void upload(bm_handle_t handle, Mat &m, bm_image *image)
{
  uchar *p = m.data;
  bm_device_mem_t mem[4];

  if (p == NULL) return;  // m.data==NULL means there is not virtual memory, thus no sync up required.
#ifndef USING_SOC
  if (m.avOK()) return;   // if it is avOK, the data is from vpu/jpu and always working on physical memory.
#endif

  memset(mem, 0, sizeof(mem));
  bm_image_get_device_mem(*image, mem);
  for (int i = 0; i < 4; i++) {
    if (mem[i].size > 0) {
#ifdef USING_SOC
      bm_mem_flush_device_mem(handle, &mem[i]);
#else
      bm_memcpy_s2d(handle, mem[i], p);
      p += mem[i].size;
#endif
    }
  }
}

static bm_status_t toBMI_MISC(Mat &m, bm_image *image, bool update)
{
  if (!m.u || !m.u->addr) {printf("Memory allocated by user, no device memory assigned. Not support BMCV!\n"); return BM_NOT_SUPPORTED;}
  bm_status_t ret;
  bm_handle_t handle = m.u->hid ? m.u->hid : getCard();
  bm_image_data_format_ext data_format;
  bm_image_format_ext image_format;

  CV_Assert(m.rows > 0 && m.cols > 0);

  uint len, off;
  bm_device_mem_t mem;

  int step[1] = { (int)m.step[0] };
  if (m.type() == CV_8UC3) { data_format = DATA_TYPE_EXT_1N_BYTE; image_format = FORMAT_BGR_PACKED; }
  else if (m.type() == CV_32FC1) {data_format = DATA_TYPE_EXT_FLOAT32; image_format = FORMAT_GRAY; }
  else if (m.type() == CV_32FC3) {data_format = DATA_TYPE_EXT_FLOAT32; image_format = FORMAT_BGR_PACKED; }
  else if (m.type() == CV_8SC3) {data_format = DATA_TYPE_EXT_1N_BYTE_SIGNED; image_format = FORMAT_BGR_PACKED; }
  else if (m.type() == CV_8SC1) {data_format = DATA_TYPE_EXT_1N_BYTE_SIGNED; image_format = FORMAT_GRAY;}
  else return BM_NOT_SUPPORTED;
  ret = bm_image_create(handle, m.rows, m.cols, image_format, data_format, image, step);
  if (ret != BM_SUCCESS) return ret;

  off = m.data - m.datastart;
  len = m.rows * m.step[0];
  mem = bm_mem_from_device(m.u->addr + off, len);

  ret = bm_image_attach(*image, &mem);

  if (update) upload(handle, m, image);

  return ret;
}

static bm_status_t toBMI_COMP(Mat &m, bm_image *image)
{
  if (!m.u || !m.u->addr) {printf("Memory allocated by user, no device memory assigned. Not support BMCV!\n"); return BM_NOT_SUPPORTED;}
  bm_status_t ret;
  bm_handle_t handle = m.u->hid ? m.u->hid : getCard();

  CV_Assert(m.rows > 0 && m.cols > 0);

  int step[4] = { m.avStep(4), m.avStep(5), m.avStep(6), m.avStep(7) };

  ret = bm_image_create(handle, m.avRows(), m.avCols(), FORMAT_COMPRESSED, DATA_TYPE_EXT_1N_BYTE, image, step);
  if (ret != BM_SUCCESS) return ret;

  bm_device_mem_t mem[4];
  memset(mem, 0, sizeof(mem));

  // 0
  uint len0 = m.avStep(6);
  mem[0] = bm_mem_from_device(m.avAddr(6), len0);

  // 1
  uint len1 = m.avRows() * m.avStep(4);
  mem[1] = bm_mem_from_device(m.avAddr(4), len1);

  // 2
  uint len2 = m.avStep(7);
  mem[2] = bm_mem_from_device(m.avAddr(7), len2);

  // 3
  uint len3 = m.avRows() * m.avStep(5) / 2;
  mem[3] = bm_mem_from_device(m.avAddr(5), len3);

  ret = bm_image_attach(*image, mem);
  return ret;
}

static bm_status_t toBMI_GRAY8(Mat &m, bm_image *image, bool update)
{
  if (!m.u || !m.u->addr) {printf("Memory allocated by user, no device memory assigned. Not support BMCV!\n"); return BM_NOT_SUPPORTED;}
  bm_status_t ret;
  bm_handle_t handle = m.u->hid ? m.u->hid : getCard();

  CV_Assert(m.rows > 0 && m.cols > 0);

  int step[1] = { (int)m.step[0] };
  ret = bm_image_create(handle, m.rows, m.cols, FORMAT_GRAY, DATA_TYPE_EXT_1N_BYTE, image, step);
  if (ret != BM_SUCCESS) return ret;

  uint off = m.data - m.datastart;
  uint len = m.rows * m.step[0];
  bm_device_mem_t mem = bm_mem_from_device(m.u->addr + off, len);

  ret = bm_image_attach(*image, &mem);
  if (update) upload(handle, m, image);

  return ret;
}

static bm_status_t toBMI_AVFrame(Mat &m, bm_image *image, bool update)
{
  if (!m.u || !m.u->addr) {printf("Memory allocated by user, no device memory assigned. Not support BMCV!\n"); return BM_NOT_SUPPORTED;}
  bm_status_t ret;
  bm_handle_t handle = m.u->hid ? m.u->hid : getCard();
  int plane, hscale[3], wscale[3];
  int step[3] = { m.avStep(4), m.avStep(5), m.avStep(6) };
  bm_image_format_ext bmformat = (bm_image_format_ext)map_avformat_to_bmformat(m.avFormat());
  if (bmformat < 0) return BM_NOT_SUPPORTED;

  plane = av::get_scale_and_plane(m.avFormat(), wscale, hscale);
  ret = bm_image_create(handle, m.rows, m.cols, bmformat, DATA_TYPE_EXT_1N_BYTE, image, step);
  if (ret != BM_SUCCESS) return ret;

  bm_device_mem_t mem[3];
  memset(mem, 0, sizeof(mem));

  uint off = m.data - m.datastart;
  // 0
  uint len0 = m.rows * m.avStep(4);
  mem[0] = bm_mem_from_device(m.avAddr(4) + off, len0);

  // 1-2
  for (int i = 1; i < plane; i++){
      int cx = off % m.avStep(4) / wscale[i];
      int cy = off / m.avStep(4) / hscale[i];
      uint cff = cy * m.avStep(4+i) + cx;

      uint len = m.rows * m.avStep(4+i) / hscale[i];
      mem[i] = bm_mem_from_device(m.avAddr(4+i) + cff, len);
  }

  ret = bm_image_attach(*image, mem);
  if (update) upload(handle, m, image);

  return ret;
}

bm_status_t toBMI(Mat &m, bm_image *image, bool update)
{
  if (m.avOK()) {
    if (m.avComp()) return toBMI_COMP(m, image);
    else return toBMI_AVFrame(m, image, update);
  } else if (m.type() == CV_8UC1)
    return toBMI_GRAY8(m, image, update);
  else
    return toBMI_MISC(m, image, update);
}

static bm_status_t toBMI_4IN1(Mat &m, Mat &m1, Mat &m2, Mat &m3, bm_image *image, bool update)
{
  if (!m.u || !m.u->addr) {printf("Memory allocated by user, no device memory assigned. Not support BMCV!\n"); return BM_NOT_SUPPORTED;}
  bm_status_t ret;
  bm_handle_t handle = m.u->hid ? m.u->hid : getCard();

  CV_Assert(m.rows > 0 && m.cols > 0);
  CV_Assert(m.rows == m1.rows && m.rows == m2.rows && m.rows == m3.rows);
  CV_Assert(m.cols == m1.cols && m.cols == m2.cols && m.cols == m3.cols);

  Mat v[4] = { m, m1, m2, m3 };

  bm_image src[4];
  memset(src, 0, sizeof(src));

  for (int i = 0; i < 4; i++) {
    ret = toBMI(v[i], src + i, update);
    if (ret != BM_SUCCESS) goto done;
  }

  ret = bm_image_create(handle, m.rows, m.cols, src[0].image_format, DATA_TYPE_EXT_4N_BYTE, image);
  if (ret != BM_SUCCESS) goto done;

  ret = bmcv_image_storage_convert(handle, 4, src, image);
  if (ret != BM_SUCCESS) goto done;

  bm_thread_sync(handle);

done:
  for (int i = 0; i < 4; i++) {
    bm_image_destroy(src[i]);
  }

  return ret;
}


bm_status_t toBMI(Mat &m, Mat &m1, Mat &m2, Mat &m3, bm_image *image, bool update)
{
  if ((m.avOK() && (m.avFormat() == AV_PIX_FMT_GRAY8 || m.avFormat() == AV_PIX_FMT_NV12  ||
                    m.avFormat() == AV_PIX_FMT_NV16  || m.avFormat() == AV_PIX_FMT_YUV444P ||
                    m.avFormat() == AV_PIX_FMT_GBRP  || m.avFormat() == AV_PIX_FMT_YUV420P))
      || ((m.type() == CV_8UC1) && (m1.type() == CV_8UC1) && (m2.type() == CV_8UC1) && (m3.type() == CV_8UC1))
      || ((m.type() == CV_8UC3) && (m1.type() == CV_8UC3) && (m2.type() == CV_8UC3) && (m3.type() == CV_8UC3)))
      return toBMI_4IN1(m, m1, m2, m3, image, update);

  printf("MAT AVForamt Invalid\n");
  return BM_NOT_SUPPORTED;
}

static void download(bm_handle_t handle, Mat &m, bm_image *image)
{
  uchar *p = m.data;
  bm_device_mem_t mem[4];

  memset(mem, 0, sizeof(mem));
  bm_image_get_device_mem(*image, mem);

  for (int i = 0; i < 4; i++) {
    if (mem[i].size > 0) {
#ifdef USING_SOC
      bm_mem_invalidate_device_mem(handle, &mem[i]);
#else
      bm_memcpy_d2s(handle, p, mem[i]);
      p += mem[i].size;
#endif
    }
  }
}

bm_status_t attachDeviceMemory(Mat &m)
{
    if (m.empty()){
        printf("input Mat is empty, please create it!\n");
        return BM_NOT_SUPPORTED;
    }
    if (m.allocator && m.allocator != hal::getAllocator()){
        printf("user defined allocator does not supported!\n");
        return BM_NOT_SUPPORTED;
    }

#ifdef USING_SOC
    printf("Attach device memory to host is not supported at soc mode, please re-create mat again\n");
    return BM_NOT_SUPPORTED;
#else
    if (!(m.u && m.u->addr)){
        int d = m.dims;
        int i;
        int sz[CV_MAX_DIM];
        Size whole_size;
        Point start_offset;
        Size roi_size;
        int isSubmatrix = m.isSubmatrix();

        if (d <= 2){
            m.locateROI(whole_size, start_offset);
            roi_size = m.size();
            sz[0] = whole_size.height;
            sz[1] = whole_size.width;
        } else {
            for (i=0; i<d; i++)
                sz[i] = m.size.p[i];
        }

        m.create(d, sz, m.dataend-m.datastart, m.type(), m.step.p, (void*)m.datastart, 0, -1, m.card);
        if (isSubmatrix)
		    m = m(Rect(start_offset.x, start_offset.y, roi_size.width, roi_size.height));
    }

    return BM_SUCCESS;
#endif
}

bm_status_t toMAT(Mat &in, Mat &m, bool update)
{
  bm_status_t ret;
  csc_type_t csc;

  if (in.avOK()) {
    bm_image src;

    csc = get_csc_type_by_colorinfo(in.u->frame->colorspace, in.u->frame->color_range, 0);
    toBMI(in, &src, update);

    ret = toMAT(&src, m, update, csc);

    bm_image_destroy(src);
  } else {
    m = in;
    ret = BM_SUCCESS;
  }

  return ret;
}

bm_status_t toMAT(bm_image *image, Mat &m, int color_space, int color_range, void* vaddr, int fd0, bool update, bool nocopy)
{
    bm_status_t ret;
    bm_handle_t handle = bm_image_get_handle(image);
    bm_image_format_info_t image_info;
    int id = bm_get_devid(handle);
    int fd;
    bm_uint64 addr;
    int *p_plane_stride;
    int plane_size[4];
    int i, total = 0;
    int height, width;

    if (m.card) id = BM_MAKEFLAG(0, BM_CARD_HEAP(m.card), id);
    if (image->data_type != DATA_TYPE_EXT_1N_BYTE) {printf("toMAT only support U8\n"); return BM_NOT_SUPPORTED;}
    if (!bm_image_is_attached(*image)) {printf("bm image is null\n"); return BM_ERR_FAILURE;}

    if (!nocopy){
        csc_type_t csc;
        csc = get_csc_type_by_colorinfo((AVColorSpace)color_space, (AVColorRange)color_range, 0);
        ret = toMAT(image, m, update, csc);

        return ret;
    }

    ret = bm_image_get_format_info(image, &image_info);
    if (ret != BM_SUCCESS) {
        printf("Get bm_image format info failed\n");
        return ret;
    }

    p_plane_stride = image_info.stride;
    height = image_info.height;
    width = image_info.width;

    /* prepare input parameter */
    int actual_total = 0;
    for (i = 1; i < image_info.plane_nb; i++){
        bm_int64 base_addr = bm_mem_get_device_addr(image_info.plane_data[i-1]);
        bm_int64 curr_addr = bm_mem_get_device_addr(image_info.plane_data[i]);

        plane_size[i-1] = curr_addr - base_addr;
        actual_total += image_info.plane_data[i-1].size;
        total += plane_size[i-1];
    }

    plane_size[i-1] = image_info.plane_data[i-1].size;
    total += plane_size[i-1];
    actual_total += image_info.plane_data[i-1].size;
    if (total > actual_total + 1024*1024 || total < actual_total){
        printf("Mat does not support discontinuous memory!\n");
        return BM_NOT_SUPPORTED;
    }

    addr = bm_mem_get_device_addr(image_info.plane_data[0]);
    fd = (fd0 >= 0) ? fd0 : image_info.plane_data[0].u.device.dmabuf_fd;
    if (fd < 0 && addr > 0) fd = 0;   // have valid device memory

    /* format conversion */
    if (image->image_format == FORMAT_BGR_PACKED){
#if 0
        printf("unsupported BGR_PACKED image format\n");
        return BM_NOT_SUPPORTED;
#else
        size_t step[2] = {0, 0};
        step[0] = p_plane_stride[0];
        m.create(height, width, total, CV_8UC3, step, vaddr, addr, fd, id);
#endif
    }
    else {
        AVPixelFormat format;
        format = (AVPixelFormat)map_bmformat_to_avformat(image_info.image_format);
        if (format < 0) return BM_NOT_SUPPORTED;

        AVFrame *f = av::create(height, width, format, vaddr, addr, fd, p_plane_stride, plane_size, color_space,
                     color_range, id);
        /* mat.create will not create new avframe if avframe is already in mat.
         * But here is special that avframe contains decoded buffer, we expect it
         * to be updated. So release img at first
         */
        if (!m.empty()) m.release();
        m.create(f, id);
    }

    if (update) download(handle, m, image);

    return BM_SUCCESS;
}

bm_status_t toMAT(bm_image *image, Mat &m, bool update, csc_type_t csc)
{
  bm_status_t ret;
  bm_handle_t handle = bm_image_get_handle(image);

  // uint len;
  // bm_device_mem_t mem;

  int id = bm_get_devid(handle);
  if (m.card) id = BM_MAKEFLAG(0, BM_CARD_HEAP(m.card), id);
  if (m.empty()) {
    m.create(image->height, image->width, CV_8UC3, id);
  }

  bm_image dst;
  ret = toBMI(m, &dst, false);
  if (ret != BM_SUCCESS) goto done;

#ifdef USING_SOC
  download(handle, m, &dst);  // clear cache at soc mode, ugly but have to
#endif
  ret = bmcv_image_storage_convert_with_csctype(handle, 1, image, &dst, csc);
  if (ret != BM_SUCCESS) goto done;

  bm_thread_sync(handle);
#ifndef USING_SOC
  if (update) download(handle, m, &dst);
#endif

done:
  bm_image_destroy(dst);
  return ret;
}

bm_status_t toMAT(bm_image *image, Mat &m, Mat &m1, Mat &m2, Mat &m3, bool update, csc_type_t csc)
{
  bm_status_t ret;
  bm_handle_t handle = bm_image_get_handle(image);
  int id = bm_get_devid(handle);
  if (m.card) id = BM_MAKEFLAG(0, BM_CARD_HEAP(m.card), id);

  if (m.empty()) {
    m.create(image->height, image->width, CV_8UC3, id);
  }
  if (m1.empty()) {
    m1.create(image->height, image->width, CV_8UC3, id);
  }
  if (m2.empty()) {
    m2.create(image->height, image->width, CV_8UC3, id);
  }
  if (m3.empty()) {
    m3.create(image->height, image->width, CV_8UC3, id);
  }

  Mat v[4] = { m, m1, m2, m3 };

  bm_image dst[4];
  memset(dst, 0, sizeof(dst));

  for (int i = 0; i < 4; i++) {
    int step[1] = { (int)m.step[0] };
    ret = bm_image_create(handle, image->height, image->width, FORMAT_BGR_PACKED, DATA_TYPE_EXT_1N_BYTE, dst + i, step);
    if (ret != BM_SUCCESS) goto done;

    uint len = m.rows * m.step[0];
    bm_device_mem_t mem = bm_mem_from_device(v[i].u->addr, len);

    ret = bm_image_attach(dst[i], &mem);
    if (ret != BM_SUCCESS) goto done;
  }
#ifdef USING_SOC
  for (int i = 0; i < 4; i++)
    download(handle, v[i], &dst[i]); // clear cache at soc mode, ugly but have to
#endif
  ret = bmcv_image_storage_convert_with_csctype(handle, 4, image, dst, csc);
  if (ret != BM_SUCCESS) goto done;

  bm_thread_sync(handle);

done:
  for (int i = 0; i < 4; i++) {
#ifndef USING_SOC
    if (update) download(handle, v[i], &dst[i]);
#endif
    bm_image_destroy(dst[i]);
  }

  return ret;
}

bm_status_t decomp(Mat &m, Mat &out, bool update)
{
  if (!m.u || !m.u->addr) {printf("Memory allocated by user, no device memory assigned. Not support BMCV!\n"); return BM_NOT_SUPPORTED;}
  bm_status_t ret;
  bm_handle_t handle = m.u->hid ? m.u->hid : getCard();
  csc_type_t csc;

  if (!m.avComp()) return BM_NOT_SUPPORTED;
  if (out.empty()) return BM_ERR_NOMEM;

  bm_image src;
  toBMI(m, &src, false);

  bm_image dst;
  toBMI(out, &dst, false);

  if (out.avOK()){
    out.u->frame->colorspace = m.u->frame->colorspace;
    out.u->frame->color_range = m.u->frame->color_range;
    csc = CSC_MAX_ENUM;
  } else
    csc = get_csc_type_by_colorinfo(m.u->frame->colorspace, m.u->frame->color_range, 0);

  bmcv_rect_t rt;
  rt.start_x = 0;
  rt.start_y = 0;
  rt.crop_w = m.cols;  // here m.cols represent display width if display width not equal to coded width in fbc frame
  rt.crop_h = m.rows; // here m.rows represent display height if display height not equal to coded width

#ifdef USING_SOC
  download(handle, m, &dst);  // clear cache at soc mode, ugly but have to
#endif
  ret = bmcv_image_vpp_csc_matrix_convert(handle, 1, src, &dst, csc, nullptr, BMCV_INTER_NEAREST, &rt);
  if (ret != BM_SUCCESS) goto done;

  bm_thread_sync(handle);
#ifndef USING_SOC
  if(update) download(handle, out, &dst);
#endif

done:
  bm_image_destroy(src);
  bm_image_destroy(dst);
  return ret;
}

bm_status_t convert(Mat &m, Mat &out, bool update)
{
  if (!m.u || !m.u->addr) {printf("Memory allocated by user, no device memory assigned. Not support BMCV!\n"); return BM_NOT_SUPPORTED;}
  bm_status_t ret;
  bm_handle_t handle = m.u->hid ? m.u->hid : getCard();
  csc_type_t csc = CSC_MAX_ENUM;
  int id = getId(handle);
  if (out.card) id = BM_MAKEFLAG(0, BM_CARD_HEAP(out.card), id);

  bm_image src;
  toBMI(m, &src, update);

  bm_image dst;
  if (out.empty()) out.create(src.height, src.width, CV_8UC3, id);
  toBMI(out, &dst, false);

  if (m.avOK() && out.avOK())
  {
    out.u->frame->colorspace = m.u->frame->colorspace;
    out.u->frame->color_range = m.u->frame->color_range;
    csc = CSC_MAX_ENUM;
  }
  else if (m.avOK() && !out.avOK())
    csc = get_csc_type_by_colorinfo(m.u->frame->colorspace, m.u->frame->color_range, 0);
  else if (!m.avOK() && out.avOK())
    csc = get_csc_type_by_colorinfo(out.u->frame->colorspace, out.u->frame->color_range, 1);
  else
    csc = CSC_MAX_ENUM;

#ifdef USING_SOC
  download(handle, out, &dst);  // clear cache at soc mode, ugly but have to
#endif
  ret = bmcv_image_storage_convert_with_csctype(handle, 1, &src, &dst, csc);
  if (ret != BM_SUCCESS) goto done;

  bm_thread_sync(handle);
#ifndef USING_SOC
  if (update) download(handle, out, &dst);
#endif

done:
  bm_image_destroy(src);
  bm_image_destroy(dst);
  return ret;
}

bm_status_t convert(Mat &m, std::vector<Rect> &vrt, std::vector<Size> &vsz, std::vector<Mat> &out, bool update,
        csc_type_t csc, csc_matrix_t *matrix, bmcv_resize_algorithm algorithm)
{
  if (!m.u || !m.u->addr) {printf("Memory allocated by user, no device memory assigned. Not support BMCV!\n"); return BM_NOT_SUPPORTED;}
  bm_status_t ret = BM_SUCCESS;
  bm_handle_t handle = m.u->hid ? m.u->hid : getCard();

  CV_Assert(vrt.size() == vsz.size());
  CV_Assert(out.size() == 0 || out.size() == vsz.size());

  bm_image src;
  toBMI(m, &src, update);

  if (out.size() == 0) {
    for (unsigned int i = 0; i < vrt.size(); i++) {
      Mat ma(vsz[i].height, vsz[i].width, CV_8UC3, SophonDevice(getId(handle)));
      out.push_back(ma);
    }
  } else {
    for (unsigned int i = 0; i < vrt.size(); i++) {
      int id = getId(handle);
      if (out[i].card) id = BM_MAKEFLAG(0, BM_CARD_HEAP(out[i].card), id);

      if (out[i].empty())
        out[i].create(vsz[i].height, vsz[i].width, CV_8UC3, id);
    }
  }

  bm_image *dst = new bm_image[vrt.size()];
  for (unsigned int i = 0; i < vrt.size(); i++) {
    toBMI(out[i], dst + i, false);
  }

  bmcv_rect_t *brt = new bmcv_rect_t[vrt.size()];

  for (unsigned int i = 0; i < vrt.size(); i++) {
    brt[i].start_x = vrt[i].x;
    brt[i].start_y = vrt[i].y;
    brt[i].crop_w = vrt[i].width;
    brt[i].crop_h = vrt[i].height;
  }

  bool has_csc = true, has_convert = false;
  if ((algorithm != BMCV_INTER_NEAREST) && !(m.avOK() && m.avFormat() == AV_PIX_FMT_YUV444P)){
    if (src.image_format == dst[0].image_format)
      has_csc = false;
    if (!(vrt[0].width == m.cols && vrt[0].height == m.rows && vrt.size() == 1 &&
          vrt[0].width == vsz[0].width && vrt[0].height == vsz[0].height))
      has_convert = true;
  }

  bm_image *dst_csc = NULL;
  int output_num;
  bmcv_resize_algorithm alg;
  bmcv_rect_t *rt;
  if (has_convert && has_csc){
    dst_csc = new bm_image;
    ret = bm_image_create(handle, m.rows, m.cols, dst[0].image_format, DATA_TYPE_EXT_1N_BYTE, dst_csc);
    if (ret != BM_SUCCESS) goto done;

    output_num = 1;
    alg = BMCV_INTER_NEAREST;
    rt = NULL;
  }else if (!has_convert && !has_csc) {   // bypass
    if (update) {
      download(handle, m, &src);
    }
    out[0] = m;
    goto done;
  }else if(has_csc){
    dst_csc = dst;
    output_num = vrt.size();
    alg = algorithm;
    rt = brt;
  }else
    dst_csc = &src;

#ifdef USING_SOC
  for (unsigned int i = 0; i < vrt.size(); i++) {
    download(handle, out[i], &dst[i]); // clear cache at soc mode
  }
#endif
 // step1 - csc
  if (has_csc) {
    ret = bmcv_image_vpp_csc_matrix_convert(handle, output_num, src, dst_csc, csc, matrix, alg, rt);
    if (ret != BM_SUCCESS) goto done;
  }

  // step2 - convert
  if (has_convert){
    ret = bmcv_image_vpp_convert(handle, vrt.size(), *dst_csc, dst, brt, algorithm);
    if (ret != BM_SUCCESS) goto done;
  }

  bm_thread_sync(handle);

#ifndef USING_SOC
  if (update)
    for (unsigned int i = 0; i < vrt.size(); i++) {
      download(handle, out[i], &dst[i]);
    }
#endif

done:
  bm_image_destroy(src);
  for (unsigned int i = 0; i < vrt.size(); i++) {
    bm_image_destroy(dst[i]);
  }
  if (has_convert && has_csc){
      bm_image_destroy(*dst_csc);
      delete dst_csc;
  }

  if (dst) delete[] dst;
  if (brt) delete[] brt;
  return ret;
}

bm_status_t convert(Mat &m, std::vector<Rect> &vrt, bm_image *dst, bool update)
{
  if (!m.u || !m.u->addr) {printf("Memory allocated by user, no device memory assigned. Not support BMCV!\n"); return BM_NOT_SUPPORTED;}
  bm_status_t ret;
  bm_handle_t handle = m.u->hid ? m.u->hid : getCard();

  bm_image src;
  toBMI(m, &src, update);

  bmcv_rect_t *brt = new bmcv_rect_t[vrt.size()];

  for (unsigned int i = 0; i < vrt.size(); i++) {
    brt[i].start_x = vrt[i].x;
    brt[i].start_y = vrt[i].y;
    brt[i].crop_w = vrt[i].width;
    brt[i].crop_h = vrt[i].height;
  }

  ret = bmcv_image_vpp_convert(handle, vrt.size(), src, dst, brt);
  if (ret != BM_SUCCESS) goto done;

  bm_thread_sync(handle);

done:
  bm_image_destroy(src);

  if (brt) delete[] brt;
  return ret;
}

bm_status_t resize(Mat &m, Mat &out, bool update, int interpolation)
{
  if (!m.u || !m.u->addr) {printf("Memory allocated by user, no device memory assigned. Not support BMCV!\n"); return BM_NOT_SUPPORTED;}
  bm_status_t ret;
  bm_handle_t handle = m.u->hid ? m.u->hid : getCard();

  if (out.empty()) {
    printf("In resize API, out Mat need to allocate memory at first!\n");
    return BM_ERR_FAILURE;
  }

  bm_image src, dst;
  toBMI(m, &src, update);

  toBMI(out, &dst, false);

  bmcv_resize_t resize;
  resize.start_x = 0;
  resize.start_y = 0;
  resize.in_width = m.cols;
  resize.in_height = m.rows;
  resize.out_width = dst.width;
  resize.out_height = dst.height;

  bmcv_resize_image attr[4];
  memset(attr, 0, sizeof(attr));

  attr[0].resize_img_attr = &resize;
  attr[0].roi_num = 1;
  attr[0].stretch_fit = 1;
  attr[0].interpolation = interpolation;

#ifdef USING_SOC
  download(handle, out, &dst);
#endif
  ret = bmcv_image_resize(handle, 1, attr, &src, &dst);
  if (ret != BM_SUCCESS) goto done;

  bm_thread_sync(handle);
#ifndef USING_SOC
  if (update) download(handle, out, &dst);
#endif

done:
  bm_image_destroy(src);
  bm_image_destroy(dst);
  return ret;
}

bm_status_t stitch(std::vector<Mat>& in, std::vector<Rect>& srt, std::vector<Rect>& drt, Mat &out, bool update,
        bmcv_resize_algorithm algorithm)
{
    bm_status_t ret;
    bm_handle_t handle;

    CV_Assert(in.size() > 0 && in.size() < 256);
    CV_Assert(srt.size() == drt.size() || srt.size() == 0);
    CV_Assert(drt.size() == in.size());
    CV_Assert(out.rows != 0 && out.cols != 0);        // out mat must be prepared

    /* preparation */
    if (!in[0].u || !in[0].u->addr) {printf("Memory allocated by user, no device memory assigned. Not support BMCV!\n"); return BM_NOT_SUPPORTED;}
    handle = in[0].u->hid ? in[0].u->hid : getCard();
    for (unsigned int i = 1; i < in.size(); i++) {
        CV_Assert(in[i].avOK() == in[0].avOK());
        if (!in[0].avOK())
            CV_Assert(in[i].type() == in[0].type());
        else
            CV_Assert(in[i].avFormat() == in[0].avFormat());
    }
    for (unsigned int i = 0; i < drt.size(); i++){
        if (drt[i].x > out.cols) drt[i].x = out.cols;
        if (drt[i].y > out.rows) drt[i].y = out.rows;
        if (drt[i].x + drt[i].width > out.cols) drt[i].width = out.cols - drt[i].x;
        if (drt[i].y + drt[i].height > out.rows) drt[i].height = out.rows - drt[i].y;
    }
    for (unsigned int i = 0; i < srt.size(); i++){
        if (srt[i].x > in[i].cols) srt[i].x = in[i].cols;
        if (srt[i].y > in[i].rows) srt[i].y = in[i].rows;
        if (srt[i].x + srt[i].width > in[i].cols) srt[i].width = in[i].cols - srt[i].x;
        if (srt[i].y + srt[i].height > in[i].rows) srt[i].height = in[i].rows - srt[i].y;
    }

    if (!out.empty()){
        CV_Assert(out.avOK() == in[0].avOK());
        if (in[0].avOK()) CV_Assert(out.avFormat() == in[0].avFormat());
        else CV_Assert(out.type() == in[0].type());
    } else {
        int id = getId(handle);
        if (out.card) id = BM_MAKEFLAG(0, BM_CARD_HEAP(out.card), id);
        if (in[0].avOK())
            if (in[0].avFormat() == AV_PIX_FMT_YUV420P){
                AVFrame *f = cv::av::create(out.rows, out.cols, id);
                out.create(f, id);
            } else {
                printf("only format YUV420P allocation is supported\n");
                return BM_NOT_SUPPORTED;
            }
        else
            out.create(out.rows, out.cols, in[0].type(), id);
    }

    /* start to stitch */
    bm_image *src_image = new bm_image[in.size()];
    bm_image dst_image;
    bmcv_rect_t *src_crop = srt.size() ? new bmcv_rect_t[srt.size()] : NULL;
    bmcv_rect_t *dst_crop = new bmcv_rect_t[drt.size()];

    for (unsigned int i = 0; i < in.size(); i++)
        toBMI(in[i], src_image + i, update);
    toBMI(out, &dst_image, update);

    for (unsigned int i = 0; i < srt.size(); i++){
        src_crop[i].start_x = srt[i].x;
        src_crop[i].start_y = srt[i].y;
        src_crop[i].crop_w = srt[i].width;
        src_crop[i].crop_h = srt[i].height;
    }

    for (unsigned int i = 0; i < drt.size(); i++){
        dst_crop[i].start_x = drt[i].x;
        dst_crop[i].start_y = drt[i].y;
        dst_crop[i].crop_w = drt[i].width;
        dst_crop[i].crop_h = drt[i].height;
    }
#ifdef USING_SOC
    download(handle, out, &dst_image);
#endif
    ret = bmcv_image_vpp_stitch(handle, in.size(), src_image, dst_image, dst_crop, src_crop, algorithm);
    if (ret != BM_SUCCESS) goto done;

    bm_thread_sync(handle);
#ifndef USING_SOC
    if (update)
        download(handle, out, &dst_image);
#endif

done:
    for (unsigned int i = 0; i < in.size(); i++)
        bm_image_destroy(src_image[i]);
    bm_image_destroy(dst_image);

    if (src_image) delete[] src_image;
    if (src_crop) delete[] src_crop;
    if (dst_crop) delete[] dst_crop;

    return ret;
 }

static void dump_RGB_AND_GRAY(Mat &image, const String &fname)
{
  if (image.channels() > 3) return;
  if (image.channels() == 1 && image.avOK()) downloadMat(image);

  fstream fs;
  fs.open(fname, ios::out | ios::binary | ios::trunc);

  for (int i = 0; i < image.rows; i++)
    fs.write(reinterpret_cast<const char*>(image.ptr(i)), image.cols*image.step[1]);

  fs.close();
}

static void dump_NV12_AND_NV16(Mat &image, const String &fname)
{
  fstream fs;
  fs.open(fname, ios::out | ios::binary | ios::trunc);

  downloadMat(image);

  int yrows = image.avRows();
  int crows = (image.avFormat() == AV_PIX_FMT_NV16) ? yrows : (yrows+1)/2;
  int step[2] = {image.avStep(0), image.avStep(1)};
  int ycols = image.avCols();
  int ccols = ycols;
  const char *p[2] = {reinterpret_cast<const char*>(image.avAddr(0)), \
      reinterpret_cast<const char*>(image.avAddr(1))};

  for (int i = 0; i < yrows; i++)
      fs.write(p[0]+i*step[0], ycols);
  for (int i = 0; i < crows; i++)
      fs.write(p[1]+i*step[1], ccols);

  fs.close();
}

static void dump_YUV420P_AND_YUV422P(Mat &image, const String &fname)
{
  fstream fs;
  fs.open(fname, ios::out | ios::binary | ios::trunc);

  downloadMat(image);

  int yrows = image.avRows();
  int crows = (image.avFormat() == AV_PIX_FMT_YUV422P) ? yrows : (yrows+1)/2;
  int step[3] = {image.avStep(0), image.avStep(1), image.avStep(2)};
  int ycols = image.avCols();
  int ccols = (ycols+1)/2;
  const char *p[3] = {reinterpret_cast<const char*>(image.avAddr(0)), \
      reinterpret_cast<const char*>(image.avAddr(1)),
      reinterpret_cast<const char*>(image.avAddr(2))};

  for (int i = 0; i < yrows; i++)
      fs.write(p[0]+i*step[0], ycols);
  for (int i = 0; i < crows; i++)
      fs.write(p[1]+i*step[1], ccols);
  for (int i = 0; i < crows; i++)
      fs.write(p[2]+i*step[2], ccols);

  fs.close();
}

static void dump_YUV444P_AND_RGBP(Mat &image, const String &fname)
{
  fstream fs;
  fs.open(fname, ios::out | ios::binary | ios::trunc);

  downloadMat(image);

  int rows = image.avRows();
  int step[3] = {image.avStep(0), image.avStep(1), image.avStep(2)};
  int cols = image.avCols();
  const char *p[3] = {reinterpret_cast<const char*>(image.avAddr(0)), \
      reinterpret_cast<const char*>(image.avAddr(1)),
      reinterpret_cast<const char*>(image.avAddr(2))};

  for (int i = 0; i < rows; i++)
      fs.write(p[0]+i*step[0], cols);
  for (int i = 0; i < rows; i++)
      fs.write(p[1]+i*step[1], cols);
  for (int i = 0; i < rows; i++)
      fs.write(p[2]+i*step[2], cols);

  fs.close();
}

static void dump_COMP(Mat &image, const String &fname)
{
  fstream fs;
  bm_handle_t handle = image.u->hid ? image.u->hid : getCard();

  int rows = image.avRows();
  int size[4] = {image.avStep(0)*rows, image.avStep(1)*rows/2, image.avStep(2), image.avStep(3)};
  const char *p[4] = {
	  reinterpret_cast<const char*>(new char[size[0]]),
	  reinterpret_cast<const char*>(new char[size[1]]),
	  reinterpret_cast<const char*>(new char[size[2]]),
	  reinterpret_cast<const char*>(new char[size[3]])
  };

  for (int i = 0; i < 4; i++){
    fs.open(fname+"."+std::to_string(i), ios::out | ios::binary | ios::trunc);

	bm_device_mem_t mem = bm_mem_from_device(image.avAddr(4+i), size[i]);
	bm_memcpy_d2s(handle, const_cast<char *>(p[i]), mem);

	fs.write(p[i], size[i]);
	fs.close();
  }

  delete p[0];
  delete p[1];
  delete p[2];
  delete p[3];
}

void dumpMat(Mat &image, const String &fname)
{
  /* to be extend for other format later */
  if(!image.avOK()) dump_RGB_AND_GRAY(image, fname);
  else if(image.avComp()) return dump_COMP(image, fname);
  else if(AV_PIX_FMT_NV12 == image.avFormat()) dump_NV12_AND_NV16(image, fname);
  else if(AV_PIX_FMT_NV16 == image.avFormat()) dump_NV12_AND_NV16(image, fname);
  else if(AV_PIX_FMT_YUV420P == image.avFormat()) dump_YUV420P_AND_YUV422P(image, fname);
  else if(AV_PIX_FMT_YUV422P == image.avFormat()) dump_YUV420P_AND_YUV422P(image, fname);
  else if(AV_PIX_FMT_GRAY8 == image.avFormat()) dump_RGB_AND_GRAY(image, fname);
  else if(AV_PIX_FMT_YUV444P == image.avFormat()) dump_YUV444P_AND_RGBP(image, fname);
  else if(AV_PIX_FMT_GBRP == image.avFormat()) dump_YUV444P_AND_RGBP(image, fname);
}


void print(Mat &m, bool dump)
{
  if (m.avOK()) printf("MAT YUV: %dx%d\n", m.cols, m.rows);
  else printf("MAT RGB: %dx%d\n", m.cols, m.rows);

  if (dump) dumpMat(m, "mat_dump.bin");
}

static int bm_image_sizeof_data_type(bm_image *image){

    switch(image->data_type){
    case DATA_TYPE_EXT_FLOAT32:
        return sizeof(float);
    case DATA_TYPE_EXT_1N_BYTE:
    case DATA_TYPE_EXT_1N_BYTE_SIGNED:
        return sizeof(char);
    case DATA_TYPE_EXT_4N_BYTE:
    case DATA_TYPE_EXT_4N_BYTE_SIGNED:
        return sizeof(char) * 4;
    default:
        return 1;
    }
}

static void dump_RGB_AND_GRAY(bm_image *image, const String &fname)
{
  fstream fs;
  fs.open(fname, ios::out | ios::binary | ios::trunc);

  int size, stride;
  int rgbWidth = image->width * (image->image_format == FORMAT_BGR_PACKED ? 3 : 1);
  int rgbHeight = image->height;
  int size_of_data_type;

  bm_image_get_byte_size(*image, &size);
  bm_image_get_stride(*image, &stride);
  size_of_data_type = bm_image_sizeof_data_type(image);

  char *pp[1]= { new char[size] };
  bm_image_copy_device_to_host(*image, (void **)pp);

  for (int i = 0; i < rgbHeight; i++)
	fs.write(pp[0]+i*stride, rgbWidth * size_of_data_type);

  delete pp[0];

  fs.close();
}

static void dump_YUV420P_AND_YUV422P(bm_image *image, const String &fname)
{
  fstream fs;
  fs.open(fname, ios::out | ios::binary | ios::trunc);

  int size[3], stride[3];
  int yWidth = image->width;
  int uvWidth = (yWidth+1)/2;
  int yHeight = image->height;
  int uvHeight = (image->image_format == FORMAT_YUV422P) ? yHeight : (yHeight+1)/2;
  int size_of_data_type = bm_image_sizeof_data_type(image);

  bm_image_get_byte_size(*image, size);
  bm_image_get_stride(*image, stride);

  char *pp[3]= { new char[size[0]], new char[size[1]], new char[size[2]] };
  bm_image_copy_device_to_host(*image, (void **)pp);

  for (int i = 0; i < yHeight; i++)
	fs.write(pp[0]+i*stride[0], yWidth*size_of_data_type);
  for (int i = 0; i < uvHeight; i++)
	fs.write(pp[1]+i*stride[1], uvWidth*size_of_data_type);
  for (int i = 0; i < uvHeight; i++)
	fs.write(pp[2]+i*stride[2], uvWidth*size_of_data_type);

  delete pp[0];
  delete pp[1];
  delete pp[2];

  fs.close();
}

static void dump_NV12_AND_NV16(bm_image *image, const string &fname)
{
  fstream fs;
  fs.open(fname, ios::out | ios::binary | ios::trunc);

  int size[2], stride[2];
  int yWidth = image->width;
  int uvWidth = yWidth;
  int yHeight = image->height;
  int uvHeight = (image->image_format == FORMAT_NV16) ? yHeight : (yHeight+1)/2;
  int size_of_data_type = bm_image_sizeof_data_type(image);

  bm_image_get_byte_size(*image, size);
  bm_image_get_stride(*image, stride);

  char *pp[2]= { new char[size[0]], new char[size[1]] };
  bm_image_copy_device_to_host(*image, (void **)pp);

  for (int i = 0; i < yHeight; i++)
	fs.write(pp[0]+i*stride[0], yWidth*size_of_data_type);
  for (int i = 0; i < uvHeight; i++)
	fs.write(pp[1]+i*stride[1], uvWidth*size_of_data_type);

  delete pp[0];
  delete pp[1];

  fs.close();
}

static void dump_YUV444P_AND_RGBP(bm_image *image, const String &fname)
{
  fstream fs;
  fs.open(fname, ios::out | ios::binary | ios::trunc);

  int size[3], stride[3];
  int width = image->width;
  int height = image->height;
  int size_of_data_type = bm_image_sizeof_data_type(image);

  bm_image_get_byte_size(*image, size);
  bm_image_get_stride(*image, stride);

  char *pp[3]= { new char[size[0]], new char[size[1]], new char[size[2]] };
  bm_image_copy_device_to_host(*image, (void **)pp);

  for (int i = 0; i < height; i++)
	fs.write(pp[0]+i*stride[0], width*size_of_data_type);
  for (int i = 0; i < height; i++)
	fs.write(pp[1]+i*stride[1], width*size_of_data_type);
  for (int i = 0; i < height; i++)
	fs.write(pp[2]+i*stride[2], width*size_of_data_type);

  delete pp[0];
  delete pp[1];
  delete pp[2];

  fs.close();

}

void dumpBMImage(bm_image *image, const String &fname)
{
  if (image->image_format == FORMAT_BGR_PACKED) dump_RGB_AND_GRAY(image, fname);
  else if (image->image_format == FORMAT_YUV420P) dump_YUV420P_AND_YUV422P(image, fname);
  else if (image->image_format == FORMAT_YUV422P) dump_YUV420P_AND_YUV422P(image, fname);
  else if (image->image_format == FORMAT_NV12) dump_NV12_AND_NV16(image, fname);
  else if (image->image_format == FORMAT_NV16) dump_NV12_AND_NV16(image, fname);
  else if (image->image_format == FORMAT_GRAY) dump_RGB_AND_GRAY(image, fname);
  else if (image->image_format == FORMAT_YUV444P) dump_YUV444P_AND_RGBP(image, fname);
  else if (image->image_format == FORMAT_RGBP_SEPARATE) dump_YUV444P_AND_RGBP(image, fname);
}

void print(bm_image *image, bool dump)
{
  printf("BMI: %dx%d F%d\n", image->width, image->height, image->image_format);

  if (!dump) return;

  char name[128];
  sprintf(name, "BMI-%dx%d.bin", image->width, image->height);

  dumpBMImage(image, name);
}

///////////////////////////////////////////////////////////////////////////////////////////////
static bm_status_t uploadUsrData(bm_handle_t handle, void *src, bm_device_mem_t mem)
{
  bm_status_t ret = BM_SUCCESS;
#ifdef USING_SOC
  ret = bm_mem_flush_device_mem(handle, &mem);
#else
  ret = bm_memcpy_s2d(handle, mem, src);
#endif
  return ret;
}

static bm_status_t downloadDeviceData(bm_handle_t handle, void *dst, bm_device_mem_t mem)
{
    bm_status_t ret = BM_SUCCESS;
#ifdef USING_SOC
    ret = bm_mem_invalidate_device_mem(handle, &mem);
#else
    ret = bm_memcpy_d2s(handle, dst, mem);
#endif
    return ret;
}


/* if you change data in user space you need update data to dma*/
void uploadMat(Mat &mat)
{
  if (!mat.u || !mat.u->addr) {printf("Memory allocated by user, no device memory assigned. Not support BMCV!\n"); return;}
  bm_handle_t handle = mat.u->hid ? mat.u->hid : getCard();
  void *p = static_cast<void *>(const_cast<uchar *>(mat.datastart));
  bm_device_mem_t mem;

  memset(&mem, 0, sizeof(mem));

  if (!mat.data) return;

#ifndef USING_SOC
  CV_Assert(mat.u->size <= mat.u->mem.size);  // 4k alignment
#endif
  mem = bm_mem_from_device(mat.u->addr, mat.u->size);
  uploadUsrData(handle, p, mem);

  return;
}

void downloadMat(Mat &mat)
{
  if (!mat.u || !mat.u->addr) {printf("Memory allocated by user, no device memory assigned. Not support BMCV!\n"); return;}
  bm_handle_t handle = mat.u->hid ? mat.u->hid : getCard();
  void *p = static_cast<void *>(const_cast<uchar *>(mat.datastart));
  bm_device_mem_t mem;

  memset(&mem, 0, sizeof(mem));
#ifndef USING_SOC
  if (mat.u->mem.size == 0) return;   // no device memory
#endif
  if (mat.data == NULL) return;       // no system memory

  mem = bm_mem_from_device(mat.u->addr, mat.u->size);

  downloadDeviceData(handle, p, mem);

  return;
}

#ifdef USING_SOC
static bm_status_t matMemToBMI(Mat &memMat, bm_image *image, bm_image_format_ext format, bool needflush)
{
    if (!memMat.u || !memMat.u->addr) {printf("Memory allocated by user, no device memory assigned. Not support BMCV!\n"); return BM_NOT_SUPPORTED;}
    bm_status_t ret;
    bm_device_mem_t mem[3];
    memset(mem, 0, sizeof(mem));
    bm_handle_t handle = memMat.u->hid ? memMat.u->hid : getCard();
    if(FORMAT_YUV420P == format)
    {
        int sstep[3] = {memMat.step[0], memMat.step[0]/2,memMat.step[0]/2};
        int imghigh = memMat.rows*2/3;

        ret = bm_image_create(handle, imghigh, memMat.cols, FORMAT_YUV420P, DATA_TYPE_EXT_1N_BYTE, image, sstep);
        if (ret != BM_SUCCESS) return ret;
        uint Ylen = imghigh * memMat.step[0];
        mem[0] = bm_mem_from_device(memMat.u->addr, Ylen);
        uint Ulen = memMat.step[0] * (imghigh / 4) + ((imghigh / 2) % 2) * (memMat.step[0] / 2);
        mem[1] = bm_mem_from_device(memMat.u->addr+Ylen, Ulen);
        mem[2] = bm_mem_from_device(memMat.u->addr+Ylen+Ulen, Ulen);
        ret = bm_image_attach(*image, mem);
        if (ret != BM_SUCCESS) return ret;
    }
    else if(FORMAT_NV12 == format)
    {
        int sstep[2] = { memMat.step[0], memMat.step[0]};
        int imghigh = memMat.rows*2/3;
        ret = bm_image_create(handle, imghigh, memMat.cols, format, DATA_TYPE_EXT_1N_BYTE, image, sstep);
        if (ret != BM_SUCCESS) return ret;

        uint Ylen = imghigh * memMat.step[0];
        mem[0] = bm_mem_from_device(memMat.u->addr, Ylen);
        uint Yvlen = imghigh * memMat.step[0]/2;
        mem[1] = bm_mem_from_device(memMat.u->addr+Ylen, Yvlen);
        ret = bm_image_attach(*image, mem);
        if (ret != BM_SUCCESS) return ret;
    }
    else if(FORMAT_BGR_PACKED == format || FORMAT_RGB_PACKED == format || FORMAT_GRAY == format)
    {
        int sstep[1] = { memMat.step[0] };
        ret = bm_image_create(handle, memMat.rows, memMat.cols, format, DATA_TYPE_EXT_1N_BYTE, image, sstep);
        if (ret != BM_SUCCESS) return ret;
        uint slen = memMat.rows * memMat.step[0];
        mem[0] = bm_mem_from_device(memMat.u->addr+(memMat.data-memMat.datastart), slen);
        ret = bm_image_attach(*image, mem);
        if (ret != BM_SUCCESS) return ret;
    }
    else
    {
        printf("bmcv can't support format= %d \n", format);
        return BM_ERR_FAILURE;
    }
    if(needflush)
    {
       bm_device_mem_t totalmem = bm_mem_from_device(memMat.u->addr, memMat.rows*memMat.step[0]);
       ret =  uploadUsrData(handle, memMat.data, totalmem);
    }
    return ret;
}

/* for src Mat data is aligned with stride -> vpp u / v data is aligned with stride/2 */
static void alignWithHalfStride(uchar* uv, int stride, int width, int uvheight)
{
    if (stride/2 <= width/2 || stride%2 != 0)
        return;

    int halfStride = stride / 2;
    int halfWidth = width / 2;

    for(int j = 0; j < uvheight; j++) {
        for(int i = halfWidth-1; i >= 0; i--) {
            *(uv + j * stride + halfStride + i) = *(uv + j * stride + halfWidth + i);
        }
    }
}

 /* for dst in vpp u / v data is aligned with stride/2 -> Mat data is aligned with stride */

static void alignWithStride(uchar* uv, int stride, int width, int uvheight)
{
    if(stride / 2 <= width / 2 || stride % 2 != 0) {
        return;
    }

    int halfStride = stride / 2;
    int halfSWidth = width / 2;
    for(int j = 0; j< uvheight; j++) {
        for(int i = 0; i < halfSWidth; i++) {
            *(uv + j*stride + halfSWidth + i) = *(uv + j*stride + halfStride + i);
        }
    }
}

bm_status_t hwColorCvt(Mat &srcm, OutputArray _dst,  int srcformat, int dstformat, Size dstSz, int dcn)
{
    if (!srcm.u || !srcm.u->addr) {printf("Memory allocated by user, no device memory assigned. Not support BMCV!\n"); return BM_NOT_SUPPORTED;}
    bm_handle_t handle = srcm.u->hid ? srcm.u->hid : getCard();
    if(handle == 0 || srcm.u ==0 || srcm.u->addr == 0 || (!( srcm.rows > 0 && srcm.cols > 0)))
    {
       printf("bmcv is not open or src mat is not ION \n");
       return BM_ERR_FAILURE;
    }

    int stype = srcm.type();
    int scn = CV_MAT_CN(stype), depth = CV_MAT_DEPTH(stype);

    bm_status_t ret;
    bm_image src;
    bm_device_mem_t totalmem;
    Mat  dstm;
	int outCard = _dst.getMatRef().card;
    bm_image dst;

    int id = getId(handle);
    if (outCard) id = BM_MAKEFLAG(0, BM_CARD_HEAP(outCard), id);
    if(srcformat == FORMAT_YUV420P && !srcm.avOK())
    {
        alignWithHalfStride(srcm.data+srcm.step[0]*srcm.rows*2/3,srcm.step[0],srcm.cols,srcm.rows-srcm.rows*2/3);
    }
    if (srcm.avOK())
        ret = toBMI(srcm, &src, false);
    else
        ret = matMemToBMI(srcm, &src, (bm_image_format_ext)srcformat,true);
    if (ret != BM_SUCCESS) goto done;
    dstm.allocator = hal::getAllocator();
    if ((bm_image_format_ext)dstformat == FORMAT_YUV420P)
        dstm.create(dstSz, CV_8UC1, id);
    else
        dstm.create(dstSz, CV_MAKETYPE(depth, dcn), id);
    if(dstformat == FORMAT_YUV420P )
    {
        memset(dstm.data,0,dstm.step[0]*dstm.rows);
        ret = matMemToBMI(dstm, &dst, (bm_image_format_ext)dstformat,true);
    }
    else{
        ret = matMemToBMI(dstm, &dst, (bm_image_format_ext)dstformat,false);
    }
    if (ret != BM_SUCCESS) goto done;
    bmcv_rect_t brt;
    brt.start_x = 0;
    brt.start_y = 0;
    brt.crop_w = src.width;
    brt.crop_h = src.height;

    totalmem = bm_mem_from_device(dstm.u->addr, dstm.rows*dstm.step[0]);
    ret = bm_mem_invalidate_device_mem(handle, &totalmem);
    if (ret != BM_SUCCESS) goto done;

    ret = bmcv_image_vpp_convert(handle, 1, src, &dst, &brt);
    if (ret != BM_SUCCESS) goto done;

    bm_thread_sync(handle);
    if(dstformat == FORMAT_YUV420P )
    {
        alignWithStride(dstm.data+dstm.step[0]*dstm.rows*2/3,dstm.step[0],dstm.cols,dstm.rows-dstm.rows*2/3);
    }
    _dst.assign(dstm);
done:
    bm_image_destroy(dst);
    bm_image_destroy(src);
    return ret;
}

/* if recevie resize action from  avframe Mat, will decode it to origin Mat first.
 * Note: diff with bmcv::resize, this function will filter vpp-enable
 * operation to bmcv::resize */
bm_status_t hwResize( Mat &srcm, OutputArray _dst, Size dsize, int interpolation)
{
   Size srcsize = srcm.size();

   if (!srcm.u || !srcm.u->addr) {printf("Memory allocated by user, no device memory assigned. Not support BMCV!\n"); return BM_NOT_SUPPORTED;}
   bm_handle_t handle = srcm.u->hid ? srcm.u->hid : getCard();

   /* filter out vpp-applicable case */
   if(handle == 0 ||
      (dsize.width > srcsize.width*32 || dsize.height > srcsize.height*32 ||
       dsize.width < srcsize.width/32 || dsize.height < srcsize.height/32))
   {
       printf("bmcv is not open or src mat is not ION \n");
       return BM_ERR_FAILURE;
   }

   Mat srcMat;
   bm_status_t ret;

   if(srcm.avOK() && (srcm.avFormat() == AV_PIX_FMT_NV12 ||srcm.avFormat() == AV_PIX_FMT_GRAY8
    ||srcm.avFormat() == AV_PIX_FMT_NV16 || srcm.avFormat() == AV_PIX_FMT_YUV444P))
   {
        ret= toMAT(srcm, srcMat, false);
        if (ret != BM_SUCCESS) return ret;
   }
   else if(srcm.u !=0 && srcm.u->addr != 0 &&(srcm.rows > 0 && srcm.cols > 0))
   {
      srcMat = srcm;
      uploadMat(srcMat);  // upload to physical memory
   }
   else
   {
       //printf("bmcv is not support \n");
       return BM_ERR_FAILURE;
    }

    Mat  dstm;
    int outCard = _dst.getMatRef().card;
    int id = getId(handle);
    if (outCard) id = BM_MAKEFLAG(0, BM_CARD_HEAP(outCard), id);
    dstm.allocator = hal::getAllocator();   // for CV_8UC1 case
    dstm.create(dsize, srcMat.type(), id);

    downloadMat(dstm);
    ret = resize(srcMat, dstm, false, interpolation);

    if (ret != BM_SUCCESS) goto done;

    _dst.assign(dstm);

done:
    return ret;
}
#if 0
bm_status_t hwBorder(Mat &srcm, int top, int bottom, int left, int right,OutputArray _dst,const Scalar& value)
{
   if (!srcm.u || !srcm.u->addr) {printf("Memory allocated by user, no device memory assigned. Not support BMCV!\n"); return BM_NOT_SUPPORTED;}
    bm_handle_t handle = srcm.u->hid ? srcm.u->hid : getCard();
    if(handle == 0)
    {
        printf("bmcv is not open or src mat is not ION \n");
        return BM_ERR_FAILURE;
    }
    Mat scrMat;
    bm_status_t ret;
    if(srcm.avOK() && (srcm.avFormat() == AV_PIX_FMT_NV12 ||srcm.avFormat() == AV_PIX_FMT_GRAY8
     ||srcm.avFormat() == AV_PIX_FMT_NV16 || srcm.avFormat() == AV_PIX_FMT_YUV444P))
    {
         ret= toMAT(srcm,scrMat);
         if (ret != BM_SUCCESS) return ret;
    }
    else if(srcm.u !=0 && srcm.u->addr != 0 &&(srcm.rows > 0 && srcm.cols > 0))
    {
       scrMat = srcm;
    }
    else
    {
        printf("bmcv is not support \n");
        return BM_ERR_FAILURE;
     }

     bm_image src;
     //bm_device_mem_t totalmem;
     Mat  dstm;
     bm_image dst;
     bm_image_format_ext srcformat = FORMAT_BGR_PACKED;

     if(scrMat.type() == CV_8UC1)
     {
        srcformat = FORMAT_GRAY;
     }
     else if(scrMat.type() == CV_8UC3)
     {
        srcformat = FORMAT_BGR_PACKED;
     }
     else
     {
         return BM_ERR_NOFEATURE;
     }
     ret = matMemToBMI(scrMat, &src, srcformat,true);
     if (ret != BM_SUCCESS) goto done;

     dstm.allocator = hal::getAllocator();
     dstm.create(srcm.rows + top + bottom, srcm.cols + left + right, scrMat.type(),getId(handle));
     ret = matMemToBMI(dstm, &dst, srcformat,false);
     if (ret != BM_SUCCESS) goto done;
     bmcv_copy_to_atrr_t atrr;
     atrr.start_x = left;
     atrr.start_y = top;
     atrr.if_padding = 1;
     atrr.padding_b = saturate_cast<uchar>(value.val[0]);
     atrr.padding_g = 0;
     atrr.padding_r = 0;
     if(scrMat.type() == CV_8UC3)
     {
         atrr.padding_g = saturate_cast<uchar>(value.val[1]);
         atrr.padding_r = saturate_cast<uchar>(value.val[2]);
     }

     ret = download(handle, dstm, dst);
     if (ret != BM_SUCCESS) goto done;

     ret = bmcv_copy_to(handle, atrr, src, dst);
     if (ret != BM_SUCCESS) goto done;

     bm_thread_sync(handle);
     _dst.assign(dstm);

 done:
     bm_image_destroy(dst);
     bm_image_destroy(src);
     return ret;
}
#endif
bm_status_t hwCrop(InputArray _src, Rect rect , OutputArray _dst)
{
    Mat srcm, scrMat;
    srcm = _src.getMat();
    if (!srcm.u || !srcm.u->addr) {printf("Memory allocated by user, no device memory assigned. Not support BMCV!\n"); return BM_NOT_SUPPORTED;}
    bm_handle_t handle = srcm.u->hid ? srcm.u->hid : getCard();
    if(handle == 0)
    {
        printf("bmcv is not open or src mat is not ION \n");
        return BM_ERR_FAILURE;
    }
    if(rect.x + rect.width > srcm.cols || rect.y + rect.height> srcm.rows)
    {
        printf("crop region is bigger than src Mat\n");
        return BM_ERR_FAILURE;
    }
    bm_status_t ret;
    if(srcm.avOK() && (srcm.avFormat() == AV_PIX_FMT_NV12 ||srcm.avFormat() == AV_PIX_FMT_GRAY8
     ||srcm.avFormat() == AV_PIX_FMT_NV16 || srcm.avFormat() == AV_PIX_FMT_YUV444P))
    {
         ret= toMAT(srcm,scrMat);
         if (ret != BM_SUCCESS) return ret;
    }
    else if(srcm.u !=0 && srcm.u->addr != 0 &&(srcm.rows > 0 && srcm.cols > 0))
    {
       scrMat = srcm;
    }
    else
    {
        //printf("bmcv is not support your input format\n");
        return BM_ERR_FAILURE;
     }

     bm_image src;
     bm_device_mem_t totalmem;
     Mat  dstm;
     bm_image dst;
     bm_image_format_ext srcformat = FORMAT_BGR_PACKED;
	 int outCard = _dst.getMatRef().card;
     int id = getId(handle);
     if(outCard) id = BM_MAKEFLAG(0, BM_CARD_HEAP(outCard), id);

     if(scrMat.type() == CV_8UC1)
     {
        srcformat = FORMAT_GRAY;
     }
     else if(scrMat.type() == CV_8UC3)
     {
        srcformat = FORMAT_BGR_PACKED;
     }
     else
     {
         return BM_ERR_NOFEATURE;
     }
     ret = matMemToBMI(scrMat, &src, srcformat,true);
     if (ret != BM_SUCCESS) goto done;

     dstm.allocator = hal::getAllocator();
     dstm.create(rect.height,rect.width,scrMat.type(),id);
     ret = matMemToBMI(dstm, &dst, srcformat,false);
     if (ret != BM_SUCCESS) goto done;
     bmcv_rect_t brt;
     brt.start_x = rect.x;
     brt.start_y = rect.y;
     brt.crop_w = rect.width;
     brt.crop_h = rect.height;

     totalmem = bm_mem_from_device(dstm.u->addr, dstm.rows*dstm.step[0]);
     ret = downloadDeviceData(handle, dstm.data, totalmem);
     if (ret != BM_SUCCESS) goto done;
     ret = bmcv_image_vpp_convert(handle, 1, src, &dst, &brt);
     if (ret != BM_SUCCESS) goto done;

     bm_thread_sync(handle);
     _dst.assign(dstm);

 done:
     bm_image_destroy(dst);
     bm_image_destroy(src);
     return ret;
}

bm_status_t hwMultiCrop(InputArray _src, std::vector<Rect>& loca, std::vector<Mat>& dst_vector)
{
    Mat srcm, scrMat;
    srcm = _src.getMat();
    if (!srcm.u || !srcm.u->addr) {printf("Memory allocated by user, no device memory assigned. Not support BMCV!\n"); return BM_NOT_SUPPORTED;}
    bm_handle_t handle = srcm.u->hid ? srcm.u->hid : getCard();
    if(handle == 0)
    {
        printf("bmcv is not open or src mat is not ION \n");
        return BM_ERR_FAILURE;
    }
    bm_status_t ret;
    if(srcm.avOK() && (srcm.avFormat() == AV_PIX_FMT_NV12 ||srcm.avFormat() == AV_PIX_FMT_GRAY8
     ||srcm.avFormat() == AV_PIX_FMT_NV16 || srcm.avFormat() == AV_PIX_FMT_YUV444P))
    {
         ret= toMAT(srcm,scrMat);
         if (ret != BM_SUCCESS) return ret;
    }
    else if(srcm.u !=0 && srcm.u->addr != 0 &&(srcm.rows > 0 && srcm.cols > 0))
    {
       scrMat = srcm;
    }
    else
    {
        //printf("bmcv is not support your input format\n");
        return BM_ERR_FAILURE;
    }
     bm_image src;
     bm_device_mem_t totalmem;
     bm_image* dst = new bm_image[loca.size()];
     bm_image_format_ext srcformat = FORMAT_BGR_PACKED;
     bmcv_rect_t* brt = new bmcv_rect_t[loca.size()];
     if(scrMat.type() == CV_8UC1)
     {
        srcformat = FORMAT_GRAY;
     }
     else if(scrMat.type() == CV_8UC3)
     {
        srcformat = FORMAT_BGR_PACKED;
     }
     else
     {
         return BM_ERR_NOFEATURE;
     }
     ret = matMemToBMI(scrMat, &src, srcformat,true);
     if (ret != BM_SUCCESS) goto done;

     if (dst_vector.size() == 0) {
        for (unsigned int i = 0; i < loca.size(); i++) {
            Mat ma(loca[i].height, loca[i].width, scrMat.type(), SophonDevice(getId(handle)));
            dst_vector.push_back(ma);
        }
     } else {
        for (unsigned int i = 0; i < loca.size(); i++) {
            int id = getId(handle);
            if (dst_vector[i].card) id = BM_MAKEFLAG(0, BM_CARD_HEAP(dst_vector[i].card), id);

            if (dst_vector[i].empty())
                dst_vector[i].create(loca[i].height, loca[i].width, scrMat.type(), id);
        }
    }

     for (size_t i = 0; i < loca.size(); i++) {
        ret = matMemToBMI(dst_vector[i], &dst[i], srcformat, false);
        if (ret != BM_SUCCESS) goto done;
        brt[i].start_x = loca[i].x;
        brt[i].start_y = loca[i].y;
        brt[i].crop_w = loca[i].width;
        brt[i].crop_h = loca[i].height;
    }
    for (size_t i = 0; i < loca.size(); i++) {
        totalmem = bm_mem_from_device(dst_vector[i].u->addr, dst_vector[i].rows*dst_vector[i].step[0]);
        ret = downloadDeviceData(handle, dst_vector[i].data, totalmem);
		if (ret != BM_SUCCESS) goto done;
    }
    ret = bmcv_image_vpp_convert(handle, loca.size(), src, dst, brt);
    if (ret != BM_SUCCESS) goto done;

    bm_thread_sync(handle);

 done:
   bm_image_destroy(src);
   for (size_t i = 0; i < loca.size(); i++) {
       bm_image_destroy(dst[i]);
   }
   delete dst;
   delete brt;
   return ret;
}

#endif

bm_status_t hwSobel(InputArray _src, OutputArray _dst, int dx, int dy, int ksize, double scale, double delta)
{
    Mat srcm, scrMat, srcu, dstm;
    bm_image src;
    bm_image dst;

    srcm = _src.getMat();
    dstm = _dst.getMat();

    if ((srcm.type() != CV_8UC1) || (dstm.type() != CV_8UC1)) {printf("bmcv mat type is not CV_8UC1. Not support BMCV!\n"); return BM_NOT_SUPPORTED;}
    if (!srcm.u) {
        srcm.copyTo(srcu);
        if (!srcu.u) {
           printf("srcm copyto scrMat but u is null\n");
           return  BM_NOT_SUPPORTED;
        }
    }else {
      srcu = srcm;
    }

    bm_handle_t handle = srcu.u->hid ? srcu.u->hid : getCard();
    if(handle == 0)
    {
        printf("bmcv is not open or src mat is not ION \n");
        return BM_ERR_FAILURE;
    }

    bm_status_t ret;
    if(srcu.avOK() && (srcu.avFormat() == AV_PIX_FMT_NV12 ||srcu.avFormat() == AV_PIX_FMT_GRAY8
     ||srcu.avFormat() == AV_PIX_FMT_NV16 ||srcu.avFormat() == AV_PIX_FMT_NV21
     || srcu.avFormat() == AV_PIX_FMT_YUV422P || srcu.avFormat() == AV_PIX_FMT_YUV444P
     || srcu.avFormat() == AV_PIX_FMT_YUV420P))
    {
         ret= toMAT(srcu,scrMat);
         if (ret != BM_SUCCESS) return ret;
    }
    else if(srcu.u !=0 && srcu.u->addr != 0 &&(srcu.rows > 0 && srcu.cols > 0))
    {
       scrMat = srcu;
    }
    else
    {
          //printf("bmcv is not support your input format\n");
          return BM_ERR_FAILURE;
    }

    int id = getId(handle);
    ret = toBMI(scrMat, &src, true);
    if (ret != BM_SUCCESS) return ret;

    Mat outm;
    int sync_flag = 0;
    if (dstm.empty()||!dstm.u||!dstm.u->addr) {
        sync_flag = 1;
        outm.create(src.height, src.width, CV_8UC1, id);
    } else {
        sync_flag = 0;
        outm = dstm;
    }
    ret = toBMI(outm, &dst, false);
    if (ret != BM_SUCCESS) return ret;

#ifdef USING_SOC
    download(handle, outm, &dst);  // clear cache at soc mode, ugly but have to
#endif

    ret = bmcv_image_sobel(handle, src, dst, dx, dy, ksize, float(scale), float(delta));
    if (ret != BM_SUCCESS) goto done;
    bm_thread_sync(handle);
#ifndef USING_SOC
    download(handle, outm, &dst);
#endif
    if (sync_flag) outm.copyTo(dstm);

 done:
     bm_image_destroy(dst);
     bm_image_destroy(src);
     return ret;
}

bm_status_t hwGaussianBlur(InputArray _src, OutputArray _dst, int kw, int kh, double sigma1, double sigma2)
{
     Mat srcm, scrMat, srcu, dstm;
     bm_image src;
     bm_image dst;

     srcm = _src.getMat();
     dstm = _dst.getMat();

     if ((srcm.type() != CV_8UC1) || (dstm.type() != CV_8UC1)) {printf("bmcv mat type is not CV_8UC1 or CV_8UC3. Not support BMCV!\n"); return BM_NOT_SUPPORTED;}
     if (!srcm.u) {
        srcm.copyTo(srcu);
        if (!srcu.u) {
           printf("srcm copyto scrMat but u is null\n");
           return  BM_NOT_SUPPORTED;
	}
     }else {
       srcu = srcm;
     }

     bm_handle_t handle = srcu.u->hid ? srcu.u->hid : getCard();
     if(handle == 0)
     {
        printf("bmcv is not open or src mat is not ION \n");
        return BM_ERR_FAILURE;
     }

     bm_status_t ret;
     if(srcu.avOK() && (srcu.avFormat() == AV_PIX_FMT_NV12 ||srcu.avFormat() == AV_PIX_FMT_GRAY8
        ||srcu.avFormat() == AV_PIX_FMT_NV16 ||srcu.avFormat() == AV_PIX_FMT_NV21
        || srcu.avFormat() == AV_PIX_FMT_YUV422P || srcu.avFormat() == AV_PIX_FMT_YUV444P
        || srcu.avFormat() == AV_PIX_FMT_YUV420P))
     {
        ret= toMAT(srcu,scrMat);
        if (ret != BM_SUCCESS) return ret;
     }
     else if(srcu.u !=0 && srcu.u->addr != 0 &&(srcu.rows > 0 && srcu.cols > 0))
     {
        scrMat = srcu;
     }
     else
     {
       //printf("bmcv is not support your input format\n");
       return BM_ERR_FAILURE;
     }

    int id = getId(handle);
    ret = toBMI(scrMat, &src, true);
    if (ret != BM_SUCCESS) return ret;

    Mat outm;
    int sync_flag = 0;
    if (dstm.empty()||!dstm.u||!dstm.u->addr) {
        sync_flag = 1;
        outm.create(src.height, src.width, srcm.type(), id);
    } else {
        sync_flag = 0;
        outm = dstm;
    }
    ret = toBMI(outm, &dst, false);
    if (ret != BM_SUCCESS) return ret;
#ifdef USING_SOC
    download(handle, outm, &dst);  // clear cache at soc mode, ugly but have to
#endif

    ret = bmcv_image_gaussian_blur(handle, src, dst, kw, kh, float(sigma1), float(sigma2));
    if (ret != BM_SUCCESS) goto done;
    bm_thread_sync(handle);
#ifndef USING_SOC
    download(handle, outm, &dst);
#endif
    if (sync_flag) outm.copyTo(dstm);

done:
    bm_image_destroy(dst);
    bm_image_destroy(src);
    return ret;
}

}}

#endif
