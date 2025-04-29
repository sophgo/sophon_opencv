#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgproc/vpp.hpp"

#ifndef WIN32
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <fcntl.h>
#include <pthread.h>
#include <iostream>
#include <typeinfo>

//#ifdef USING_SOC
using namespace std;

namespace cv {
#define STRIDE_ALIGN    (64)
#define ALIGN(x, mask)  (((x) + ((mask)-1)) & ~((mask)-1))

class SettingLock
{
public:
    SettingLock() { pthread_mutex_init(&settinLock,0); }
    ~SettingLock() { pthread_mutex_destroy(&settinLock); }
    void lock() { pthread_mutex_lock(&settinLock); }
    void unlock() { pthread_mutex_unlock(&settinLock); }
private:
    pthread_mutex_t settinLock;
};


namespace vpp {
void vpp_parameter_dump(struct vpp_batch *batch);
static void crop_internal(InputArray src, std::vector<Rect>& loca, InputOutputArray dst);
static void resize_internal(InputArray src, InputOutputArray dst);
static void border_internal(InputArray src, int top, int bottom, int left, int right, OutputArray dst);
static void split_internal(InputArray src, InputOutputArray dst);
static void alignWithHalfStride(uchar* uv, int stride, int width, int uvheight);

enum dump_type
{
    DUMP_MAT = 0,
    DUMP_IML = 1
};

enum dump_dir
{
    DUMP_IN  = 0,
    DUMP_OUT = 1
};
static bool IsEnableVPP = true;
static SettingLock settinglock;

static void vpp_dump_mat( char * strFunc, int dir, Mat& img, int dump_num, int index);
//static void vpp_dump_iplImage( char * strFunc, int dir, IplImage* img, int dump_num, int fd );
static void vpp_dump( char * strFunc, int dir, void* img ,int dump_num,  int dump_type, int index);

static int resize_dump_num       = 0;
static int crop_dump_num         = 0;
static int split_dump_num        = 0;
static int border_dump_num       = 0;
static int iplImage_dump_num     = 0;
static int cvtColor1682_dump_num = 0;


 CV_EXPORTS void setDump(int dumpNum)
 {
    resize_dump_num       = dumpNum;
    crop_dump_num         = dumpNum;
    split_dump_num        = dumpNum;
    border_dump_num       = dumpNum;
    iplImage_dump_num     = dumpNum;
    cvtColor1682_dump_num = dumpNum;
 }

template <class T>
bool isIONMat(T & mat)
{
    if( (mat.u && (mat.u->fd >= 0) && (mat.u->size > 0) && (mat.u->addr > 0)))
    {
        return true;
    }
    return false;
}

template <class T>
T checkSrcMat(T & mat)
{
    if (isIONMat(mat)== false)
    {
        T hdr1;
        hdr1.allocator = hal::getAllocator();
        mat.copyTo(hdr1);
        return hdr1;
    }
    else
    {
        return mat;
    }
}

static void vpp_dump( char * strFunc, int dir, void* img ,int dump_num, int dump_type,int index = 0)
{
    if(dump_num > 0)
    {
        if(dump_type ==  DUMP_MAT)
        {
            vpp_dump_mat(strFunc, dir, (*(Mat*)img), dump_num, index);
        }
        else
        {
           //vpp_dump_iplImage(strFunc, dir, (IplImage*)img, dump_num, index);
        }
    }
}

static void vpp_dump_mat( char * strFunc, int dir, Mat&  img, int dump_num, int index)
 {
    FILE *fout = NULL;
    char filename[255];
    string strdir;
    strdir = (dir == DUMP_IN) ? "in": "out";
    pthread_t tid = pthread_self();
    sprintf(filename, "/data/pic%d_%s_%dx%d_%u%s_%d.bin", dump_num, strFunc, img.cols, img.rows, (unsigned int)tid, strdir.c_str(), index);
    fout = fopen(filename, "wb");
    if(fout != NULL)
    {
        const uint8_t* bgr = img.data;
        int stride_rgb = img.step[0];
                int channels = img.channels();
        if( channels == 1 || channels == 3)
        {
            for(int i = 0; i < img.rows; i++)
            {
                fwrite( (uint8_t *)(bgr), 1, img.cols*channels, fout);
                bgr += (size_t)(stride_rgb);
            }
        }
        else
        {
            printf("warnning:dump_jpu_mat channels=%d\n", channels);
        }
        if(NULL!=fout)
            fclose(fout);
    }
 }

#if 0
static void vpp_dump_iplImage( char * strFunc, int dir, IplImage* img, int dump_num, int fd)
{
    FILE *fout = NULL;
    char filename[255];
    string strdir;

    if(img == NULL)
        return;

    strdir = (dir == DUMP_IN) ? "in": "out";
    pthread_t tid = pthread_self();
    sprintf(filename, "/data/pic%d_%s_%dx%d_%u%s.bin", dump_num, strFunc, img->width, img->height, (unsigned int)tid,strdir.c_str());
    fout = fopen(filename, "wb");
    if(fout != NULL)
    {
        char*  addr_y = img->imageData;

        for(int i = 0; i < img->height; i++)
        {
            fwrite( (uint8_t *)(addr_y), 1, img->width, fout);
            addr_y += img->step;
        }/*
        // const uint8_t* addr_u = cmd->src_addr1;
        for(int i = 0; i < cmd->src_cropH/2; i++)
        {
            fwrite( (uint8_t *)(virt_addrY), 1, cmd->src_cropW/2, fout);
            virt_addrY += cmd->src_stride/2;
        }
        // const uint8_t* addr_v = cmd->src_addr2;
        for(int i = 0; i < cmd->src_cropH/2; i++)
        {
            fwrite( (uint8_t *)(virt_addrY), 1, cmd->src_cropW/2, fout);
            virt_addrY += cmd->src_stride/2;
        } */
        fclose(fout);
        fout = NULL;
    }
}
#endif
void vpp_parameter_dump(struct vpp_batch *batch)
{
#if defined VPP_BM1684
    struct vpp_cmd *cmd;
    for (int i = 0; i < batch->num; i++) {
        cmd = &(batch->cmd[i]);
        cout << "batch->num is  " << batch->num << endl;
        cout << "cmd id is  " << i << endl;
        cout << "cmd->src_format " << cmd->src_format << endl;
        cout << "cmd->src_stride " << cmd->src_stride << endl;
        cout << "cmd->src_endian " << cmd->src_endian << endl;
        cout << "cmd->src_endian_a " << cmd->src_endian_a << endl;
        cout << "cmd->src_plannar " << cmd->src_plannar << endl;
        cout << "cmd->src_fd0 " << cmd->src_fd0 << endl;
        cout << "cmd->src_fd1 " << cmd->src_fd1 << endl;
        cout << "cmd->src_fd2 " << cmd->src_fd2 << endl;
        cout << "cmd->src_addr0 " << cmd->src_addr0 << endl;
        cout << "cmd->src_addr1 " << cmd->src_addr1 << endl;
        cout << "cmd->src_addr2 " << cmd->src_addr2 << endl;
        cout << "cmd->src_axisX " << cmd->src_axisX << endl;
        cout << "cmd->src_axisY " << cmd->src_axisY << endl;
        cout << "cmd->src_cropW " << cmd->src_cropW << endl;
        cout << "cmd->src_cropH " << cmd->src_cropH << endl;
        cout << "cmd->dst_format " << cmd->dst_format << endl;
        cout << "cmd->dst_stride " << cmd->dst_stride << endl;
        cout << "cmd->dst_endian " << cmd->dst_endian << endl;
        cout << "cmd->dst_endian_a " << cmd->dst_endian_a << endl;
        cout << "cmd->dst_plannar " << cmd->dst_plannar << endl;
        cout << "cmd->dst_fd0 " << cmd->dst_fd0 << endl;
        cout << "cmd->dst_fd1 " << cmd->dst_fd1 << endl;
        cout << "cmd->dst_fd2 " << cmd->dst_fd2 << endl;
        cout << "cmd->dst_addr0 " << cmd->dst_addr0 << endl;
        cout << "cmd->dst_addr1 " << cmd->dst_addr1 << endl;
        cout << "cmd->dst_addr2 " << cmd->dst_addr2 << endl;
        cout << "cmd->dst_axisX " << cmd->dst_axisX << endl;
        cout << "cmd->dst_axisY " << cmd->dst_axisY << endl;
        cout << "cmd->dst_cropW " << cmd->dst_cropW << endl;
        cout << "cmd->dst_cropH " << cmd->dst_cropH << endl;
        cout << "cmd->src_csc_en " << cmd->src_csc_en << endl;
        cout << "cmd->hor_filter_sel " << cmd->hor_filter_sel << endl;
        cout << "cmd->ver_filter_sel " << cmd->ver_filter_sel << endl;
        cout << "cmd->scale_x_init " << cmd->scale_x_init << endl;
        cout << "cmd->scale_y_init " << cmd->scale_y_init << endl;
        cout << "cmd->csc_type " << cmd->csc_type << endl;
        cout << "cmd->mapcon_enable " << cmd->mapcon_enable << endl;
        cout << "cmd->src_fd3 " << cmd->src_fd3 << endl;
        cout << "cmd->src_addr3 " << cmd->src_addr3 << endl;
        cout << "cmd->cols " << cmd->cols << endl;
        cout << "cmd->rows " << cmd->rows << endl;
    }
#endif
    return;
}

void enableVPPConverter( bool isEnabled )
{
    settinglock.lock();

    if(isEnabled) {
        IsEnableVPP = true;
    } else {
        IsEnableVPP = false;
    }

    settinglock.unlock();
 }

bool IsVPPConverterEnabled()
{
    settinglock.lock();

    if(IsEnableVPP) {
        settinglock.unlock();
        return true;
    }

    settinglock.unlock();
    return false;
}
#if defined(ION_CACHE) && defined(VPP_BM1682)
/* for src Mat data is aligned with stride -> vpp u / v data is aligned with stride/2 */
static void alignWithHalfStride(uchar* uv, int stride, int width, int uvheight)
{
    if (stride/2 <= width/2 || stride%2 != 0)
        return;

    int halfStride = stride / 2;
    int halfSWidth = width / 2;

    for(int j = 0; j < uvheight; j++) {
        for(int i = halfSWidth-1; i >= 0; i--) {
            *(uv + j * stride + halfStride + i) = *(uv + j * stride + halfSWidth + i);
        }
    }
}
#endif

 /* for dst in vpp u / v data is aligned with stride/2 -> Mat data is aligned with stride */
#if 0
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
#endif

/* For 1682, only support BGR->RGB 420/nv12->RGB/BGR  only support cv8U had been restricted in color.cpp*/
bool vppCvtColor(Mat& src, Mat& dst, int srcFmt, int dstFmt)
{
#if defined(ION_CACHE) && defined(VPP_BM1682)

    int support_case = 0;

    if (CV_MAT_DEPTH(src.type()) == CV_8U && srcFmt == FMT_SRC_BGR && dstFmt == FMT_DST_RGB)
        support_case = 1;
    if ((srcFmt == FMT_SRC_I420 || srcFmt == FMT_SRC_NV12) &&
        (dstFmt == FMT_DST_RGB || dstFmt == FMT_DST_BGR))
        support_case = 1;

    if (support_case==0)
        return false;

    if (isIONMat(dst)== false || dst.isSubmatrix())
    {
        return false;
    }

    Mat tmpsrc = checkSrcMat(src);
    int scn = tmpsrc.channels();
    if(((tmpsrc.cols > MAX_RESOLUTION_W) || (tmpsrc.rows > MAX_RESOLUTION_H)) ||
       ((tmpsrc.cols < MIN_RESOLUTION_W) || (tmpsrc.rows < MIN_RESOLUTION_H)) ||
       ((int)tmpsrc.step[0] < ALIGN(tmpsrc.cols*scn, STRIDE_ALIGN)) ) {
        return false;
    }

    if(dstFmt == FMT_DST_RGB || dstFmt == FMT_DST_BGR ) {
        int dcn = dst.channels();
        if(((int)dst.step[0] < ALIGN(dst.cols*dcn, STRIDE_ALIGN)) ||
           ((dst.step.p[0] % STRIDE_ALIGN) != 0))
            return false;
    }

    struct vpp_batch batch;
    memset(&batch, 0, sizeof(batch));  /* init batch cmd*/
    batch.num = 1;
    struct vpp_cmd *cmd = &batch.cmd[0];

    cmd->dst_format = dstFmt;
    cmd->src_format = srcFmt;
    cmd->dst_stride = dst.step.p[0];
    cmd->src_stride = tmpsrc.step.p[0];
    cmd->dst_cropW = dst.cols;
    cmd->src_cropW = tmpsrc.cols;
    cmd->dst_cropH = dst.rows;
    cmd->src_cropH = tmpsrc.rows;
    cmd->dst_fd0 = dst.u->fd;
    cmd->src_fd0 = tmpsrc.u->fd;

    switch(srcFmt)
    {
    case FMT_SRC_BGR:
        break;
    case FMT_SRC_I420:
        if(cmd->src_stride > cmd->src_cropW)
            alignWithHalfStride(tmpsrc.data + tmpsrc.step[0] * (cmd->src_cropH * 2 / 3), cmd->src_stride,
                                cmd->src_cropW, tmpsrc.rows-(cmd->src_cropH * 2 / 3));
        cmd->csc_type = 0; /* TODO 0 YCbcCr -> RGB, 1 YPbPr -> RGB, */
        break;
    case FMT_SRC_NV12:
        cmd->csc_type = 0; /* TODO 0 YCbcCr -> RGB, 1 YPbPr -> RGB, */
        break;
    default:
        break;
    }

    int vppFd = open("/dev/bm-vpp", O_RDWR);
    if (vppFd < 0) {
        CV_Error(CV_VppOpenErr, "open /dev/bm-vpp failed");
        return false;
    }

    int ret = ioctl(vppFd, VPP_UPDATE_BATCH, &batch);
    if (ret < 0)
    {
        CV_Error(CV_VppOpenErr, "ioctl falied\n");
        close(vppFd);
        return false;
    }

    close(vppFd);
    return true;
#else
    printf("opencv can't support vpp in ion no cache and no 1682 vpp mode\n");
    return false;    // can't support in nocache mode
#endif
}

/* python API */
void resize(InputArray _src, CV_OUT UMat& _dst)
{
    UMat src;
    Size dsize = _dst.size();
    int dtype = _dst.type();

    if (_src.isMat()){
        src.allocator = hal::getAllocator();
        _src.copyTo(src);
    } else if (_src.isUMat())
        src = _src.getUMat();

    if (_dst.empty() == 0){
        _dst.release();
    }
    _dst.allocator = hal::getAllocator();
    _dst.create(dsize, dtype);

    resize_internal(src, _dst);
}

void crop(InputArray _src, std::vector<Rect>& _loca, CV_OUT std::vector<UMat>& _dst)
{
    _InputArray src_array;
    UMat src;
    std::vector<UMat> src_vector;

    if (_src.isMat()){
        src.allocator = hal::getAllocator();
        _src.copyTo(src);
        src_array = _InputArray(src);
    }else if (_src.isMatVector()){
        for (size_t i = 0; i < _loca.size(); i++){
            UMat um;

            um.allocator = hal::getAllocator();
            _src.getMat(i).copyTo(um);
            src_vector.push_back(um);
        }
        src_array = _InputArray(src_vector);
    }
    else
        src_array = _src;

    crop_internal(src_array, _loca, _dst);

    src_vector.clear();
}

void border(InputArray src, int top, int bottom, int left, int right, CV_OUT UMat& dst)
{
    UMat src_umat;

    if (src.isMat()){
        src_umat.allocator = hal::getAllocator();
        src.copyTo(src_umat);
    }
    else if (src.isUMat())
        src_umat = src.getUMat();

    border_internal(src_umat, top, bottom, left, right, dst);
    //usleep(10000);
}

void split(InputArray src, CV_OUT std::vector<UMat>& dst)
{
    UMat src_umat;

    if (src.isMat()){
        src_umat.allocator = hal::getAllocator();
        src.copyTo(src_umat);
    }
    else if (src.isUMat())
        src_umat = src.getUMat();

    split_internal(src_umat, dst);
}

void split(InputArray src, CV_OUT UMat& dst)
{
    UMat src_umat;
    Size dsize = dst.size();
    int dtype = dst.type();

    if (src.isMat()){
        src_umat.allocator = hal::getAllocator();
        src.copyTo(src_umat);
    }
    else if (src.isUMat())
        src_umat = src.getUMat();

    if (dst.empty() == 0){
        dst.release();    // python allocated memory is not HW continuous
    }
    dst.allocator = hal::getAllocator();
    dst.create(dsize, dtype);

    split_internal(src_umat, dst);
}

/* C++ API */

void resize(Mat& src, Mat& dst)
{
    resize_internal(src, dst);
    if(resize_dump_num > 0)
    {
        vpp_dump((char*)"resize", DUMP_IN, &src, resize_dump_num, DUMP_MAT);
        vpp_dump((char*)"resize", DUMP_OUT, &dst, resize_dump_num, DUMP_MAT);
        resize_dump_num--;
    }
}

void crop(Mat& src, std::vector<Rect>& loca, Mat* dst)
{
    std::vector<Mat> mat_vec;
    size_t i;

    crop_internal(src, loca, mat_vec);

    for (i = 0; i < loca.size(); i++)
        dst[i] = mat_vec[i];

    mat_vec.clear();

    if(crop_dump_num > 0)
    {
        vpp_dump((char*)"crop", DUMP_IN, &src, crop_dump_num, DUMP_MAT);
        for (i = 0; i < loca.size(); i++)
            vpp_dump((char*)"crop", DUMP_OUT, &dst[i], crop_dump_num, DUMP_MAT, i);
        crop_dump_num--;
    }
}

void crop(Mat* src, std::vector<Rect>& loca, Mat* dst)
{
    std::vector<Mat> src_vec;
    std::vector<Mat> dst_vec;
    size_t i;

    for (i = 0; i < loca.size(); i++)
        src_vec.push_back(src[i]);

    crop_internal(src_vec, loca, dst_vec);

    for (i = 0; i < loca.size(); i++)
        dst[i] = dst_vec[i];

    src_vec.clear();
    dst_vec.clear();

    if(crop_dump_num > 0)
    {
        vpp_dump((char*)"crop", DUMP_IN, &src, crop_dump_num, DUMP_MAT);
        for (i = 0; i < loca.size(); i++)
        {
            vpp_dump((char*)"crop", DUMP_IN, &src[i], crop_dump_num, DUMP_MAT, i);
            vpp_dump((char*)"crop", DUMP_OUT, &dst[i], crop_dump_num, DUMP_MAT, i);
        }
        crop_dump_num--;
    }
}

Mat border(Mat& src, int top, int bottom, int left, int right)
{
    Mat dst_mat;

    border_internal(src, top, bottom, left, right, dst_mat);

    if(border_dump_num > 0)
    {
        vpp_dump((char*)"border", DUMP_IN, &src, border_dump_num, DUMP_MAT);
        vpp_dump((char*)"border", DUMP_OUT, &dst_mat, border_dump_num, DUMP_MAT);
        border_dump_num--;
    }
    return dst_mat;
}

void split(Mat& src, Mat* dst)
{
    std::vector<Mat> mat_vec;
    size_t i;

    split_internal(src, mat_vec);

    for (i = 0; i < mat_vec.size(); i++)
        dst[i] = mat_vec[i];

    mat_vec.clear();

    if(split_dump_num > 0)
    {
        vpp_dump((char*)"split", DUMP_IN, &src, split_dump_num, DUMP_MAT);
        for (i = 0; i < mat_vec.size(); i++)
            vpp_dump((char*)"split", DUMP_OUT, &dst[i], split_dump_num, DUMP_MAT, i);
        split_dump_num--;
    }

}

void split(Mat& src, Mat& dst)
{
    split_internal(src, dst);

    if(split_dump_num > 0)
    {
        vpp_dump((char*)"split", DUMP_IN, &src, split_dump_num, DUMP_MAT);
        vpp_dump((char*)"split", DUMP_OUT, &dst, split_dump_num, DUMP_MAT);
        split_dump_num--;
    }
}

#ifndef ION_CACHE
void split(Mat& src, unsigned long addr0, unsigned long addr1, unsigned long addr2)
{

    CV_Assert(src.channels() == 3);
    CV_Assert(CV_MAT_DEPTH(src.type()) == CV_8U);
    CV_Assert((addr0 != 0) && (addr1 != 0) && (addr2 != 0));
#if (defined VPP_BM1880) || (defined VPP_BM1684)
    CV_Assert((src.cols <= MAX_RESOLUTION_W) && (src.rows <= MAX_RESOLUTION_H));
    CV_Assert((src.cols >= MIN_RESOLUTION_W_LINEAR) && (src.rows >= MIN_RESOLUTION_H_LINEAR));
#else
    CV_Assert((src.cols <= MAX_RESOLUTION_W) && (src.rows <= MAX_RESOLUTION_H));
    CV_Assert((src.cols >= MIN_RESOLUTION_W) && (src.rows >= MIN_RESOLUTION_H));
#endif

    int stride = src.cols;
    if (stride % 16) {
        stride = (stride / 16 + 1) * 16;
    }

    int fd = open("/dev/bm-vpp", O_RDWR);
    if (fd < 0) {
        CV_Error(CV_VppOpenErr, "open /dev/bm-vpp failed");
        return;
    }

    struct vpp_batch batch;
    memset(&batch,0,sizeof(batch));
    batch.num = 1;

    struct vpp_cmd *cmd = &batch.cmd[0];
#if (defined VPP_BM1880) || (defined VPP_BM1684)
    cmd->src_format = RGB24;
    cmd->src_endian_a = 0;
    cmd->src_endian = 1;
    cmd->src_plannar = 0;
#if !defined VPP_BM1684
    cmd->tile_mode = 0;
#endif
    cmd->src_stride = src.step.p[0];
    cmd->src_addr0 = src.u->addr;
    cmd->src_addr1 = 0;
    cmd->src_addr2 = 0;
    cmd->src_cropH = src.rows;
    cmd->src_cropW = src.cols;
    cmd->src_axisX = 0;
    cmd->src_axisY = 0;

    cmd->dst_format = RGB24;
    cmd->dst_endian_a = 0;
    cmd->dst_endian = 0;
    cmd->dst_plannar = 1;
    cmd->dst_stride = stride;
    cmd->dst_addr0 = addr0;
    cmd->dst_addr1 = addr1;
    cmd->dst_addr2 = addr2;
    cmd->dst_cropH = src.rows;
    cmd->dst_cropW = src.cols;
    cmd->dst_axisX = 0;
    cmd->dst_axisY = 0;

    /*scl ctr*/
    cmd->hor_filter_sel = 2;
    cmd->ver_filter_sel = 2;
    cmd->scale_x_init = 0;
    cmd->scale_y_init = 0;
#else
    cmd->src_format = FMT_SRC_BGR;
    cmd->src_stride = src.step.p[0];
    cmd->src_addr0 = src.u->addr;
    cmd->src_addr1 = 0;
    cmd->src_addr2 = 0;
    cmd->src_cropH = src.rows;
    cmd->src_cropW = src.cols;
    cmd->src_axisX = 0;
    cmd->src_axisY = 0;

    cmd->dst_format = FMT_DST_RGBP;
    cmd->dst_stride = stride;
    cmd->dst_addr0 = addr0;
    cmd->dst_addr1 = addr1;
    cmd->dst_addr2 = addr2;
    cmd->dst_cropH = src.rows;
    cmd->dst_cropW = src.cols;
    cmd->dst_axisX = 0;
    cmd->dst_axisY = 0;
#endif
    int ret = ioctl(fd, VPP_UPDATE_BATCH_NON_CACHE, &batch);
    if (ret < 0) {
        cout << "errno is " << errno << endl;
        vpp_parameter_dump(&batch);
        CV_Error(CV_VppIoctlErr, "ioctl split failed");
    }

    close(fd);
}
#endif

/* internal implementation API */
static void resize_internal(InputArray src, InputOutputArray dst)
{
    CV_Assert((src.isMat() && dst.isMat()) || (src.isUMat() && dst.isUMat()));
    CV_Assert(CV_MAT_DEPTH(src.type()) == CV_8U);
    UMatData *src_u, *dst_u;
    int src_cols = src.cols();
    int src_rows = src.rows();
    int dst_cols = dst.cols();
    int dst_rows = dst.rows();
    size_t src_step = src.step();
    size_t dst_step = dst.step();
    UMat uSrcM;
    Mat  srcM;
    if(src.isMat()){
        srcM = src.getMat();
        srcM = checkSrcMat(srcM);
        src_u = srcM.u;
        dst_u = dst.getMat().u;
    }
    else {
        uSrcM = src.getUMat();
        uSrcM = checkSrcMat(uSrcM);
        src_u = uSrcM.u;
        dst_u = dst.getUMat().u;
    }

    CV_Assert(dst.empty() == 0);

    CV_Assert((src_cols <= MAX_RESOLUTION_W) && (src_rows <= MAX_RESOLUTION_H));
    CV_Assert((dst_cols <= MAX_RESOLUTION_W) && (dst_rows <= MAX_RESOLUTION_H));
#if (defined VPP_BM1880) || (defined VPP_BM1684)
    CV_Assert((src_cols >= MIN_RESOLUTION_W_LINEAR) && (src_rows >= MIN_RESOLUTION_H_LINEAR));
    CV_Assert((dst_cols >= MIN_RESOLUTION_W_LINEAR) && (dst_rows >= MIN_RESOLUTION_H_LINEAR));
#else
    CV_Assert((src_cols >= MIN_RESOLUTION_W) && (src_rows >= MIN_RESOLUTION_H));
    CV_Assert((dst_cols >= MIN_RESOLUTION_W) && (dst_rows >= MIN_RESOLUTION_H));
#endif

    if ((src_u->fd < 0) || (dst_u->fd < 0)) {
        CV_Error(CV_HalMemErr, "alloc ion mem failed, fd < 0");
        return;
    }

    int fd = open("/dev/bm-vpp", O_RDWR);
    if (fd < 0) {
        CV_Error(CV_VppOpenErr, "open /dev/bm-vpp failed");
        return;
    }

    struct vpp_batch batch;
    memset(&batch, 0, sizeof(batch));
    batch.num = 1;

    struct vpp_cmd *cmd = &batch.cmd[0];

#if (defined VPP_BM1880) || (defined VPP_BM1684)
    cmd->src_format = RGB24;
    cmd->src_endian_a = 0;
    cmd->src_endian = 1;
    cmd->src_plannar = 0;
#if !defined VPP_BM1684
    cmd->tile_mode = 0;
#endif
    cmd->src_stride = src_step;

#ifdef ION_CACHE
    cmd->src_fd0 = src_u->fd;
    cmd->src_fd1 = 0;
    cmd->src_fd2 = 0;
#else
    cmd->src_addr0 = src_u->addr;
    cmd->src_addr1 = 0;
    cmd->src_addr2 = 0;
#endif

    cmd->src_cropH = src_rows;
    cmd->src_cropW = src_cols;
    cmd->src_axisX = 0;
    cmd->src_axisY = 0;

    cmd->dst_format = RGB24;
    cmd->dst_endian_a = 0;
    cmd->dst_endian = 1;
    cmd->dst_plannar = 0;
    cmd->dst_stride = dst_step;

#ifdef ION_CACHE
    cmd->dst_fd0 = dst_u->fd;
    cmd->dst_fd1 = 0;
    cmd->dst_fd2 = 0;
#else
    cmd->dst_addr0 = dst_u->addr;
    cmd->dst_addr1 = 0;
    cmd->dst_addr2 = 0;
#endif

    cmd->dst_cropH = dst_rows;
    cmd->dst_cropW = dst_cols;
    cmd->dst_axisX = 0;
    cmd->dst_axisY = 0;

    /*scl ctr*/
    cmd->hor_filter_sel = 2;
    cmd->ver_filter_sel = 2;
    cmd->scale_x_init = 0;
    cmd->scale_y_init = 0;
#else                                         /*BM1682*/
    cmd->src_format = FMT_SRC_BGR;
    cmd->src_stride = src_step;

#ifdef ION_CACHE
    cmd->src_fd0 = src_u->fd;
    cmd->src_fd1 = 0;
    cmd->src_fd2 = 0;
#else
    cmd->src_addr0 = src_u->addr;
    cmd->src_addr1 = 0;
    cmd->src_addr2 = 0;
#endif

    cmd->src_cropH = src_rows;
    cmd->src_cropW = src_cols;
    cmd->src_axisX = 0;
    cmd->src_axisY = 0;

    cmd->dst_format = FMT_DST_BGR;
    cmd->dst_stride = dst_step;

#ifdef ION_CACHE
    cmd->dst_fd0 = dst_u->fd;
    cmd->dst_fd1 = 0;
    cmd->dst_fd2 = 0;
#else
    cmd->dst_addr0 = dst_u->addr;
    cmd->dst_addr1 = 0;
    cmd->dst_addr2 = 0;
#endif

    cmd->dst_cropH = dst_rows;
    cmd->dst_cropW = dst_cols;
    cmd->dst_axisX = 0;
    cmd->dst_axisY = 0;
#endif

#ifdef ION_CACHE
    int ret = ioctl(fd, VPP_UPDATE_BATCH, &batch);
#else
    int ret = ioctl(fd, VPP_UPDATE_BATCH_NON_CACHE, &batch);
#endif
    if (ret < 0) {
        cout << "errno is " << errno << endl;
        vpp_parameter_dump(&batch);
        CV_Error(CV_VppIoctlErr, "ioctl resize failed");
    }

    close(fd);
}

static void crop_internal(InputArray src, std::vector<Rect>& loca, InputOutputArray dst)
{
    //CV_Assert(dst != NULL);
    CV_Assert(CV_MAT_DEPTH(src.type()) == CV_8U);
    CV_Assert(loca.size() > 0 && loca.size() <= VPP_MAX_BATCH_NUM);
    CV_Assert((src.isMat() && dst.isMatVector()) || (src.isUMat() && dst.isUMatVector()) ||
               (src.isMatVector() && dst.isMatVector()) || (src.isUMatVector() && dst.isUMatVector()));
    UMatData *src_u[VPP_MAX_BATCH_NUM], *dst_u[VPP_MAX_BATCH_NUM];
    int src_cols[VPP_MAX_BATCH_NUM];
    int src_rows[VPP_MAX_BATCH_NUM];
    size_t src_step[VPP_MAX_BATCH_NUM];
    bool src_is_batch = false;
    std::vector<Mat> mat_srcvector;
    std::vector<UMat> umat_srcvector;
    if (src.isMat() || src.isMatVector()){
        std::vector<Mat>& mat_vector = *(std::vector<Mat> *)dst.getObj();

        CV_Assert(mat_vector.empty());
        src_is_batch = src.isMatVector();
        if (src_is_batch){
            std::vector<Mat>& src_vector = *(std::vector<Mat> *)src.getObj();
            CV_Assert(loca.size() == src_vector.size());
        }

        for (size_t i = 0; i < loca.size(); i++) {
            Mat m;
            m.allocator = hal::getAllocator();
            m.create(loca[i].height, loca[i].width, src.type());
            dst_u[i] = m.u;
            mat_vector.push_back(m);
            Mat sMat = src_is_batch ? src.getMat(i) : src.getMat();
            sMat = checkSrcMat(sMat);
            mat_srcvector.push_back(sMat);
            src_u[i] = sMat.u;
        }
    }
    else if (src.isUMat() || src.isUMatVector()){
        std::vector<UMat>& umat_vector = *(std::vector<UMat> *)dst.getObj();

        CV_Assert(umat_vector.empty());
        src_is_batch = src.isUMatVector();
        if (src_is_batch){
            std::vector<UMat>& src_vector = *(std::vector<UMat> *)src.getObj();
            CV_Assert(loca.size() == src_vector.size());
        }

        for (size_t i = 0; i < loca.size(); i++) {
            UMat um;
            um.allocator = hal::getAllocator();
            um.create(loca[i].height, loca[i].width, src.type());
            dst_u[i] = um.u;
            umat_vector.push_back(um);
            UMat uSMat = src_is_batch ? src.getUMat(i) : src.getUMat();
            uSMat = checkSrcMat(uSMat);
            umat_srcvector.push_back(uSMat);
            src_u[i] = uSMat.u;
        }
    }

    for (size_t i = 0; i < loca.size(); i++){
        src_cols[i] = src_is_batch ? src.cols(i) : src.cols();
        src_rows[i] = src_is_batch ? src.rows(i) : src.rows();
        src_step[i] = src_is_batch ? src.step(i) : src.step();

        CV_Assert((src_cols[i] <= MAX_RESOLUTION_W) && (src_rows[i] <= MAX_RESOLUTION_H));
#if (defined VPP_BM1880) || (defined VPP_BM1684)
        CV_Assert((src_cols[i] >= MIN_RESOLUTION_W_LINEAR) && (src_rows[i] >= MIN_RESOLUTION_H_LINEAR));
#else
        CV_Assert((src_cols[i] >= MIN_RESOLUTION_W) && (src_rows[i] >= MIN_RESOLUTION_H));
#endif
    }

#if (defined VPP_BM1880) || (defined VPP_BM1684)
    for (size_t i = 0; i < loca.size(); i++) {
        CV_Assert((loca[i].x <= src_cols[i]) && (loca[i].y <= src_rows[i]));
        CV_Assert(((loca[i].x + loca[i].width) <= src_cols[i]) && ((loca[i].y + loca[i].height) <= src_rows[i]));
        CV_Assert((loca[i].width >= MIN_RESOLUTION_W_LINEAR) && (loca[i].height >= MIN_RESOLUTION_H_LINEAR));
    }
#else
    for (size_t i = 0; i < loca.size(); i++) {
        CV_Assert((loca[i].x <= src_cols[i]) && (loca[i].y <= src_rows[i]));
        CV_Assert(((loca[i].x + loca[i].width) <= src_cols[i]) && ((loca[i].y + loca[i].height) <= src_rows[i]));
        CV_Assert((loca[i].width >= MIN_RESOLUTION_W) && (loca[i].height >= MIN_RESOLUTION_H));
    }
#endif

    int fd = open("/dev/bm-vpp", O_RDWR);
    if (fd < 0) {
        CV_Error(CV_VppOpenErr, "open /dev/bm-vpp failed");
        return;
    }

    struct vpp_batch batch;
    memset(&batch,0,sizeof(batch));
    batch.num = loca.size();

    for (size_t i = 0; i < loca.size(); i++) {
        if ((src_u[i]->fd < 0) || (dst_u[i]->fd < 0)) {
            CV_Error(CV_HalMemErr, "alloc ion mem failed, fd < 0");
            return;
        }

        struct vpp_cmd *cmd = &batch.cmd[i];

#if (defined VPP_BM1880) || (defined VPP_BM1684)
        cmd->src_format = RGB24;
        cmd->src_endian_a = 0;
        cmd->src_endian = 1;
        cmd->src_plannar = 0;
#if !defined VPP_BM1684
        cmd->tile_mode = 0;
#endif
        cmd->src_stride = src_step[i];

#ifdef ION_CACHE
        cmd->src_fd0 = src_u[i]->fd;
        cmd->src_fd1 = 0;
        cmd->src_fd2 = 0;
#else
        cmd->src_addr0 = src_u[i]->addr;
        cmd->src_addr1 = 0;
        cmd->src_addr2 = 0;
#endif

        cmd->src_cropH = loca[i].height;
        cmd->src_cropW = loca[i].width;
        cmd->src_axisX = loca[i].x;
        cmd->src_axisY = loca[i].y;

        cmd->dst_format = RGB24;
        cmd->dst_endian_a = 0;
        cmd->dst_endian = 1;
        cmd->dst_plannar = 0;
        cmd->dst_stride = dst.step(i);

#ifdef ION_CACHE
        cmd->dst_fd0 = dst_u[i]->fd;
        cmd->dst_fd1 = 0;
        cmd->dst_fd2 = 0;
#else
        cmd->dst_addr0 = dst_u[i]->addr;
        cmd->dst_addr1 = 0;
        cmd->dst_addr2 = 0;
#endif

        cmd->dst_cropH = dst.rows(i);
        cmd->dst_cropW = dst.cols(i);
        cmd->dst_axisX = 0;
        cmd->dst_axisY = 0;

        /*scl ctr*/
        cmd->hor_filter_sel = 2;
        cmd->ver_filter_sel = 2;
        cmd->scale_x_init = 0;
        cmd->scale_y_init = 0;
#else                                       /*BM1682*/
        cmd->src_format = FMT_SRC_BGR;
        cmd->src_stride = src_step[i];

#ifdef ION_CACHE
        cmd->src_fd0 = src_u[i]->fd;
        cmd->src_fd1 = 0;
        cmd->src_fd2 = 0;
#else
        cmd->src_addr0 = src_u[i]->addr;
        cmd->src_addr1 = 0;
        cmd->src_addr2 = 0;
#endif

        cmd->src_cropH = loca[i].height;
        cmd->src_cropW = loca[i].width;
        cmd->src_axisX = loca[i].x;
        cmd->src_axisY = loca[i].y;

        cmd->dst_format = FMT_DST_BGR;
        cmd->dst_stride = dst.step(i);

#ifdef ION_CACHE
        cmd->dst_fd0 = dst_u[i]->fd;
        cmd->dst_fd1 = 0;
        cmd->dst_fd2 = 0;
#else
        cmd->dst_addr0 = dst_u[i]->addr;
        cmd->dst_addr1 = 0;
        cmd->dst_addr2 = 0;
#endif

        cmd->dst_cropH = dst.rows(i);
        cmd->dst_cropW = dst.cols(i);
        cmd->dst_axisX = 0;
        cmd->dst_axisY = 0;
#endif
    }

#ifdef ION_CACHE
    int ret = ioctl(fd, VPP_UPDATE_BATCH, &batch);
#else
    int ret = ioctl(fd, VPP_UPDATE_BATCH_NON_CACHE, &batch);
#endif

    if (ret < 0) {
        cout << "errno is " << errno << endl;
        vpp_parameter_dump(&batch);
        CV_Error(CV_VppIoctlErr, "ioctl crop failed");
    }

    close(fd);
}

static void border_internal(InputArray src, int top, int bottom, int left, int right, OutputArray dst)
{
    CV_Assert(CV_MAT_DEPTH(src.type()) == CV_8U);
    CV_Assert((src.isMat() && dst.isMat()) || (src.isUMat() && dst.isUMat()));
    CV_Assert( top >= 0 && bottom >= 0 && left >= 0 && right >= 0 );

    UMatData *src_u, *dst_u;
    int src_cols = src.cols();
    int src_rows = src.rows();
    size_t src_step = src.step();
    size_t dst_step = 0;
    UMat uSrcM;
    Mat  srcM;
    if(src.isMat()){
        Mat m;
        srcM = src.getMat();
        srcM = checkSrcMat(srcM);
        src_u = srcM.u;

        m.create(src_rows + top + bottom, src_cols + left + right, CV_8UC3);
        dst_u = m.u;
        dst.assign(m);
    }
    else {
        UMat um;
        uSrcM = src.getUMat();
        uSrcM = checkSrcMat(uSrcM);
        src_u = uSrcM.u;

        um.allocator = hal::getAllocator();
        um.create(src_rows + top + bottom, src_cols + left + right, CV_8UC3);
        dst_u = um.u;
        dst.assign(um);
    }
    dst_step = dst.step();

    CV_Assert((src_cols <= MAX_RESOLUTION_W) && (src_rows <= MAX_RESOLUTION_H));
    CV_Assert(((src_cols + left + right) <= MAX_RESOLUTION_W) && ((src_rows + top + bottom) <= MAX_RESOLUTION_H));
#if (defined VPP_BM1880) || (defined VPP_BM1684)
    CV_Assert((src_cols >= MIN_RESOLUTION_W_LINEAR) && (src_rows >= MIN_RESOLUTION_H_LINEAR));
    CV_Assert(((src_cols + left + right) >= MIN_RESOLUTION_W_LINEAR) && ((src_rows + top + bottom) >= MIN_RESOLUTION_H_LINEAR));
#else
    CV_Assert((src_cols >= MIN_RESOLUTION_W) && (src_rows >= MIN_RESOLUTION_H));
    CV_Assert(((src_cols + left + right) >= MIN_RESOLUTION_W) && ((src_rows + top + bottom) >= MIN_RESOLUTION_H));
#endif

    if ((src_u->fd < 0) || (dst_u->fd < 0)) {
        CV_Error(CV_HalMemErr, "alloc ion mem failed, fd < 0");
        return;
    }

    int fd = open("/dev/bm-vpp", O_RDWR);
    if (fd < 0) {
        CV_Error(CV_VppOpenErr, "open /dev/bm-vpp failed");
        return;
    }

    struct vpp_batch batch;
    memset(&batch,0,sizeof(batch));
    memset(dst_u->data,0,dst_u->size);
    batch.num = 1;

    struct vpp_cmd *cmd = &batch.cmd[0];

#if (defined VPP_BM1880) || (defined VPP_BM1684)
    cmd->src_format = RGB24;
    cmd->src_endian_a = 0;
    cmd->src_endian = 1;
    cmd->src_plannar = 0;
#if !defined VPP_BM1684
    cmd->tile_mode = 0;
#endif
    cmd->src_stride = src_step;

#ifdef ION_CACHE
    cmd->src_fd0 = src_u->fd;
    cmd->src_fd1 = 0;
    cmd->src_fd2 = 0;
#else
    cmd->src_addr0 = src_u->addr;
    cmd->src_addr1 = 0;
    cmd->src_addr2 = 0;
#endif

    cmd->src_cropH = src_rows;
    cmd->src_cropW = src_cols;
    cmd->src_axisX = 0;
    cmd->src_axisY = 0;

    cmd->dst_format = RGB24;
    cmd->dst_endian_a = 0;
    cmd->dst_endian = 1;
    cmd->dst_plannar = 0;
    cmd->dst_stride = dst_step;

#ifdef ION_CACHE
    cmd->dst_fd0 = dst_u->fd;
    cmd->dst_fd1 = 0;
    cmd->dst_fd2 = 0;
#else
    cmd->dst_addr0 = dst_u->addr;
    cmd->dst_addr1 = 0;
    cmd->dst_addr2 = 0;
#endif

    cmd->dst_cropH = src_rows;
    cmd->dst_cropW = src_cols;
    cmd->dst_axisX = left;
    cmd->dst_axisY = top;

    /*scl ctr*/
    cmd->hor_filter_sel = 2;
    cmd->ver_filter_sel = 2;
    cmd->scale_x_init = 0;
    cmd->scale_y_init = 0;
#else                           /*BM1682*/
    cmd->src_format = FMT_SRC_BGR;
    cmd->src_stride = src_step;

#ifdef ION_CACHE
    cmd->src_fd0 = src_u->fd;
    cmd->src_fd1 = 0;
    cmd->src_fd2 = 0;
#else
    cmd->src_addr0 = src_u->addr;
    cmd->src_addr1 = 0;
    cmd->src_addr2 = 0;
#endif

    cmd->src_cropH = src_rows;
    cmd->src_cropW = src_cols;
    cmd->src_axisX = 0;
    cmd->src_axisY = 0;

    cmd->dst_format = FMT_DST_BGR;
    cmd->dst_stride = dst_step;

#ifdef ION_CACHE
    cmd->dst_fd0 = dst_u->fd;
    cmd->dst_fd1 = 0;
    cmd->dst_fd2 = 0;
#else
    cmd->dst_addr0 = dst_u->addr;
    cmd->dst_addr1 = 0;
    cmd->dst_addr2 = 0;
#endif

    cmd->dst_cropH = src_rows;
    cmd->dst_cropW = src_cols;
    cmd->dst_axisX = left;
    cmd->dst_axisY = top;
#endif

#ifdef ION_CACHE
    int ret = ioctl(fd, VPP_UPDATE_BATCH, &batch);
#else
    int ret = ioctl(fd, VPP_UPDATE_BATCH_NON_CACHE, &batch);
#endif
    if (ret < 0) {
        cout << "errno is " << errno << endl;
        vpp_parameter_dump(&batch);
        CV_Error(CV_VppIoctlErr, "ioctl border failed");
    }

    close(fd);
    return;
}

static void split_internal(InputArray src, InputOutputArray dst)
{
    CV_Assert(CV_MAT_DEPTH(src.type()) == CV_8U);
    CV_Assert((src.isMat() && dst.isMatVector()) || (src.isUMat() && dst.isUMatVector()) ||
               (src.isMat() && dst.isMat()) || (src.isUMat() && dst.isUMat()));
    CV_Assert(src.channels() == 3);

    UMatData *src_u, *dst_u[3];
    int src_cols = src.cols();
    int src_rows = src.rows();
    size_t src_step = src.step();
    size_t dst_step = 0;
    UMat uSrcM;
    Mat  srcM;
    if (src.isMat()){
        srcM = src.getMat();
        srcM = checkSrcMat(srcM);
        src_u = srcM.u;

        if (src_u->fd < 0) {
            CV_Error(CV_HalMemErr, "alloc ion mem failed, fd < 0");
            return;
        }

        if (dst.isMatVector()) {
            std::vector<Mat>& mat_vector = *(std::vector<Mat> *)dst.getObj();

            CV_Assert(mat_vector.empty());
            for (size_t i = 0; i < 3; i++) {
                Mat m;
                m.allocator = hal::getAllocator();
                m.create(src.dims(), src.getMat().size, src.depth());
                dst_u[i] = m.u;

                if (dst_u[i]->fd < 0) {
                    CV_Error(CV_HalMemErr, "alloc ion mem failed, fd < 0");
                    return;
                }

                mat_vector.push_back(m);
            }

            dst_step = dst.step(0);
        } else if (dst.isMat()) {
            CV_Assert(dst.empty() == 0);
            dst_u[0] = dst.getMat().u;
            dst_u[1] = dst_u[2] = NULL;

            if (dst_u[0]->fd < 0) {
                CV_Error(CV_HalMemErr, "alloc ion mem failed, fd < 0");
                return;
            }

            dst_step = dst.cols();
            dst_step = (dst_step + 63) & ~0x3f;
        }
    }
    else {
        uSrcM = src.getUMat();
        uSrcM = checkSrcMat(uSrcM);
        src_u = uSrcM.u;

        if (src_u->fd < 0) {
            CV_Error(CV_HalMemErr, "alloc ion mem failed, fd < 0");
            return;
        }

        if (dst.isUMatVector()) {
            std::vector<UMat>& umat_vector = *(std::vector<UMat> *)dst.getObj();

            CV_Assert(umat_vector.empty());
            for (size_t i = 0; i < 3; i++) {
                UMat um;
                um.allocator = hal::getAllocator();
                um.create(src.dims(), src.getUMat().size, src.depth());
                dst_u[i] = um.u;

                if (dst_u[i]->fd < 0) {
                    CV_Error(CV_HalMemErr, "alloc ion mem failed, fd < 0");
                    return;
                }

                umat_vector.push_back(um);
            }

            dst_step = dst.step(0);
        } else if (dst.isUMat()) {
            CV_Assert(dst.empty() == 0);
            dst_u[0] = dst.getUMat().u;
            dst_u[1] = dst_u[2] = NULL;

            if (dst_u[0]->fd < 0) {
                CV_Error(CV_HalMemErr, "alloc ion mem failed, fd < 0");
                return;
            }

            dst_step = dst.cols();
            dst_step = (dst_step + 63) & ~0x3f;
        }
    }

    CV_Assert((src_cols <= MAX_RESOLUTION_W) && (src_rows <= MAX_RESOLUTION_H));
#if (defined VPP_BM1880) || (defined VPP_BM1684)
    CV_Assert((src_cols >= MIN_RESOLUTION_W_LINEAR) && (src_rows >= MIN_RESOLUTION_H_LINEAR));
#else
    CV_Assert((src_cols >= MIN_RESOLUTION_W) && (src_rows >= MIN_RESOLUTION_H));
#endif

    int fd = open("/dev/bm-vpp", O_RDWR);
    if (fd < 0) {
        CV_Error(CV_VppOpenErr, "open /dev/bm-vpp failed");
        return;
    }

    struct vpp_batch batch;
    memset(&batch,0,sizeof(batch));
    batch.num = 1;

    struct vpp_cmd *cmd = &batch.cmd[0];
#if (defined VPP_BM1880) || (defined VPP_BM1684)
    cmd->src_format = RGB24;
    cmd->src_endian_a = 0;
    cmd->src_endian = 1;
    cmd->src_plannar = 0;
#if !defined VPP_BM1684
    cmd->tile_mode = 0;
#endif
    cmd->src_stride = src_step;

#ifdef ION_CACHE
    cmd->src_fd0 = src_u->fd;
    cmd->src_fd1 = 0;
    cmd->src_fd2 = 0;
#else
    cmd->src_addr0 = src_u->addr;
    cmd->src_addr1 = 0;
    cmd->src_addr2 = 0;
#endif

    cmd->src_cropH = src_rows;
    cmd->src_cropW = src_cols;
    cmd->src_axisX = 0;
    cmd->src_axisY = 0;

    cmd->dst_format = RGB24;
    cmd->dst_endian_a = 0;
    cmd->dst_endian = 0;
    cmd->dst_plannar = 1;
    cmd->dst_stride = dst_step;

#ifdef ION_CACHE
    cmd->dst_fd0 = dst_u[0]->fd;
    cmd->dst_fd1 = dst_u[1] ? dst_u[1]->fd : 0;
    cmd->dst_fd2 = dst_u[2] ? dst_u[2]->fd : 0;
#else
    cmd->dst_addr0 = dst_u[0]->addr;
    cmd->dst_addr1 = dst_u[1] ? dst_u[1]->addr : 0;
    cmd->dst_addr2 = dst_u[2] ? dst_u[2]->addr : 0;
#endif

    cmd->dst_cropH = src_rows;
    cmd->dst_cropW = src_cols;
    cmd->dst_axisX = 0;
    cmd->dst_axisY = 0;

    /*scl ctr*/
    cmd->hor_filter_sel = 2;
    cmd->ver_filter_sel = 2;
    cmd->scale_x_init = 0;
    cmd->scale_y_init = 0;
#else                                       /*BM1682*/
    cmd->src_format = FMT_SRC_BGR;
    cmd->src_stride = src_step;

#ifdef ION_CACHE
    cmd->src_fd0 = src_u->fd;
    cmd->src_fd1 = 0;
    cmd->src_fd2 = 0;
#else
    cmd->src_addr0 = src_u->addr;
    cmd->src_addr1 = 0;
    cmd->src_addr2 = 0;
#endif

    cmd->src_cropH = src_rows;
    cmd->src_cropW = src_cols;
    cmd->src_axisX = 0;
    cmd->src_axisY = 0;

    cmd->dst_format = FMT_DST_RGBP;
    cmd->dst_stride = dst_step;

#ifdef ION_CACHE
    cmd->dst_fd0 = dst_u[0]->fd;
    cmd->dst_fd1 = dst_u[1] ? dst_u[1]->fd : 0;
    cmd->dst_fd2 = dst_u[2] ? dst_u[2]->fd : 0;
#else
    cmd->dst_addr0 = dst_u[0]->addr;
    cmd->dst_addr1 = dst_u[1] ? dst_u[1]->addr : 0;
    cmd->dst_addr2 = dst_u[2] ? dst_u[2]->addr : 0;
#endif

    cmd->dst_cropH = src_rows;
    cmd->dst_cropW = src_cols;
    cmd->dst_axisX = 0;
    cmd->dst_axisY = 0;
#endif

    int ret;
    if (dst.isMat() || dst.isUMat()) {
        ret = ioctl(fd, VPP_UPDATE_BATCH_SPLIT, &batch);
    }else {
#ifdef ION_CACHE
        ret = ioctl(fd, VPP_UPDATE_BATCH, &batch);
#else
        ret = ioctl(fd, VPP_UPDATE_BATCH_NON_CACHE, &batch);
#endif
    }
    if (ret < 0) {
        cout << "errno is " << errno << endl;
        vpp_parameter_dump(&batch);
        CV_Error(CV_VppIoctlErr, "ioctl split failed");
    }

    close(fd);

    return;
}

void crop(Mat* src, std::vector<Rect>& loca, Mat& dst)
{
    CV_Assert(CV_MAT_DEPTH(src[0].type()) == CV_8U);
    CV_Assert(loca.size() > 0 && loca.size() <= 16);
    CV_Assert(src != NULL);
    CV_Assert(dst.empty() == 1);
    unsigned i;

#if (defined VPP_BM1880) || (defined VPP_BM1684)
    CV_Error(CV_VppParamErr, "bm1880 not support so far");
    for (i = 0; i < loca.size(); i++) {
        CV_Assert((src[i].cols <= MAX_RESOLUTION_W) && (src[i].rows <= MAX_RESOLUTION_H));
        CV_Assert((loca[i].x <= src[i].cols) && (loca[i].y <= src[i].rows));
        CV_Assert(((loca[i].x + loca[i].width) <= src[i].cols) && ((loca[i].y + loca[i].height) <= src[i].rows));
        CV_Assert((src[i].cols >= MIN_RESOLUTION_W_LINEAR) && (src[i].rows >= MIN_RESOLUTION_H_LINEAR));
        CV_Assert((loca[i].width >= MIN_RESOLUTION_W_LINEAR) && (loca[i].height >= MIN_RESOLUTION_H_LINEAR));
    }
#else
    for (i = 0; i < loca.size(); i++) {
        CV_Assert((src[i].cols <= MAX_RESOLUTION_W) && (src[i].rows <= MAX_RESOLUTION_H));
        CV_Assert((loca[i].x <= src[i].cols) && (loca[i].y <= src[i].rows));
        CV_Assert(((loca[i].x + loca[i].width) <= src[i].cols) && ((loca[i].y + loca[i].height) <= src[i].rows));
        CV_Assert((src[i].cols >= MIN_RESOLUTION_W) && (src[i].rows >= MIN_RESOLUTION_H)); // TODO
        CV_Assert((loca[i].width >= MIN_RESOLUTION_W) && (loca[i].height >= MIN_RESOLUTION_H));
    }
#endif

    unsigned int w = 1920;
    unsigned int h = loca[0].height * 3;
    dst.create(h, w, src[0].type()); // TODO

    int fd = open("/dev/bm-vpp", O_RDWR);
    if (fd < 0) {
        CV_Error(CV_VppOpenErr, "open /dev/bm-vpp failed");
        return;
    }

    struct vpp_batch batch;
    memset(&batch, 0, sizeof(batch));
    batch.num = loca.size();

    for (i = 0; i < loca.size(); i++) {
        struct vpp_cmd *cmd = &batch.cmd[i];
#if (defined VPP_BM1880) || (defined VPP_BM1684)
        cmd->src_format = RGB24;
        cmd->src_endian_a = 0;
        cmd->src_endian = 1;
        cmd->src_plannar = 0;
#if !defined VPP_BM1684
        cmd->tile_mode = 0;
#endif
        cmd->src_stride = src[i].step.p[0];

#ifdef ION_CACHE
        cmd->src_fd0 = src[i].u->fd;
        cmd->src_fd1 = 0;
        cmd->src_fd2 = 0;
#else
        cmd->src_addr0 = src[i].u->addr;
        cmd->src_addr1 = 0;
        cmd->src_addr2 = 0;
#endif

        cmd->src_cropH = loca[i].height;
        cmd->src_cropW = loca[i].width;
        cmd->src_axisX = loca[i].x;
        cmd->src_axisY = loca[i].y;

        cmd->dst_format = RGB24;
        cmd->dst_endian_a = 0;
        cmd->dst_endian = 1;
        cmd->dst_plannar = 0;
        cmd->dst_stride = dst.step.p[0];

#ifdef ION_CACHE
        cmd->dst_fd0 = dst.u->fd;
        cmd->dst_fd1 = 0;
        cmd->dst_fd2 = 0;
#else
        cmd->dst_addr0 = dst.u->addr;
        cmd->dst_addr1 = 0;
        cmd->dst_addr2 = 0;
#endif

        cmd->dst_cropH = dst.rows;
        cmd->dst_cropW = dst.cols;
        cmd->dst_axisX = 0;
        cmd->dst_axisY = 0;

        /*scl ctr*/
        cmd->hor_filter_sel = 2;
        cmd->ver_filter_sel = 2;
        cmd->scale_x_init = 0;
        cmd->scale_y_init = 0;
#else                                           /*BM1682*/
        cmd->src_format = FMT_SRC_BGR;
        cmd->src_stride = src[i].step.p[0];

#ifdef ION_CACHE
        cmd->src_fd0 = src[i].u->fd;
        cmd->src_fd1 = 0;
        cmd->src_fd2 = 0;
#else
        cmd->src_addr0 = src[i].u->addr;
        cmd->src_addr1 = 0;
        cmd->src_addr2 = 0;
#endif

        cmd->src_cropH = loca[i].height;
        cmd->src_cropW = loca[i].width;
        cmd->src_axisX = loca[i].x;
        cmd->src_axisY = loca[i].y;

        cmd->dst_format = FMT_DST_BGR;
        cmd->dst_stride = dst.step.p[0];

#ifdef ION_CACHE
        cmd->dst_fd0 = dst.u->fd;
        cmd->dst_fd1 = 0;
        cmd->dst_fd2 = 0;
#else
        cmd->dst_addr0 = dst.u->addr;
        cmd->dst_addr1 = 0;
        cmd->dst_addr2 = 0;
#endif

        cmd->dst_cropH = loca[i].height;
        cmd->dst_cropW = loca[i].width;
        if (i < loca.size() / 3) {
            cmd->dst_axisX = loca[i].width * i;
            cmd->dst_axisY = 0;
        } else if ((i >= loca.size() / 3 ) && (i < loca.size() / 3 * 2)) {
            cmd->dst_axisX = loca[i].width * (i - loca.size() / 3);
            cmd->dst_axisY = loca[i].height;
        } else {
            cmd->dst_axisX = loca[i].width * (i - loca.size() / 3 * 2);
            cmd->dst_axisY = loca[i].height * 2;
        }
#endif
    }

#ifdef ION_CACHE
    int ret = ioctl(fd, VPP_UPDATE_BATCH_CROP_TEST, &batch);
#else
    int ret = ioctl(fd, VPP_UPDATE_BATCH_NON_CACHE, &batch);
#endif
    if (ret < 0) {
        cout << "errno is " << errno << endl;
        vpp_parameter_dump(&batch);
        CV_Error(CV_VppIoctlErr, "ioctl crop failed");
    }

    close(fd);
}

void toMat(Mat& img, Mat& out)
{
    if (!img.avOK()) {
        printf("MUST Input YUV Mat\n");
        out.release();
        return;
    }

    if (out.rows == 0 || out.cols == 0) {
        out.create(img.avRows(), img.avCols(), CV_8UC3);
    }

    if (out.u->fd < 0) {
        CV_Error(CV_HalMemErr, "alloc ion mem failed, fd < 0");
        return;
    }

    int fd = open("/dev/bm-vpp", O_RDWR);
    if (fd < 0) {
        CV_Error(CV_VppOpenErr, "open /dev/bm-vpp failed");
        return;
    }

    struct vpp_batch batch;
    memset(&batch,0,sizeof(batch));
    batch.num = 1;

    struct vpp_cmd *cmd = &batch.cmd[0];
    bool IsNV12 = (img.avFormat() == AV_PIX_FMT_NV12) ? true : false;

#if (defined VPP_BM1880) || (defined VPP_BM1684)
    cmd->src_format = YUV420;
    cmd->src_endian_a = 0;
    cmd->src_endian = 0;
#if defined VPP_BM1880
    cmd->src_plannar = 0;
    cmd->tile_mode = 0;
    cmd->hor_filter_sel = 2;
    cmd->ver_filter_sel = 2;
#elif defined VPP_BM1684
    cmd->src_plannar = IsNV12 ? 0 : 1;
    cmd->src_csc_en = 1;
    cmd->src_addr2 = IsNV12 ? 0 : img.avAddr(6);
    cmd->hor_filter_sel = 5;
    cmd->ver_filter_sel = 5;
#endif
    cmd->src_stride = img.avStep(4);
    cmd->src_addr0 = img.avAddr(4);
    cmd->src_addr1 = img.avAddr(5);
    cmd->src_cropH = img.avRows();
    cmd->src_cropW = img.avCols();
    cmd->src_axisX = 0;
    cmd->src_axisY = 0;

    cmd->dst_format = RGB24;
    cmd->dst_endian_a = 0;
    cmd->dst_endian = 1;
    cmd->dst_plannar = 0;
    cmd->dst_stride = out.step.p[0];

#ifdef ION_CACHE
    cmd->src_fd0 =
    cmd->src_fd1 =
    cmd->src_fd2 = 0;
    cmd->dst_fd0 = out.u->fd;
    cmd->dst_fd1 = 0;
    cmd->dst_fd2 = 0;
#else
    cmd->dst_addr0 = out.u->addr;
    cmd->dst_addr1 = 0;
    cmd->dst_addr2 = 0;
#endif

    cmd->dst_cropH = out.rows;
    cmd->dst_cropW = out.cols;
    cmd->dst_axisX = 0;
    cmd->dst_axisY = 0;

    /*scl ctr*/
    cmd->scale_x_init = 0;
    cmd->scale_y_init = 0;
#else                                             /*BM1682*/
    cmd->src_format = FMT_SRC_NV12;
    cmd->src_stride = img.avStep(4);
    cmd->src_addr0 = img.avAddr(4);
    cmd->src_addr1 = img.avAddr(5);
    cmd->src_cropH = img.avRows();
    cmd->src_cropW = img.avCols();
    cmd->src_axisX = 0;
    cmd->src_axisY = 0;

    cmd->dst_format = FMT_DST_BGR;
    cmd->dst_stride = out.step.p[0];

#ifdef ION_CACHE
    cmd->dst_fd0 = out.u->fd;
    cmd->dst_fd1 = 0;
    cmd->dst_fd2 = 0;
#else
    cmd->dst_addr0 = out.u->addr;
    cmd->dst_addr1 = 0;
    cmd->dst_addr2 = 0;
#endif
    cmd->dst_cropH = out.rows;
    cmd->dst_cropW = out.cols;
    cmd->dst_axisX = 0;
    cmd->dst_axisY = 0;
#endif

#ifdef ION_CACHE
    int ret = ioctl(fd, VPP_UPDATE_BATCH_VIDEO, &batch);
#else
    int ret = ioctl(fd, VPP_UPDATE_BATCH_NON_CACHE, &batch);
#endif
    if (ret < 0) {
        cout << "errno is " << errno << endl;
        vpp_parameter_dump(&batch);
        CV_Error(CV_VppIoctlErr, "ioctl nv12 failed");
    }

    close(fd);
}

Mat iplImageToMat(IplImage* img)
{
#if (defined VPP_BM1880) || (defined VPP_BM1684)
    CV_Assert((img->width <= MAX_RESOLUTION_W) && (img->height <= MAX_RESOLUTION_H));
    CV_Assert((img->width >= MIN_RESOLUTION_W_LINEAR) && (img->height >= MIN_RESOLUTION_H_LINEAR));
#else
    CV_Assert((img->width <= MAX_RESOLUTION_W) && (img->height <= MAX_RESOLUTION_H));
    CV_Assert((img->width >= MIN_RESOLUTION_W) && (img->height >= MIN_RESOLUTION_H));
#endif

    Mat dst(img->height, img->width, CV_8UC3);
    if (dst.u->fd < 0) {
        CV_Error(CV_HalMemErr, "alloc ion mem failed, fd < 0");
        return dst;
    }

    int fd = open("/dev/bm-vpp", O_RDWR);
    if (fd < 0) {
        CV_Error(CV_VppOpenErr, "open /dev/bm-vpp failed");
        return dst;
    }

    struct vpp_batch batch;
    memset(&batch,0,sizeof(batch));
    batch.num = 1;

    struct vpp_cmd *cmd = &batch.cmd[0];

#if (defined VPP_BM1880) || (defined VPP_BM1684)
    cmd->src_format = YUV420;
    cmd->src_endian_a = 0;
    cmd->src_endian = 0;
    cmd->src_plannar = 0;
#if defined VPP_BM1880
    cmd->tile_mode = 0;
#elif defined VPP_BM1684
    cmd->src_csc_en = 1;
#endif
    cmd->src_stride = img->step;

    cmd->src_addr0 = img->addr0;
    cmd->src_addr1 = img->addr1;
    cmd->src_cropH = img->height;
    cmd->src_cropW = img->width;
    cmd->src_axisX = 0;
    cmd->src_axisY = 0;

    cmd->dst_format = RGB24;
    cmd->dst_endian_a = 0;
    cmd->dst_endian = 1;
    cmd->dst_plannar = 0;
    cmd->dst_stride = dst.step.p[0];

#ifdef ION_CACHE
    cmd->src_fd0 =
    cmd->src_fd1 =
    cmd->src_fd2 = 0;
    cmd->dst_fd0 = dst.u->fd;
    cmd->dst_fd1 = 0;
    cmd->dst_fd2 = 0;
#else
    cmd->dst_addr0 = dst.u->addr;
    cmd->dst_addr1 = 0;
    cmd->dst_addr2 = 0;
#endif

    cmd->dst_cropH = dst.rows;
    cmd->dst_cropW = dst.cols;
    cmd->dst_axisX = 0;
    cmd->dst_axisY = 0;

    /*scl ctr*/
    cmd->hor_filter_sel = 2;
    cmd->ver_filter_sel = 2;
    cmd->scale_x_init = 0;
    cmd->scale_y_init = 0;
#else                                             /*BM1682*/
    cmd->src_format = FMT_SRC_NV12;
    cmd->src_stride = img->step;
    cmd->src_addr0 = img->addr0;
    cmd->src_addr1 = img->addr1;
    cmd->src_cropH = img->height;
    cmd->src_cropW = img->width;
    cmd->src_axisX = 0;
    cmd->src_axisY = 0;

    cmd->dst_format = FMT_DST_BGR;
    cmd->dst_stride = dst.step.p[0];

#ifdef ION_CACHE
    cmd->dst_fd0 = dst.u->fd;
    cmd->dst_fd1 = 0;
    cmd->dst_fd2 = 0;
#else
    cmd->dst_addr0 = dst.u->addr;
    cmd->dst_addr1 = 0;
    cmd->dst_addr2 = 0;
#endif
    cmd->dst_cropH = dst.rows;
    cmd->dst_cropW = dst.cols;
    cmd->dst_axisX = 0;
    cmd->dst_axisY = 0;
#endif

#ifdef ION_CACHE
    int ret = ioctl(fd, VPP_UPDATE_BATCH_VIDEO, &batch);
#else
    int ret = ioctl(fd, VPP_UPDATE_BATCH_NON_CACHE, &batch);
#endif
    if (ret < 0) {
        cout << "errno is " << errno << endl;
        vpp_parameter_dump(&batch);
        CV_Error(CV_VppIoctlErr, "ioctl nv12 failed");
    }

    close(fd);

    if(iplImage_dump_num > 0)
    {
        vpp_dump((char*)"iplImageToMat", DUMP_OUT, &dst,crop_dump_num, DUMP_MAT);
        iplImage_dump_num--;
    }
    return dst;
}

CV_EXPORTS Mat iplImageToMat(IplImage* img, CSC_TYPE csc_type)
{
#if (defined VPP_BM1880) || (defined VPP_BM1684)
    CV_Assert((img->srcFmt == FMT_SRC_I420) || (img->srcFmt == FMT_SRC_NV12) || (img->srcFmt == FMT_SRC_RGBP));
    CV_Assert((img->width <= MAX_RESOLUTION_W) && (img->height <= MAX_RESOLUTION_H));
    CV_Assert((img->width >= MIN_RESOLUTION_W_LINEAR) && (img->height >= MIN_RESOLUTION_H_LINEAR));
    CV_Assert(typeid(csc_type) == typeid(CSC_TYPE));
#else
    CV_Assert((img->srcFmt == FMT_SRC_I420) || (img->srcFmt == FMT_SRC_NV12) || (img->srcFmt == FMT_SRC_RGBP));
    CV_Assert((img->width <= MAX_RESOLUTION_W) && (img->height <= MAX_RESOLUTION_H));
    CV_Assert((img->width >= MIN_RESOLUTION_W) && (img->height >= MIN_RESOLUTION_H));
    CV_Assert(typeid(csc_type) == typeid(CSC_TYPE));
#endif

    Mat dst(img->height, img->width, CV_8UC3);
    if (dst.u->fd < 0) {
        CV_Error(CV_HalMemErr, "alloc ion mem failed, fd < 0");
        return dst;
    }

    int fd = open("/dev/bm-vpp", O_RDWR);
    if (fd < 0) {
        CV_Error(CV_VppOpenErr, "open /dev/bm-vpp failed");
        return dst;
    }

    struct vpp_batch batch;
    memset(&batch,0,sizeof(batch));
    batch.num = 1;

    struct vpp_cmd *cmd = &batch.cmd[0];

#if (defined VPP_BM1880) || (defined VPP_BM1684)
    switch (img->srcFmt) {
    case FMT_SRC_I420:
        cmd->src_format = YUV420;
        cmd->src_endian_a = 0;
        cmd->src_endian = 0;
        cmd->src_plannar = 1;
#if defined VPP_BM1880
        cmd->tile_mode = 0;
#elif defined VPP_BM1684
        cmd->src_csc_en = 1;
#endif
        break;
    case FMT_SRC_NV12:
        cmd->src_format = YUV420;
        cmd->src_endian_a = 0;
        cmd->src_endian = 0;
        cmd->src_plannar = 0;
#if defined VPP_BM1880
        cmd->tile_mode = 0;
#elif defined VPP_BM1684
        cmd->src_csc_en = 1;
#endif
        break;
    case FMT_SRC_RGBP:
        cmd->src_format = RGB24;
        cmd->src_endian_a = 0;
        cmd->src_endian = 0;
        cmd->src_plannar = 1;
#if defined VPP_BM1880
        cmd->tile_mode = 1;
#elif defined VPP_BM1684
        cmd->src_csc_en = 0;
#endif

        break;
    default:
        CV_Error(CV_VppParamErr, "unrecognized src format");
        break;
    }
    cmd->src_stride = img->step;
    cmd->src_addr0 = img->addr0;
    cmd->src_addr1 = img->addr1;
    if((img->srcFmt == FMT_SRC_I420) || (img->srcFmt == FMT_SRC_RGBP)) {
        cmd->src_addr2 = img->addr2;
    } else {
        cmd->src_addr2 = 0;
    }
    cmd->src_cropH = img->height;
    cmd->src_cropW = img->width;
    cmd->src_axisX = 0;
    cmd->src_axisY = 0;

    cmd->dst_format = RGB24;
    cmd->dst_endian_a = 0;
    cmd->dst_endian = 1;
    cmd->dst_plannar = 0;
    cmd->dst_stride = dst.step.p[0];

#ifdef ION_CACHE
    cmd->src_fd0 =
    cmd->src_fd1 =
    cmd->src_fd2 = 0;
    cmd->dst_fd0 = dst.u->fd;
    cmd->dst_fd1 = 0;
    cmd->dst_fd2 = 0;
#else
    cmd->dst_addr0 = dst.u->addr;
    cmd->dst_addr1 = 0;
    cmd->dst_addr2 = 0;
#endif

    cmd->dst_cropH = dst.rows;
    cmd->dst_cropW = dst.cols;
    cmd->dst_axisX = 0;
    cmd->dst_axisY = 0;

    /*scl ctr*/
    cmd->hor_filter_sel = 2;
    cmd->ver_filter_sel = 2;
    cmd->scale_x_init = 0;
    cmd->scale_y_init = 0;
    cmd->csc_type = csc_type;
#else                                             /*BM1682*/
    cmd->src_format = img->srcFmt;
    cmd->src_stride = img->step;
    cmd->src_addr0 = img->addr0;
    cmd->src_addr1 = img->addr1;
    if((img->srcFmt == FMT_SRC_I420) || (img->srcFmt == FMT_SRC_RGBP)) {
        cmd->src_addr2 = img->addr2;
    } else {
        cmd->src_addr2 = 0;
    }

    cmd->src_cropH = img->height;
    cmd->src_cropW = img->width;
    cmd->src_axisX = 0;
    cmd->src_axisY = 0;

    cmd->dst_format = FMT_DST_BGR;
    cmd->dst_stride = dst.step.p[0];

    cmd->csc_type = csc_type;

#ifdef ION_CACHE
    cmd->dst_fd0 = dst.u->fd;
    cmd->dst_fd1 = 0;
    cmd->dst_fd2 = 0;
#else
    cmd->dst_addr0 = dst.u->addr;
    cmd->dst_addr1 = 0;
    cmd->dst_addr2 = 0;
#endif
    cmd->dst_cropH = dst.rows;
    cmd->dst_cropW = dst.cols;
    cmd->dst_axisX = 0;
    cmd->dst_axisY = 0;
#endif

#ifdef ION_CACHE
    int ret = ioctl(fd, VPP_UPDATE_BATCH_VIDEO, &batch);
#else
    int ret = ioctl(fd, VPP_UPDATE_BATCH_NON_CACHE, &batch);
#endif
    if (ret < 0) {
        cout << "errno is " << errno << endl;
        vpp_parameter_dump(&batch);
        CV_Error(CV_VppIoctlErr, "ioctl nv12 failed");
    }

    close(fd);

    if(iplImage_dump_num > 0)
    {
        vpp_dump((char*)"iplImage_csc", DUMP_OUT, &dst,iplImage_dump_num, DUMP_MAT);
        iplImage_dump_num--;
    }
    return dst;
}

Mat iplImageToMat(IplImage* img, int mode)
{
#if (defined VPP_BM1880) || (defined VPP_BM1684)
    CV_Assert((img->width <= MAX_RESOLUTION_W) && (img->height <= MAX_RESOLUTION_H));
    CV_Assert((img->width >= MIN_RESOLUTION_W_LINEAR) && (img->height >= MIN_RESOLUTION_H_LINEAR));
#else
    CV_Assert((img->width <= MAX_RESOLUTION_W) && (img->height <= MAX_RESOLUTION_H));
    CV_Assert((img->width >= MIN_RESOLUTION_W) && (img->height >= MIN_RESOLUTION_H));
#endif
    CV_Assert((mode == 0) || (mode == 1));

    Mat dst(img->height, img->width, CV_8UC3);
    if (dst.u->fd < 0) {
        CV_Error(CV_HalMemErr, "alloc ion mem failed, fd < 0");
        return dst;
    }

    int fd = open("/dev/bm-vpp", O_RDWR);
    if (fd < 0) {
        CV_Error(CV_VppOpenErr, "open /dev/bm-vpp failed");
        return dst;
    }

    struct vpp_batch batch;
    memset(&batch,0,sizeof(batch));
    batch.num = 1;

    struct vpp_cmd *cmd = &batch.cmd[0];

#if (defined VPP_BM1880) || (defined VPP_BM1684)
    cmd->src_format = YUV420;
    cmd->src_endian_a = 0;
    cmd->src_endian = 0;
    cmd->src_plannar = 0;
#if defined VPP_BM1880
    cmd->tile_mode = mode;
#elif defined VPP_BM1684
    cmd->src_csc_en = 1;
#endif
    cmd->src_stride = img->step;

#ifdef ION_CACHE
    cmd->src_fd0 = img->addr0;
    cmd->src_fd1 = img->addr1;
    cmd->src_fd2 = 0;
#else
    cmd->src_addr0 = img->addr0;
    cmd->src_addr1 = img->addr1;
    cmd->src_addr2 = 0;
#endif

    cmd->src_cropH = img->height;
    cmd->src_cropW = img->width;
    cmd->src_axisX = 0;
    cmd->src_axisY = 0;

    cmd->dst_format = RGB24;
    cmd->dst_endian_a = 0;
    cmd->dst_endian = 1;
    cmd->dst_plannar = 0;
    cmd->dst_stride = dst.step.p[0];

#ifdef ION_CACHE
    cmd->dst_fd0 = dst.u->fd;
    cmd->dst_fd1 = 0;
    cmd->dst_fd2 = 0;
#else
    cmd->dst_addr0 = dst.u->addr;
    cmd->dst_addr1 = 0;
    cmd->dst_addr2 = 0;
#endif

    cmd->dst_cropH = dst.rows;
    cmd->dst_cropW = dst.cols;
    cmd->dst_axisX = 0;
    cmd->dst_axisY = 0;

    /*scl ctr*/
    cmd->hor_filter_sel = 2;
    cmd->ver_filter_sel = 2;
    cmd->scale_x_init = 0;
    cmd->scale_y_init = 0;
#else                                     /*BM1682*/
    cmd->src_format = FMT_SRC_NV12;
    cmd->src_stride = img->step;

#ifdef ION_CACHE
    cmd->src_fd0 = img->addr0;
    cmd->src_fd1 = img->addr1;
    cmd->src_fd2 = 0;
#else
    cmd->src_addr0 = img->addr0;
    cmd->src_addr1 = img->addr1;
    cmd->src_addr2 = 0;
#endif

    cmd->src_cropH = img->height;
    cmd->src_cropW = img->width;
    cmd->src_axisX = 0;
    cmd->src_axisY = 0;

    cmd->dst_format = FMT_DST_BGR;
    cmd->dst_stride = dst.step.p[0];

#ifdef ION_CACHE
    cmd->dst_fd0 = dst.u->fd;
    cmd->dst_fd1 = 0;
    cmd->dst_fd2 = 0;
#else
    cmd->dst_addr0 = dst.u->addr;
    cmd->dst_addr1 = 0;
    cmd->dst_addr2 = 0;
#endif

    cmd->dst_cropH = dst.rows;
    cmd->dst_cropW = dst.cols;
    cmd->dst_axisX = 0;
    cmd->dst_axisY = 0;
#endif

#ifdef ION_CACHE
    int ret = ioctl(fd, VPP_UPDATE_BATCH, &batch);
#else
    int ret = ioctl(fd, VPP_UPDATE_BATCH_NON_CACHE, &batch);
#endif
    if (ret < 0) {
        cout << "errno is " << errno << endl;
        vpp_parameter_dump(&batch);
        CV_Error(CV_VppIoctlErr, "ioctl nv12 failed");
    }

    close(fd);
    if(iplImage_dump_num > 0)
    {
        printf("Input iplImage info: iplImageToMat_mode, width=%d,height=%d,src_stride=%d,dst_stride=%d, FMT_SRC_NV12,FMT_DST_BGR\n", img->width, img->height, img->step, (int)dst.step.p[0]);
        vpp_dump((char*)"iplImage_mode", DUMP_OUT, &dst,iplImage_dump_num, DUMP_MAT);
        iplImage_dump_num--;
    }
    return dst;
}

#if (!defined VPP_BM1880) && (!defined VPP_BM1684)
Mat cvtColor1682(Mat& src, int srcFmt, int dstFmt)
{
    CV_Assert((srcFmt >= FMT_SRC_I420) && (srcFmt <= FMT_SRC_RGB));
    CV_Assert((srcFmt == FMT_SRC_BGR) && (dstFmt == FMT_DST_RGB));
    CV_Assert((src.cols >= MIN_RESOLUTION_W) && (src.rows >= MIN_RESOLUTION_H));
    // CV_Assert((dstFmt >= FMT_DST_I420) && (dstFmt <= FMT_DST_RGB));
    // CV_Assert((dst.cols >= MIN_RESOLUTION_W) && (dst.rows >= MIN_RESOLUTION_H));

    Mat dst(src.rows, src.cols, CV_8UC3);

    if ((src.u->fd < 0) && (dst.u->fd < 0)) {
        CV_Error(CV_HalMemErr, "alloc ion mem failed, fd < 0");
        return dst;
    }

    int fd = open("/dev/bm-vpp", O_RDWR);
    if (fd < 0) {
        CV_Error(CV_VppOpenErr, "open /dev/bm-vpp failed");
        return dst;
    }

    struct vpp_batch batch;
    memset(&batch,0,sizeof(batch));
    batch.num = 1;

    struct vpp_cmd *cmd = &batch.cmd[0];

    cmd->src_format = srcFmt;
    cmd->src_stride = src.step.p[0];

#ifdef ION_CACHE
    cmd->src_fd0 = src.u->fd;
    cmd->src_fd1 = 0;
    cmd->src_fd2 = 0;
#else
    cmd->src_addr0 = src.u->addr;
    cmd->src_addr1 = 0;
    cmd->src_addr2 = 0;
#endif

    cmd->src_cropH = src.rows;
    cmd->src_cropW = src.cols;
    cmd->src_axisX = 0;
    cmd->src_axisY = 0;

    cmd->dst_format = dstFmt;
    cmd->dst_stride = dst.step.p[0];

#ifdef ION_CACHE
    cmd->dst_fd0 = dst.u->fd;
    cmd->dst_fd1 = 0;
    cmd->dst_fd2 = 0;
#else
    cmd->dst_addr0 = dst.u->addr;
    cmd->dst_addr1 = 0;
    cmd->dst_addr2 = 0;
#endif

    cmd->dst_cropH = dst.rows;
    cmd->dst_cropW = dst.cols;
    cmd->dst_axisX = 0;
    cmd->dst_axisY = 0;

#ifdef ION_CACHE
    int ret = ioctl(fd, VPP_UPDATE_BATCH, &batch);
#else
    int ret = ioctl(fd, VPP_UPDATE_BATCH_NON_CACHE, &batch);
#endif
    if (ret < 0) {
        CV_Error(CV_VppIoctlErr, "ioctl resize failed");
    }

    close(fd);

    if(cvtColor1682_dump_num > 0)
    {
        printf("Input mat info: cvtColor1682_1 width=%d,height=%d,step=%d,srcformat=%d\n", src.cols, src.rows, (int)src.step.p[0], srcFmt);
        vpp_dump((char*)"cvtColor1682", DUMP_OUT, &dst,cvtColor1682_dump_num, DUMP_MAT);
        cvtColor1682_dump_num--;
    }
    return dst;
}

CV_EXPORTS void cvtColor1682(Mat& src, IplImage* img, int srcFmt, int dstFmt)
{
    CV_Assert(CV_MAT_DEPTH(src.type()) == CV_8U);
    CV_Assert((srcFmt >= FMT_SRC_I420) && (srcFmt <= FMT_SRC_RGB));
    CV_Assert((srcFmt == FMT_SRC_BGR) && (dstFmt == FMT_DST_RGB));
    CV_Assert((src.cols >= MIN_RESOLUTION_W) && (src.rows >= MIN_RESOLUTION_H));
    // CV_Assert((dstFmt >= FMT_DST_I420) && (dstFmt <= FMT_DST_RGB));
    // CV_Assert((dst.cols >= MIN_RESOLUTION_W) && (dst.rows >= MIN_RESOLUTION_H));

    int fd = open("/dev/bm-vpp", O_RDWR);
    if (fd < 0) {
        CV_Error(CV_VppOpenErr, "open /dev/bm-vpp failed");
        return;
    }

    struct vpp_batch batch;
    memset(&batch,0,sizeof(batch));
    batch.num = 1;

    struct vpp_cmd *cmd = &batch.cmd[0];

#if (defined VPP_BM1880) || (defined VPP_BM1684)
    cmd->src_format = RGB24;
    cmd->src_endian_a = 0;
    cmd->src_endian = 1;
    cmd->src_plannar = 0;
#if !defined VPP_BM1684
    cmd->tile_mode = 0;
#endif
    cmd->src_stride = src.step.p[0];

    cmd->src_addr0 = src.u->addr;
    cmd->src_addr1 = 0;
    cmd->src_addr2 = 0;

    cmd->src_cropH = src.rows;
    cmd->src_cropW = src.cols;
    cmd->src_axisX = 0;
    cmd->src_axisY = 0;

    cmd->dst_format = RGB24;
    cmd->dst_endian_a = 0;
    cmd->dst_endian = 0;
    cmd->dst_plannar = 0;
    cmd->dst_stride = img->step;

    cmd->dst_addr0 = img->addr0;
    cmd->dst_addr1 = img->addr1;
    cmd->dst_addr2 = 0;

    cmd->dst_cropH = src.rows;
    cmd->dst_cropW = src.cols;
    cmd->dst_axisX = 0;
    cmd->dst_axisY = 0;

    /*scl ctr*/
    cmd->hor_filter_sel = 2;
    cmd->ver_filter_sel = 2;
    cmd->scale_x_init = 0;
    cmd->scale_y_init = 0;
#else                                         /*BM1682*/
    cmd->src_format = srcFmt;
    cmd->src_stride = src.step.p[0];

    cmd->src_addr0 = src.u->addr;
    cmd->src_addr1 = 0;
    cmd->src_addr2 = 0;

    cmd->src_cropH = src.rows;
    cmd->src_cropW = src.cols;
    cmd->src_axisX = 0;
    cmd->src_axisY = 0;

    cmd->dst_format = dstFmt;
    cmd->dst_stride = img->step;

    cmd->dst_addr0 = img->addr0;
    cmd->dst_addr1 = img->addr1;
    cmd->dst_addr2 = 0;

    cmd->dst_cropH = src.rows;
    cmd->dst_cropW = src.cols;
    cmd->dst_axisX = 0;
    cmd->dst_axisY = 0;
#endif
    int ret = ioctl(fd, VPP_UPDATE_BATCH_NON_CACHE, &batch);
    if (ret < 0) {
        CV_Error(CV_VppIoctlErr, "ioctl resize failed");
    }

    close(fd);


    if(cvtColor1682_dump_num > 0)
    {
        printf("Input mat info: cvtColor1682_2 width=%d,height=%d,src_step=%d,srcformat=%d\n", src.cols, src.rows, (int)src.step.p[0], srcFmt);
        printf("Output mat info: cvtColor1682_2 width=%d,height=%d,dst_step=%d,dstformat=%d\n", src.cols, src.rows, img->step, dstFmt);
         // vpp_dump((char*)"iplImage_mode", DUMP_OUT, (void*)img,iplImage_dump_num, DUMP_IML);
        cvtColor1682_dump_num--;
    }
    return;
}
#endif
}}

//#endif
#endif
