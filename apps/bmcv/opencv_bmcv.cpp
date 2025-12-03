// This file is part of OpenCV project.
// It is subject to the license terms in the LICENSE file found in the top-level directory
// of this distribution and at http://opencv.org/license.html.

#include <iostream>

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

using namespace cv;
using namespace std;

typedef void (*test_func_t)(int argc, const char** argv);

typedef struct {
  const char* name;
  test_func_t test_func;
  int valid_argc_count[4];
  int valid_argc_len;
} test_case_entry;

static void test_conv_1(int argc, const char** argv);
static void test_conv_4(int argc, const char** argv);
static void test_size(int argc, const char** argv);
static void test_video(int argc, const char** argv);
static void test_image(int argc, const char** argv);
static void test_cvt(int argc, const char** argv);
static void test_bmcv2cv(int argc, const char** argv);
static void test_bitwise_and(int argc, const char** argv);
static void test_bitwise_or(int argc, const char** argv);
static void test_bitwise_xor(int argc, const char** argv);

static const test_case_entry test_case_table[] = {
  {"conv_1", test_conv_1, {3,4}, 2},
  {"conv_4", test_conv_4, {6,7}, 2},
  {"size", test_size, {3,4}, 2},
  {"video", test_video, {3,4}, 2},
  {"image", test_image, {3,4}, 2},
  {"cvt", test_cvt, {3,4}, 2},
  {"bmcv2cv", test_bmcv2cv, {3,4}, 2},
  {"bitwise_and", test_bitwise_and, {4,5}, 2},
  {"bitwise_or", test_bitwise_or, {4,5}, 2},
  {"bitwise_xor", test_bitwise_xor, {4,5}, 2},
  // ...
};

#define TEST_CASE_COUNT (sizeof(test_case_table) / sizeof(test_case_table[0]))

int g_device_id = 0;
static int dump_data(const char *fn, Mat in, int yuv_enable)
{
  FILE *fp;

  fp = fopen(fn, "wb");
  if (fp == NULL){
    printf("open %s failed\n", fn);
    return -1;
  }

  if (yuv_enable && in.avOK()){
#ifdef USING_SOC
    for (int i = 0; i < in.avRows(); i++){
      fwrite((char*)in.avAddr(0)+i*in.avStep(0),1,in.avCols(),fp);
    }
    for (int i = 0; i < in.avRows()/2; i++){
      fwrite((char*)in.avAddr(1)+i*in.avStep(1),1,in.avCols(),fp);
    }
    /*
    for (int i = 0; i < in.avRows()/2; i++){
      fwrite((char*)in.avAddr(2)+i*in.avStep(2),1,in.avCols()/2,fp);
    }
    */
#else
    bm_handle_t handle = in.u->hid ? in.u->hid : bmcv::getCard();
    bm_device_mem_t mem;
    unsigned char *p;

    p = (unsigned char *)malloc(in.avRows() * in.avStep(0) * 3);
    if (!p){
      printf("malloc failed\n");
      fclose(fp);
      return -1;
    }
    mem = bm_mem_from_device(in.u->addr, in.u->size);
    bm_memcpy_d2s(handle, p, mem);

    for (int i = 0; i < in.avRows(); i++){
      fwrite((char*)p+i*in.avStep(0),1,in.avCols(),fp);
    }
    for (int i = 0; i < in.avRows()/2; i++){
      fwrite((char*)p + in.avRows()*in.avStep(0) + i*in.avStep(1),1,in.avCols(),fp);
    }

    if (p) {free(p); p = NULL;}
#endif
  }else {
    for (int i = 0; i < in.rows; i++)
    {
      fwrite(in.data+i*in.step[0],1,in.cols*in.channels(),fp);
    }
  }

  if (fp) fclose(fp);

  return 0;
}

static void test_conv_1(int argc, const char** argv)
{
  Mat m0 = imread(argv[2], IMREAD_COLOR, g_device_id);
  bmcv::print(m0);

  Mat sub(m0, Rect(16, 16, 128, 128));
  bmcv::print(sub);

  bm_image image;
  bmcv::toBMI(sub, &image);
  bmcv::print(&image);

  Mat d0;
  bmcv::toMAT(&image, d0);
  bmcv::print(d0);

  imwrite("conv_1_0.png", d0);
  bm_image_destroy(image);
}

static void test_conv_4(int argc, const char** argv)
{
  Mat m0 = imread(argv[2], IMREAD_COLOR, g_device_id);
  Mat m1 = imread(argv[3], IMREAD_COLOR, g_device_id);
  Mat m2 = imread(argv[4], IMREAD_COLOR, g_device_id);
  Mat m3 = imread(argv[5], IMREAD_COLOR, g_device_id);
  bmcv::print(m0);

  bm_image image;
  bmcv::toBMI(m0, m1, m2, m3, &image);
  bmcv::print(&image);

  Mat d0, d1, d2, d3;
  bmcv::toMAT(&image, d0, d1, d2, d3);
  bmcv::print(d0);

  imwrite("conv_4_0.png", d0);
  imwrite("conv_4_1.png", d1);
  imwrite("conv_4_2.png", d2);
  imwrite("conv_4_3.png", d3);
  bm_image_destroy(image);
}

static void test_size(int argc, const char** argv)
{
  Mat m0 = imread(argv[2], IMREAD_COLOR, g_device_id);
  bmcv::print(m0);

  Mat out(200, 200, CV_8UC3, SophonDevice(g_device_id));
  bmcv::resize(m0, out);
  bmcv::print(out);

  imwrite("size_1_0.png", out);
}

static void test_video(int argc, const char** argv)
{
  VideoCapture cap(argv[2], cv::CAP_ANY, g_device_id);
  cap.set(cv::CAP_PROP_OUTPUT_YUV, PROP_TRUE);

  Mat frame;
  cap >> frame;
  bmcv::print(frame, true);

  Mat m;
  bmcv::toMAT(frame, m, true);
  imwrite("video_in.png", m);

  Rect rt0(0, 0, 1920, 1080);
  Rect rt1(0, 0, 1280, 720);

  std::vector<Rect> vrt= { rt0, rt1 };

  Size sz0(640, 640);
  Size sz1(128, 128);

  std::vector<Size> vsz= { sz0, sz1 };

  std::vector<Mat> out;
  bmcv::convert(frame, vrt, vsz, out);
  bmcv::print(out[0]);
  bmcv::print(out[1]);

  imwrite("video_0.png", out[0]);
  imwrite("video_1.png", out[1]);
}

static void test_image(int argc, const char** argv)
{
  Mat frame = imread(argv[2], IMREAD_AVFRAME, g_device_id);
  bmcv::print(frame);

  imwrite("image_0.jpg", frame);

  //Mat sub(frame, Rect(16, 16, 128, 128));
  //bmcv::print(sub);
  Mat &sub = frame;

  Mat image;
  bmcv::toMAT(sub, image);
  bmcv::print(image);

  imwrite("image_0.png", image);
}

static void test_cvt(int argc, const char** argv)
{
  Mat image = imread(argv[2], IMREAD_COLOR, g_device_id);
  Mat gray(image.rows, image.cols, CV_8UC1, SophonDevice(g_device_id));

  cvtColor(image, gray, COLOR_BGR2GRAY);

  imwrite("image_cvt.png", gray);
}

static void read_file(const char *filename, void* jpeg_data, size_t* size)
{
    FILE *fp = fopen(filename, "rb+");
    if (fp == NULL) {
        fprintf(stderr, "file not exist or permission not allowed\n");
        exit(-1);
    }
    fseek(fp, 0, SEEK_END);
    *size = ftell(fp);
    fseek(fp, 0, SEEK_SET);
    fread(jpeg_data, *size, 1, fp);
    printf("read from %s %ld bytes\n", filename, *size);
    fclose(fp);
}

static int get_plane_width(int format, int plane, int width)
{
    int ret;
    if (plane > 2) return 0;

    switch (format){
        case FORMAT_YUV420P:
        case FORMAT_YUV422P:
            ret = (plane == 0) ? width : width/2;
            break;
        case FORMAT_NV12:
        case FORMAT_NV16:
            ret = (plane != 2) ? width : 0;
            break;
        case FORMAT_YUV444P:
            ret = width;
            break;
        case FORMAT_GRAY:
            ret = (plane == 0) ? width : 0;
            break;
        default:
            fprintf(stderr, "ERROR: unsupported format %d\n", format);
            return -1;
    }

    return ret;
}

static int bmimage_copyto_bmimage(bm_image in, bm_image out)
{
    int i, j;
    int plane_num = bm_image_get_plane_num(in);
    bm_device_mem_t src_mem[4], dst_mem[4];
    int src_stride[4], dst_stride[4];
    int width = out.width;
    int height = out.height;
    bm_handle_t handle = bm_image_get_handle(&in);
    int total_copy = 0;
    bm_status_t ret;

    assert(width <= in.width);
    assert(height <= in.height);
    ret = bm_image_get_device_mem(in, src_mem);
    assert(ret == BM_SUCCESS);
    ret = bm_image_get_device_mem(out, dst_mem);
    assert(ret == BM_SUCCESS);
    ret = bm_image_get_stride(in, src_stride);
    assert(ret == BM_SUCCESS);
    ret = bm_image_get_stride(out, dst_stride);
    assert(ret == BM_SUCCESS);

    for (i = 0; i < plane_num; i++){
        int dst_offset = 0;
        int src_offset = 0;
        int plane_width = get_plane_width(in.image_format, i, width);

        if (plane_width < 0) return -1;

        for (j = 0; j < height; j++){
            if (BM_SUCCESS != bm_memcpy_d2d_byte(handle, dst_mem[i], dst_offset, src_mem[i], src_offset, plane_width)){
                fprintf(stderr, "ERROR: bm d2d failed\n");
                return -1;
            }
            total_copy += plane_width;
            dst_offset += dst_stride[i];
            src_offset += src_stride[i];
        }
    }
    bm_thread_sync(handle);

    return total_copy;
}

static void test_bmcv2cv(int argc, const char** argv)
{
    bm_handle_t handle;
    string prefix;
    bm_status_t ret;
    int width, height;

    ret = bm_dev_request(&handle, BM_CARD_ID( g_device_id ));
    if (ret != BM_SUCCESS) {
        printf("create bm handle failed!");
        exit(-1);
    }
    // read input from picture
    size_t size = 0;
    unsigned char* jpeg_data = (unsigned char*)malloc(4096 * 4096 * 3);
    read_file(argv[2], jpeg_data, &size);

    // create bm_image used to save output
    bm_image src[1];

start:
    // decode input
    ret = bmcv_image_jpeg_dec(handle, (void**)&jpeg_data, &size, 1, src);
    assert(ret == BM_SUCCESS);

    width = src[0].width;
    height = src[0].height;

    /* original image */
    printf("step1: test original image\n");
    Mat m;
    cv::bmcv::toMAT(src, m, AVCOL_SPC_BT470BG, AVCOL_RANGE_JPEG, NULL, -1, false, true);

    cv::imwrite(prefix+"out"+".jpg", m);

    /* resize to image of different size */
    printf("step2: test 0-63 aligned image\n");
    for (int i = 0; i < 64; i++){
        bm_image out;
        int stride[3];

        stride[0] = (width - i + 15) &~0xf;
        stride[1] = get_plane_width(src->image_format, 1, stride[0]);
        stride[2] = get_plane_width(src->image_format, 2, stride[0]);
        ret = bm_image_create(handle, height-i, width-i, src->image_format, src->data_type, &out, stride);
        assert(ret == BM_SUCCESS);

        ret = bm_image_alloc_contiguous_mem(1, &out);
        assert(ret == BM_SUCCESS);

        /* Because some format does not supported by bmcv, we do it by d2d */
        if (bmimage_copyto_bmimage(src[0], out) < 0){
            printf("bm_image copyto bmimage failed\n");
            exit(-1);
        }

        Mat m0;
        cv::bmcv::toMAT(&out, m0, AVCOL_SPC_BT470BG, AVCOL_RANGE_JPEG, NULL, -1, false, true);
        cv::imwrite(prefix+"out"+to_string(i)+".jpg", m0);

        bm_image_free_contiguous_mem(1, &out);
        bm_image_destroy(out);
    }

    /* convert to BGR */
    printf("step3: test BGR png\n");
    {
        bm_image out;

        ret = bm_image_create(handle, height, width, FORMAT_BGR_PACKED, src->data_type, &out);
        assert(ret == BM_SUCCESS);

        ret = bm_image_alloc_contiguous_mem(1, &out);
        assert(ret == BM_SUCCESS);

        ret = bmcv_image_storage_convert_with_csctype(handle, 1, src, &out, CSC_YPbPr2RGB_BT601);
        assert(ret == BM_SUCCESS);

        Mat m1;
        cv::bmcv::toMAT(&out, m1, AVCOL_SPC_BT470BG, AVCOL_RANGE_JPEG, NULL, -1, false, true);
        cv::bmcv::downloadMat(m1);

        cv::imwrite(prefix+"out_bgr"+".png", m1);

        bm_image_free_contiguous_mem(1, &out);
        bm_image_destroy(out);
    }

    if (src->image_format == FORMAT_YUV420P){
        bm_image_data_format_ext data_type = src->data_type;
        bm_image_destroy(*src);

        ret = bm_image_create(handle, height, width, FORMAT_NV12, data_type, src);
        assert(ret == BM_SUCCESS);
        prefix = "nv12_";
        printf("restart for nv12 format\n");
        goto start;
    } else if (src->image_format == FORMAT_YUV422P) {
        bm_image_data_format_ext data_type = src->data_type;
        bm_image_destroy(*src);

        ret = bm_image_create(handle, height, width, FORMAT_NV16, data_type, src);
        assert(ret == BM_SUCCESS);
        prefix = "nv16_";
        printf("restart for nv16 format\n");
        goto start;
    }

    /* end */
    bm_image_destroy(*src);
    free(jpeg_data);
    bm_dev_free(handle);

    return;
}

static void test_bitwise_and(int argc, const char** argv) {
    bm_handle_t handle = NULL;
    bm_status_t ret;
    /*-----------------------test BGR:-----------------------*/
    printf("test BGR:\n");
    Mat frame0 = imread(argv[2], IMREAD_COLOR, g_device_id);
    Mat frame1 = imread(argv[3], IMREAD_COLOR, g_device_id);
    Mat output(frame0.size(), frame0.type());
    bool update = true;
    handle = frame0.u->hid ? frame0.u->hid : bmcv::getCard();
    ret = bmcv::bitwise_and(frame0, frame1, output, update);
    if (ret != BM_SUCCESS) {
        printf("bitwise_and test BGR failed!\n");
        exit(-1);
    }
    bmcv::downloadMat(output);
    imwrite("bgr_bitwise_and.png", output);
    bmcv::print(output);

    Mat cv_output;
    cv::bitwise_and(frame0, frame1, cv_output);
    imwrite("cv_bgr_bitwise_and.png", cv_output);
    bmcv::print(cv_output);

    /*-----------------------test GRAY:-----------------------*/
    frame0.release();
    frame1.release();
    output.release();
    cv_output.release();
    printf("test GRAY:\n");
    frame0 = imread(argv[2], IMREAD_GRAYSCALE, g_device_id);
    frame1 = imread(argv[3], IMREAD_GRAYSCALE, g_device_id);
    ret = bmcv::bitwise_and(frame0, frame1, output, update);
    if (ret != BM_SUCCESS) {
        printf("bitwise_and test GRAY failed!\n");
        exit(-1);
    }
    bmcv::downloadMat(output);
    imwrite("gray_bitwise_and.png", output);
    bmcv::print(output);

    cv::bitwise_and(frame0, frame1, cv_output);
    imwrite("cv_gray_bitwise_and.png", cv_output);
    bmcv::print(cv_output);

    /*-----------------------test NV12:-----------------------*/
    frame0.release();
    frame1.release();
    output.release();
    cv_output.release();
    printf("test NV12:\n");
    frame0 = imread(argv[2], IMREAD_AVFRAME, g_device_id);
    frame1 = imread(argv[3], IMREAD_AVFRAME, g_device_id);

    int p_plane_stride[3], plane_size[3];
    int wscale[3], hscale[3];
    int plane_num = av::get_scale_and_plane(AV_PIX_FMT_NV12 , wscale, hscale);
    for (int i = 0; i < plane_num; i++) {
        p_plane_stride[i] = frame0.avCols() / wscale[i];
        plane_size[i] = p_plane_stride[i] * (frame0.avRows() / hscale[i]);
    }
    int id = bmcv::getId(handle);
    AVFrame *f_0 = av::create(
        frame0.rows, frame0.cols, AV_PIX_FMT_NV12, nullptr, 0, -1, p_plane_stride, plane_size);
    Mat frame0_nv12;
    frame0_nv12.create(f_0, id);
    bmcv::convert(frame0, frame0_nv12);

    AVFrame *f_1 = av::create(
        frame0.rows, frame0.cols, AV_PIX_FMT_NV12, nullptr, 0, -1, p_plane_stride, plane_size);
    Mat frame1_nv12;
    frame1_nv12.create(f_1, id);
    bmcv::convert(frame1, frame1_nv12);

    ret = bmcv::bitwise_and(frame0_nv12, frame1_nv12, output, update);
    if (ret != BM_SUCCESS) {
        printf("bitwise_and test NV12 failed!\n");
        exit(-1);
    }
    imwrite("nv12_bitwise_and.jpg", output);
    bmcv::print(output);

    return;
}

static void test_bitwise_or(int argc, const char** argv) {
  bm_handle_t handle = NULL;
  bm_status_t ret;
  /*-----------------------test BGR:-----------------------*/
  printf("test BGR:\n");
  Mat frame0 = imread(argv[2], IMREAD_COLOR, g_device_id);
  Mat frame1 = imread(argv[3], IMREAD_COLOR, g_device_id);
  Mat output(frame0.size(), frame0.type());
  bool update = true;
  handle = frame0.u->hid ? frame0.u->hid : bmcv::getCard();
  ret = bmcv::bitwise_or(frame0, frame1, output, update);
  if (ret != BM_SUCCESS) {
      printf("bitwise_or test BGR failed!\n");
      exit(-1);
  }
  bmcv::downloadMat(output);
  imwrite("bgr_bitwise_or.png", output);
  bmcv::print(output);

  Mat cv_output;
  cv::bitwise_or(frame0, frame1, cv_output);
  imwrite("cv_bgr_bitwise_or.png", cv_output);
  bmcv::print(cv_output);

  /*-----------------------test GRAY:-----------------------*/
  frame0.release();
  frame1.release();
  output.release();
  cv_output.release();
  printf("test GRAY:\n");
  frame0 = imread(argv[2], IMREAD_GRAYSCALE, g_device_id);
  frame1 = imread(argv[3], IMREAD_GRAYSCALE, g_device_id);
  ret = bmcv::bitwise_or(frame0, frame1, output, update);
  if (ret != BM_SUCCESS) {
      printf("bitwise_or test GRAY failed!\n");
      exit(-1);
  }
  bmcv::downloadMat(output);
  imwrite("gray_bitwise_or.png", output);
  bmcv::print(output);

  cv::bitwise_or(frame0, frame1, cv_output);
  imwrite("cv_gray_bitwise_or.png", cv_output);
  bmcv::print(cv_output);

  /*-----------------------test NV12:-----------------------*/
  frame0.release();
  frame1.release();
  output.release();
  cv_output.release();
  printf("test NV12:\n");
  frame0 = imread(argv[2], IMREAD_AVFRAME, g_device_id);
  frame1 = imread(argv[3], IMREAD_AVFRAME, g_device_id);

  int p_plane_stride[3], plane_size[3];
  int wscale[3], hscale[3];
  int plane_num = av::get_scale_and_plane(AV_PIX_FMT_NV12 , wscale, hscale);
  for (int i = 0; i < plane_num; i++) {
      p_plane_stride[i] = frame0.avCols() / wscale[i];
      plane_size[i] = p_plane_stride[i] * (frame0.avRows() / hscale[i]);
  }
  int id = bmcv::getId(handle);
  AVFrame *f_0 = av::create(
      frame0.rows, frame0.cols, AV_PIX_FMT_NV12, nullptr, 0, -1, p_plane_stride, plane_size);
  Mat frame0_nv12;
  frame0_nv12.create(f_0, id);
  bmcv::convert(frame0, frame0_nv12);

  AVFrame *f_1 = av::create(
      frame0.rows, frame0.cols, AV_PIX_FMT_NV12, nullptr, 0, -1, p_plane_stride, plane_size);
  Mat frame1_nv12;
  frame1_nv12.create(f_1, id);
  bmcv::convert(frame1, frame1_nv12);

  ret = bmcv::bitwise_or(frame0_nv12, frame1_nv12, output, update);
  if (ret != BM_SUCCESS) {
      printf("bitwise_or test NV12 failed!\n");
      exit(-1);
  }
  imwrite("nv12_bitwise_or.jpg", output);
  bmcv::print(output);

  return;
}

static void test_bitwise_xor(int argc, const char** argv) {
  bm_handle_t handle = NULL;
  bm_status_t ret;
  /*-----------------------test BGR:-----------------------*/
  printf("test BGR:\n");
  Mat frame0 = imread(argv[2], IMREAD_COLOR, g_device_id);
  Mat frame1 = imread(argv[3], IMREAD_COLOR, g_device_id);
  Mat output(frame0.size(), frame0.type());
  bool update = true;
  handle = frame0.u->hid ? frame0.u->hid : bmcv::getCard();
  ret = bmcv::bitwise_xor(frame0, frame1, output, update);
  if (ret != BM_SUCCESS) {
      printf("bitwise_xor test BGR failed!\n");
      exit(-1);
  }
  bmcv::downloadMat(output);
  imwrite("bgr_bitwise_xor.png", output);
  bmcv::print(output);

  Mat cv_output;
  cv::bitwise_xor(frame0, frame1, cv_output);
  imwrite("cv_bgr_bitwise_xor.png", cv_output);
  bmcv::print(cv_output);

  /*-----------------------test GRAY:-----------------------*/
  frame0.release();
  frame1.release();
  output.release();
  cv_output.release();
  printf("test GRAY:\n");
  frame0 = imread(argv[2], IMREAD_GRAYSCALE, g_device_id);
  frame1 = imread(argv[3], IMREAD_GRAYSCALE, g_device_id);
  ret = bmcv::bitwise_xor(frame0, frame1, output, update);
  if (ret != BM_SUCCESS) {
      printf("bitwise_xor test GRAY failed!\n");
      exit(-1);
  }
  bmcv::downloadMat(output);
  imwrite("gray_bitwise_xor.png", output);
  bmcv::print(output);

  cv::bitwise_xor(frame0, frame1, cv_output);
  imwrite("cv_gray_bitwise_xor.png", cv_output);
  bmcv::print(cv_output);

  /*-----------------------test NV12:-----------------------*/
  frame0.release();
  frame1.release();
  output.release();
  cv_output.release();
  printf("test NV12:\n");
  frame0 = imread(argv[2], IMREAD_AVFRAME, g_device_id);
  frame1 = imread(argv[3], IMREAD_AVFRAME, g_device_id);

  int p_plane_stride[3], plane_size[3];
  int wscale[3], hscale[3];
  int plane_num = av::get_scale_and_plane(AV_PIX_FMT_NV12 , wscale, hscale);
  for (int i = 0; i < plane_num; i++) {
      p_plane_stride[i] = frame0.avCols() / wscale[i];
      plane_size[i] = p_plane_stride[i] * (frame0.avRows() / hscale[i]);
  }
  int id = bmcv::getId(handle);
  AVFrame *f_0 = av::create(
      frame0.rows, frame0.cols, AV_PIX_FMT_NV12, nullptr, 0, -1, p_plane_stride, plane_size);
  Mat frame0_nv12;
  frame0_nv12.create(f_0, id);
  bmcv::convert(frame0, frame0_nv12);

  AVFrame *f_1 = av::create(
      frame0.rows, frame0.cols, AV_PIX_FMT_NV12, nullptr, 0, -1, p_plane_stride, plane_size);
  Mat frame1_nv12;
  frame1_nv12.create(f_1, id);
  bmcv::convert(frame1, frame1_nv12);

  ret = bmcv::bitwise_xor(frame0_nv12, frame1_nv12, output, update);
  if (ret != BM_SUCCESS) {
      printf("bitwise_xor test NV12 failed!\n");
      exit(-1);
  }
  imwrite("nv12_bitwise_xor.jpg", output);
  bmcv::print(output);

  return;
}

void print_usage(const char* prog) {
  printf("USAGE:\n");
  printf("    %s conv_1 <image1> [device_id]\n", prog);
  printf("    %s conv_4 <image1> <image2> <image3> <image4> [device_id]\n", prog);
  printf("    %s size <image> [device_id]\n", prog);
  printf("    %s video <rtsp_url> [device_id]\n", prog);
  printf("    %s image <jpeg_file> [device_id]\n", prog);
  printf("    %s cvt <jpeg_file> [device_id]\n", prog);
  printf("    %s bmcv2cv <jpeg_file> [device_id]\n", prog);
  printf("    %s bitwise_and <jpeg1> <jpeg2> [device_id]\n", prog);
  printf("    %s bitwise_or <jpeg1> <jpeg2> [device_id]\n", prog);
}

static const test_case_entry* parse_args(const char *arg) {
  for (size_t i = 0; i < TEST_CASE_COUNT; ++i) {
      if (strcmp(arg, test_case_table[i].name) == 0) {
          return &test_case_table[i];
      }
  }
  printf("Invalid test case: %s\n", arg);
  return NULL;
}

static bool check_argc(int argc, const int* valid_counts, int count_len) {
  for (int i = 0; i < count_len; i++) {
      if (argc == valid_counts[i]) {
          return true;
      }
  }
  return false;
}

static void parse_device_id(int argc, const char **argv) {
    g_device_id = atoi(argv[argc - 1]);
    printf("bm_device %d used.\n", g_device_id);
}

int main(int argc, const char** argv)
{
    if (argc < 3) {
        printf("Invalid input!\n");
        print_usage(argv[0]);
        return -1;
    }

    const test_case_entry* tc_entry = parse_args(argv[1]);
    if (!tc_entry) {
        print_usage(argv[0]);
        return -1;
    }

    if (!check_argc(argc, tc_entry->valid_argc_count, tc_entry->valid_argc_len)) {
      print_usage(argv[0]);
      return -1;
  }
  parse_device_id(argc, argv);
  tc_entry->test_func(argc, argv);
}