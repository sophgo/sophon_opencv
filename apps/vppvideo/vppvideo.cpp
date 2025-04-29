/*
 * vppresize - Bitmain vpp NV12SP to RGB24 packet convert test
 *
 * Copyright (C) 2018 he li
 *
 * li he  <he.li@bitmain.com>
 *
 * This program is designed to test the function of the VPP NV12SP to RGB24 packet convert.
 */

#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgproc/vpp.hpp>

using namespace std;
using namespace cv;

int main(int argc, char **argv)
{
#ifdef __ARM_ARCH
    if (argc != 8) {
        cout << "usage:" << endl;
        cout << "vppvideo height width step mode y.bin uv.bin out.bmp" << endl;
        cout << "mode: 0 : linear, 1 : tile"<<endl;
        return -1;
    }

    int h = atoi(argv[1]);
    int w = atoi(argv[2]);
    int s = atoi(argv[3]);
    int mode = atoi(argv[4]);

    char* imageName1 = argv[7];
	CV_Assert(imageName1 != NULL);

    Mat src0;
    src0.allocator = hal::getAllocator();
    src0.create(h+256, w+256, CV_8UC1);
    printf("data: %p 0x%llx\n", src0.data, src0.u->addr);
    Mat src1;
    src1.allocator = hal::getAllocator();
    src1.create(h, w, CV_8UC1);
    printf("data: %p 0x%llx\n", src1.data, src1.u->addr);

    FILE *fp0 = fopen(argv[5], "rb");
    fseek(fp0, 0, SEEK_END);
    int len0 = ftell(fp0);
    printf("len0 : %d\n", len0);

    fseek(fp0, 0, SEEK_SET);
    fread(src0.data, 1, len0, fp0);

    FILE *fp1 = fopen(argv[6], "rb");
    fseek(fp1, 0, SEEK_END);
    int len1 = ftell(fp1);
    printf("len1 : %d\n", len1);

    fseek(fp1, 0, SEEK_SET);
    fread(src1.data, 1, len1, fp1);

    IplImage img;
    img.height = h;
    img.width = w;
    img.step = s;

#ifdef ION_CACHE
    img.addr0 = src0.u->fd;
    img.addr1 = src1.u->fd;
    Mat dst = vpp::iplImageToMat(&img, mode);
#else
    img.addr0 = src0.u->addr;
    img.addr1 = src1.u->addr;
    Mat dst = vpp::iplImageToMat(&img, mode);
#endif

    printf("data: %p 0x%llx\n", dst.data, dst.u->addr);
    imwrite(imageName1, dst);
#else
    printf("only support for arm\n");
#endif
    return 0;
}

