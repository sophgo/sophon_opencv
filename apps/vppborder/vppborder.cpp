/*
 * vppresize - Bitmain vpp border test
 *
 * Copyright (C) 2018 he li
 *
 * li he  <he.li@bitmain.com>
 *
 * This program is designed to test the function of the VPP border.
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
    if (argc != 7) {
        cout << "usage:" << endl;
        cout << "vppborder in.bmp out.bmp top bottom left right" << endl;
        return -1;
    }

    Mat src = imread(argv[1]);
    if (!src.data) {
        cout << "load img err!" << endl;
        return -1;
    }
    char* imageName1 = argv[2];
	CV_Assert(imageName1 != NULL);

    int top = atoi(argv[3]);
    int bottom = atoi(argv[4]);
    int left = atoi(argv[5]);
    int right = atoi(argv[6]);

    Mat dst = vpp::border(src, top, bottom, left, right);
    imwrite(imageName1, dst);
#else
    printf("only support for arm\n");
#endif
    return 0;
}
