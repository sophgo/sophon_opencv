/*
 * vppresize - Bitmain vpp split test
 *
 * Copyright (C) 2018 he li
 *
 * li he  <he.li@bitmain.com>
 *
 * This program is designed to test the function of the VPP split.
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
    if (argc != 5) {
        cout << "usage:" << endl;
        cout << "vppsplit in.bmp out1.bmp out2.bmp out3.bmp" << endl;
        return -1;
    }

    Mat src = imread(argv[1]);
    if (!src.data) {
        cout << "load img err!" << endl;
        return -1;
    }
    char* imageName1 = argv[2];
    char* imageName2 = argv[3];
    char* imageName3 = argv[4];

    Mat dst[3];
    vpp::split(src, dst);
    imwrite(imageName1, dst[0]);
    imwrite(imageName2, dst[1]);
    imwrite(imageName3, dst[2]);
#else
    printf("only support for arm\n");
#endif
    return 0;
}

