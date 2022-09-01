/*
 * vppresize - Bitmain vpp resize test
 *
 * Copyright (C) 2018 he li
 *
 * li he  <he.li@bitmain.com>
 *
 * This program is designed to test the function of the VPP resize.
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
        cout << "vppresize in.bmp height width out.bmp" << endl;
        return -1;
    }

    Mat src = imread(argv[1]);
    if (!src.data) {
        cout << "load img err!" << endl;
        return -1;
    }

	char* imageName1 = argv[4];
	CV_Assert(imageName1 != NULL);

    int h = atoi(argv[2]);
    int w = atoi(argv[3]);

    Mat dst(h, w, CV_8UC3);

    vpp::resize(src, dst);

    imwrite(imageName1, dst);
#else
    printf("only support for arm\n");
#endif
    return 0;
}

