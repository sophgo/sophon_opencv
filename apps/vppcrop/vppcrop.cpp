/*
 * vppresize - Bitmain vpp crop test
 *
 * Copyright (C) 2018 he li
 *
 * li he  <he.li@bitmain.com>
 *
 * This program is designed to test the function of the VPP crop.
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
        cout << "vppcrop in.bmp out.bmp X-axis Y-axis width height" << endl;
        return -1;
    }

    Mat src = imread(argv[1]);
    if (!src.data) {
        cout << "load img err!" << endl;
        return -1;
    }

    int x1 = atoi(argv[3]);
    int y1 = atoi(argv[4]);
    int w1 = atoi(argv[5]);
    int h1 = atoi(argv[6]);

    char* imageName1 = argv[2];
	CV_Assert(imageName1 != NULL);
    CV_Assert((x1 >= 0) && (x1 <= src.cols));
    CV_Assert((y1 >= 0) && (y1 <= src.rows));
    CV_Assert((w1 > 0) && (w1 <= src.cols));
    CV_Assert((h1 > 0) && (h1 <= src.rows));

    Mat dst[2];
    vector<Rect> loca;
    loca.push_back(Rect(x1, y1, w1, h1));
    loca.push_back(Rect(x1/2, y1/2, w1, h1));

    vpp::crop(src, loca, dst);
    imwrite(imageName1, dst[0]);
    imwrite("crop2.bmp", dst[1]);
#else
    printf("only support for arm\n");
#endif
    return 0;
}

