/*
 * vppresize - Bitmain vpp split test
 *
 * Copyright (C) 2019 xingxing.li
 *
 *  <xingxing.li@bitmain.com>
 *
 * This program is designed to test the function of opencv cvtcolor.
 */
#include <stdio.h>

#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#ifndef WIN32
#include <sys/time.h>
#endif
#include <stdlib.h>
using namespace std;
using namespace cv;

#define STRIDE_ALIGN    (16)
#define ALIGN(x, mask)  (((x) + ((mask)-1)) & ~((mask)-1))

int main(int argc, char **argv)
{
#ifdef __ARM_ARCH
    struct timeval tv0,tv1;
    int actionCode = -1;
    int FC3test = -1;
    int tooltype = 3;
    if (argc <3) {
        cout << "usage:" << endl;
        cout << "bmcvvpptest in.jpeg resize/SingleCrop/MultiCrop/border/split/merge outfile(no suffix)" << endl;
        return -1;
    }

    Mat src,dst1;
    if(strcmp(argv[2],"resize") == 0)
    {
        gettimeofday(&tv0,NULL);
        src = imread(argv[1], -1); /* ----> BGRpacked */
        gettimeofday(&tv1,NULL);
        printf("imread:time_consume(us): %ld,src.channels()=%d\n",(tv1.tv_sec-tv0.tv_sec)*1000000+(tv1.tv_usec-tv0.tv_usec),src.channels());
        if (!src.data ) {
        cout << "load img err!" << endl;
        return -1;
        }

        printf(" hi xingxing src.stride=%d \n",src.step[0]);
        gettimeofday(&tv0,NULL);

        resize(src,dst1,Size(src.cols*3/4,src.rows*3/4));

        FILE* file_resize = fopen(argv[3], "wb");
        if (!file_resize) {
            std::cerr << "Failed to open output file!" << std::endl;
            return -1;
        }
        fwrite(dst1.data, sizeof(uchar), dst1.total() * dst1.channels(),
               file_resize);
        fclose(file_resize);

        gettimeofday(&tv1,NULL);
        printf("%s:time_consume(us): %ld\n",argv[2],(tv1.tv_sec-tv0.tv_sec)*1000000+(tv1.tv_usec-tv0.tv_usec));
        // imwrite(string(argv[3])+ ".jpg", dst1);
    }
#if (defined HAVE_BMCV) && (defined USING_SOC)
    else if (strcmp(argv[2],"SingleCrop")==0)
    {
        gettimeofday(&tv0,NULL);
        src = imread(argv[1], -1);
        gettimeofday(&tv1,NULL);
        printf("imread:time_consume(us): %ld,src.channels()=%d\n",(tv1.tv_sec-tv0.tv_sec)*1000000+(tv1.tv_usec-tv0.tv_usec),src.channels());
        if (!src.data ) {
        cout << "load img err!" << endl;
        return -1;
        }
        printf("src.stride=%d \n",src.step[0]);
        gettimeofday(&tv0,NULL);
        bmcv::hwCrop(src,Rect(src.cols*1/4,src.rows*1/4,src.cols*3/4,src.rows*3/4),dst1);
        gettimeofday(&tv1,NULL);
        printf("%s:time_consume(us): %ld\n",argv[2],(tv1.tv_sec-tv0.tv_sec)*1000000+(tv1.tv_usec-tv0.tv_usec));

        FILE* file_SingleCrop = fopen(argv[3], "wb");
        if (!file_SingleCrop) {
            std::cerr << "Failed to open output file!" << std::endl;
            return -1;
        }
        fwrite(dst1.data, sizeof(uchar), dst1.total() * dst1.channels(),
               file_SingleCrop);
        fclose(file_SingleCrop);

        // imwrite(string(argv[3])+ ".jpg", dst1);
    }
    else if (strcmp(argv[2],"MultiCrop")==0)
    {
        gettimeofday(&tv0,NULL);
        src = imread(argv[1], -1);
        gettimeofday(&tv1,NULL);
        printf("imread:time_consume(us): %ld,src.channels()=%d\n",(tv1.tv_sec-tv0.tv_sec)*1000000+(tv1.tv_usec-tv0.tv_usec),src.channels());
        if (!src.data ) {
        cout << "load img err!" << endl;
        return -1;
        }
        printf("src.stride=%d \n",src.step[0]);
        gettimeofday(&tv0,NULL);
        std::vector<Rect> local;
        local.push_back(Rect(src.cols*1/4,src.rows*1/4,src.cols*3/4,src.rows*3/4));
        local.push_back(Rect(0,0,src.cols/2,src.rows/2));
        std::vector<Mat> dst_vec;
        bmcv::hwMultiCrop(src,local,dst_vec);
        gettimeofday(&tv1,NULL);
        printf("%s:time_consume(us): %ld\n",argv[2],(tv1.tv_sec-tv0.tv_sec)*1000000+(tv1.tv_usec-tv0.tv_usec));

        std::string filename_1 = std::string(argv[3]) + "_1";
        FILE* file_Multicrop_1 = fopen(filename_1.c_str(), "wb");
        if (!file_Multicrop_1) {
            std::cerr << "Failed to open output file!" << std::endl;
            return -1;
        }
        fwrite(dst_vec[0].data, sizeof(uchar),
               dst_vec[0].total() * dst_vec[0].channels(), file_Multicrop_1);
        fclose(file_Multicrop_1);

        std::string filename_2 = std::string(argv[3]) + "_2";
        FILE* file_Multicrop_2 = fopen(filename_2.c_str(), "wb");
        if (!file_Multicrop_2) {
            std::cerr << "Failed to open output file!" << std::endl;
            return -1;
        }
        fwrite(dst_vec[1].data, sizeof(uchar),
               dst_vec[1].total() * dst_vec[1].channels(), file_Multicrop_2);
        fclose(file_Multicrop_2);

        // imwrite(string(argv[3])+"1"+ ".jpg", dst_vec[0]);
        // imwrite(string(argv[3])+"2"+ ".jpg", dst_vec[1]);
    }
#endif
    else if (strcmp(argv[2],"border")==0)
    {
        gettimeofday(&tv0,NULL);
        src = imread(argv[1], -1);
        gettimeofday(&tv1,NULL);
        printf("imread:time_consume(us): %ld,src.channels()=%d\n",(tv1.tv_sec-tv0.tv_sec)*1000000+(tv1.tv_usec-tv0.tv_usec),src.channels());
        if (!src.data ) {
        cout << "load img err!" << endl;
        return -1;
        }
        printf("src.stride=%d \n",src.step[0]);
        gettimeofday(&tv0,NULL);
        Mat crop_img = src(Rect(src.cols/2,src.rows/2,192,192));
        copyMakeBorder(crop_img, dst1,src.rows/30, src.rows/30, src.rows/30, src.rows/30,BORDER_CONSTANT,Scalar(255,0,0));
        //dst1 = vpp::border(src,src.rows/30, src.rows/30, src.rows/30, src.rows/30);
        gettimeofday(&tv1,NULL);
        printf("%s:time_consume(us): %ld\n",argv[2],(tv1.tv_sec-tv0.tv_sec)*1000000+(tv1.tv_usec-tv0.tv_usec));

        FILE* file_border = fopen(argv[3], "wb");
        if (!file_border) {
            std::cerr << "Failed to open output file!" << std::endl;
            return -1;
        }
        fwrite(dst1.data, sizeof(uchar), dst1.total() * dst1.channels(),
               file_border);
        fclose(file_border);

        // imwrite(string(argv[3])+ ".jpg", dst1);
    }
#if (defined HAVE_BMCV) && (defined USING_SOC)
    else if (strcmp(argv[2],"opencv_crop")==0)
    {
        src = imread(argv[1], -1);
        if (!src.data ) {
        cout << "load img err!" << endl;
        return -1;
        }
        printf("src.stride=%d \n",src.step[0]);
        Mat crop_img = src(Rect(src.cols/4,src.rows/4,700,700));
        Mat resized,resizedhw;
        gettimeofday(&tv0,NULL);
        resize(crop_img,resized,cv::Size(240,240),0,0,1);
        gettimeofday(&tv1,NULL);
        printf("%s:opencv time_consume(us): %ld\n",argv[2],(tv1.tv_sec-tv0.tv_sec)*1000000+(tv1.tv_usec-tv0.tv_usec));
        gettimeofday(&tv0,NULL);
        bmcv::hwResize( crop_img, resizedhw, cv::Size(240,240), 1);
        gettimeofday(&tv1,NULL);
        printf("%s:vpp time_consume(us): %ld\n",argv[2],(tv1.tv_sec-tv0.tv_sec)*1000000+(tv1.tv_usec-tv0.tv_usec));

        FILE* file_resize = fopen(argv[3], "wb");
        if (!file_resize) {
            std::cerr << "Failed to open output file!" << std::endl;
            return -1;
        }
        fwrite(resized.data, sizeof(uchar),
               resized.total() * resized.channels(), file_resize);
        fclose(file_resize);

        std::string filename_hwresize = "hw_" + std::string(argv[3]);
        FILE* file_hwresize = fopen(filename_hwresize.c_str(), "wb");
        if (!file_hwresize) {
            std::cerr << "Failed to open output file!" << std::endl;
            return -1;
        }
        fwrite(resizedhw.data, sizeof(uchar),
               resizedhw.total() * resizedhw.channels(), file_hwresize);
        fclose(file_hwresize);

        // imwrite(string(argv[3])+ ".jpg", resized);
        // imwrite(string(argv[3])+ "hw.jpg", resizedhw);
        int abdf = 0;

        for (int i=0;i<resizedhw.rows;i++)
        {
            for (int j=0;j< resizedhw.cols*3;j++)
            {
                int df = *(resizedhw.data+i*resizedhw.step[0]+j)-*(resized.data+i*resized.step[0]+j);
                if(df <0)
                {
                    df = df*(-1);
                }
                if (abdf < df)
                {
                    abdf = df;
                }
                /*
                if(lowdf< -2 || highdf>2)
                {
                    printf("h=%d,w=%d \n",i,j);
                }*/
            }
        }
        printf("vpp-opencv max RGB df=%d:\n",abdf);
        /*Mat diff;
        cv::absdiff(resized, resizedhw, diff);
        printf("diff: \n");
        for (int i=0;i<resizedhw.rows;i++)
        {
            for (int j=0;j< resizedhw.cols*3;j++)
            {
               printf("%d,",*(diff.data+diff.step[0]*i+j));
            }
            printf("\n");
        }*/

    }
#endif
    else if (strcmp(argv[2],"transpose")==0)
    {
        gettimeofday(&tv0,NULL);
        src = imread(argv[1], -1);
        gettimeofday(&tv1,NULL);
        printf("imread:time_consume(us): %ld,src.channels()=%d\n",(tv1.tv_sec-tv0.tv_sec)*1000000+(tv1.tv_usec-tv0.tv_usec),src.channels());
        if (!src.data ) {
        cout << "load img err!" << endl;
        return -1;
        }
        printf("src.stride=%d \n",src.step[0]);
        Mat crop_img = src(Rect(src.cols/2,src.rows/2,192,192));
        transpose(crop_img,dst1);
        printf("%s:time_consume(us): %ld\n",argv[2],(tv1.tv_sec-tv0.tv_sec)*1000000+(tv1.tv_usec-tv0.tv_usec));

        FILE* file_transpose = fopen(argv[3], "wb");
        if (!file_transpose) {
            std::cerr << "Failed to open output file!" << std::endl;
            return -1;
        }
        fwrite(dst1.data, sizeof(uchar), dst1.total() * dst1.channels(),
               file_transpose);
        fclose(file_transpose);

        // imwrite(string(argv[3])+ ".jpg", dst1);
    }
    else if (strcmp(argv[2],"rectangle")==0)
    {
        gettimeofday(&tv0,NULL);
        src = imread(argv[1], -1);
        gettimeofday(&tv1,NULL);
        printf("imread:time_consume(us): %ld,src.channels()=%d\n",(tv1.tv_sec-tv0.tv_sec)*1000000+(tv1.tv_usec-tv0.tv_usec),src.channels());
        if (!src.data ) {
        cout << "load img err!" << endl;
        return -1;
        }
        printf("src.stride=%d \n",src.step[0]);
        Mat crop_img = src(Rect(src.cols/2,src.rows/2,192,192));
        transpose(crop_img,dst1);
        printf("%s:time_consume(us): %ld\n",argv[2],(tv1.tv_sec-tv0.tv_sec)*1000000+(tv1.tv_usec-tv0.tv_usec));

        FILE* file_rectangle = fopen(argv[3], "wb");
        if (!file_rectangle) {
            std::cerr << "Failed to open output file!" << std::endl;
            return -1;
        }
        fwrite(dst1.data, sizeof(uchar), dst1.total() * dst1.channels(),
               file_rectangle);
        fclose(file_rectangle);

        // imwrite(string(argv[3])+ ".jpg", dst1);
    }
 #if (defined HAVE_BMCV) && (defined USING_SOC)
    else if(strcmp(argv[2],"resize_anal") == 0)\
    {
        cv::Mat in_mat1 (Size(128,128), 16 );
        //cv::Scalar mean = cv::Scalar::all(127);
        //cv::Scalar stddev = cv::Scalar::all(40.f);
       // cv::randn(in_mat1, mean, stddev);
        for(int i=0;i<in_mat1.rows;i++)
        {
           for(int j=i;j<in_mat1.cols;j++)
           {
              *(in_mat1.data+i*in_mat1.step[0]+3*j)=(char)i;
              *(in_mat1.data+i*in_mat1.step[0]+3*j+1)=(char)i;
              *(in_mat1.data+i*in_mat1.step[0]+3*j+2)=(char)i;
           }
           for(int j=i;j<in_mat1.cols;j++)
           {
              *(in_mat1.data+j*in_mat1.step[0]+3*i)=(char)i;
              *(in_mat1.data+j*in_mat1.step[0]+3*i+1)=(char)i;
              *(in_mat1.data+j*in_mat1.step[0]+3*i+2)=(char)i;
           }
           printf("i=%d,",*(in_mat1.data+i*in_mat1.step[0]+3*i));
        }
        printf("in_mat1: \n");
        for (int i=0;i<in_mat1.rows;i++)
        {
            for (int j=0;j< in_mat1.cols;j++)
            {
               printf("%d,",*((char*)in_mat1.data+in_mat1.step[0]*i+j*3));
            }
            printf("\n");
        }
        printf("resize_anal chanel= %d:\n",in_mat1.channels());
        Mat resized,resizedhw;
        resize(in_mat1,resized,cv::Size(30,30),0,0,0);
        bmcv::hwResize( in_mat1, resizedhw, cv::Size(30,30), 0);
        int abdf = 0;

        for (int i=0;i<resizedhw.rows;i++)
        {
            for (int j=0;j< resizedhw.cols*3;j++)
            {
                int df = *(resizedhw.data+i*resizedhw.step[0]+j)-*(resized.data+i*resized.step[0]+j);
                if(df <0)
                {
                    df = df*(-1);
                }
                if (abdf < df)
                {
                    abdf = df;
                }
                printf("%d,",abdf);
                /*
                if(lowdf< -2 || highdf>2)
                {
                    printf("h=%d,w=%d \n",i,j);
                }*/
            }
            printf("\n");

        }
        printf("vpp-opencv max RGB df=%d:\n",abdf);

        Mat diff;
        cv::absdiff(resized, resizedhw, diff);
        printf("resizedhw: \n");
        for (int i=0;i<resized.rows;i++)
        {
            for (int j=0;j< resizedhw.cols*3;j++)
            {
               printf("%d,",*(resizedhw.data+diff.step[0]*i+j));
            }
            printf("\n");
        }
        printf("resized: \n");
        for (int i=0;i<resized.rows;i++)
        {
            for (int j=0;j< resized.cols*3;j++)
            {
               printf("%d,",*(resized.data+diff.step[0]*i+j));
            }
            printf("\n");
        }
    }
   else if(strcmp(argv[2],"resize_anal2") == 0)
   {
        cv::Mat in_mat1 (Size(30,30), 16 );
        cv::Scalar mean = cv::Scalar::all(127);
        cv::Scalar stddev = cv::Scalar::all(40.f);
        cv::randn(in_mat1, mean, stddev);
        printf("resize_anal chanel= %d:\n",in_mat1.channels());
        Mat resized,resizedhw;
        resize(in_mat1,resized,cv::Size(128,128),0,0,1);
        bmcv::hwResize( in_mat1, resizedhw, cv::Size(128,128), 1);
        int abdf = 0;

        for (int i=0;i<resizedhw.rows;i++)
        {
            for (int j=0;j< resizedhw.cols*3;j++)
            {
                int df = *(resizedhw.data+i*resizedhw.step[0]+j)-*(resized.data+i*resized.step[0]+j);
                if(df <0)
                {
                    df = df*(-1);
                }
                if (abdf < df)
                {
                    abdf = df;
                }
                printf("%d,",abdf);
                /*
                if(lowdf< -2 || highdf>2)
                {
                    printf("h=%d,w=%d \n",i,j);
                }*/
            }
            printf("\n");
        }
        printf("vpp-opencv max RGB df=%d:\n",abdf);

        Mat diff;
        cv::absdiff(resized, resizedhw, diff);
        printf("diff: \n");
        for (int i=0;i<resizedhw.rows;i++)
        {
            for (int j=0;j< resizedhw.cols*3;j++)
            {
               printf("%d,",*(diff.data+diff.step[0]*i+j));
            }
            printf("\n");
        }
    }
#endif
#else
    printf("only support for arm\n");
#endif
    return 0;
}
