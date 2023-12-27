/*
 *
 * Copyright (C) 2019 xingxing.li
 *
 *  <xingxing.li@bitmain.com>
 *
 * This program is designed to test the function of opencv cvtcolor.
 */

#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <sys/time.h>
#include <stdlib.h>
using namespace std;
using namespace cv;

#define STRIDE_ALIGN    (16)
#define ALIGN(x, mask)  (((x) + ((mask)-1)) & ~((mask)-1))
static void alignWithStride(uchar* uv, int stride, int width, int uvheight)
{
    if(stride / 2 <= width / 2 || stride % 2 != 0) {
        return;
    }

    int halfStride = stride / 2;
    int halfSWidth = width / 2;
    printf("alignWithStride xingxing,uvheight=%d,halfSWidth=%d,halfStride=%d\n",uvheight,halfSWidth,halfStride);
    for(int j = 0; j< uvheight; j++) {
        for(int i = 0; i < halfSWidth; i++) {
            *(uv + j*stride + halfSWidth + i) = *(uv + j*stride + halfStride + i);
        }
    }
}
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

int main(int argc, char **argv)
{
#ifdef __ARM_ARCH
    struct timeval tv0,tv1;
    int actionCode = -1;
    int FC3test = -1;
    int tooltype = 3;
    if (argc <3) {
        cout << "usage:" << endl;
        cout << "cvtColortest in.bmp COLOR_action action_code out2.bmp  isusetool(0=opencv,1=only-libyuv-no-vpp,3=vpp,libyuv)" << endl;
        cout <<"sample:  ./cvtcolortest 422_1920.bmp COLOR_RGB2BGR 4  out_BGR.bmp" << endl;
        cout <<"sample:  ./cvtcolortest  20190222105123790_1.jpg  NV12ToRGB 90 out.jpg 1 " << endl;
        cout <<"sample:  ./cvtcolortest .20190222105123790_1.jpg COLOR_RGB2YUV_I420  128 out_420.bmp 1" << endl;
        cout << "sample: ./cvtcolortest  ./422_1920.bmp COLOR_RGB2YUV_I420  127 out_420.bmp 1 " << endl;
        cout <<"sample:  ./cvtcolortest 422_1920.bmp COLOR_RGB2BGR 4  out_BGR.bmp 1 1  for check  cv_32F" << endl;
        cout <<"sample:  ./cvtcolortest 422_1920.bmp analysis 4  out_BGR.bmp  ---Accuracy analysis compare to opencv" << endl;
        return -1;
    }

    actionCode = atoi(argv[3]);
    if (actionCode>143 ||actionCode<-1)
    {
        printf("bad action code\n");
    }
    if (argc>5)
    {
        tooltype = atoi(argv[5]);
    }
    if (argc>6)
    {
        FC3test = atoi(argv[6]);
    }
    vpp::enableVPPConverter(true);
    cv::enableLibyuvConverter(true);

    if(tooltype == 0)
    {
        vpp::enableVPPConverter(false);
        cv::enableLibyuvConverter(false);
        printf("==== use opencv ==== \n");
    }
    else if(tooltype == 1)
    {
        vpp::enableVPPConverter(false);
        printf("==== use libyuv ==== \n");
    }

    Mat src;
    if(FC3test ==1  && src.type() == CV_8UC3)
    {
        gettimeofday(&tv0,NULL);
        src = imread(argv[1], -1);
        gettimeofday(&tv1,NULL);
        printf("imread:time_consume(us): %ld\n",(tv1.tv_sec-tv0.tv_sec)*1000000+(tv1.tv_usec-tv0.tv_usec));
        if (!src.data ) {
        cout << "load img err!" << endl;
        return -1;
        }
        Mat src_32F;
        src.convertTo(src_32F,CV_32FC3);
        src = src_32F;
    }
    Mat dst1;
    if (strcmp(argv[2],"clone")==0)
    {
        gettimeofday(&tv0,NULL);
        src = imread(argv[1], -1);  /* ----> BGRpacked */
        gettimeofday(&tv1,NULL);
        printf("imread:time_consume(us): %ld\n",(tv1.tv_sec-tv0.tv_sec)*1000000+(tv1.tv_usec-tv0.tv_usec));
        if (!src.data ) {
        cout << "load img err!" << endl;
        return -1;
        }

        gettimeofday(&tv0,NULL);
        dst1 = src.clone();
        gettimeofday(&tv1,NULL);
        printf("%s:time_consume(us): %ld\n",argv[2],(tv1.tv_sec-tv0.tv_sec)*1000000+(tv1.tv_usec-tv0.tv_usec));
    }
#if (defined HAVE_BMCV) && (defined USING_SOC)
    // else if (strcmp(argv[2],"NV12ToRGB")==0)  /*special test  for  NV12ToRGB*/
    // {
    //     Mat dstc3;
    //     if(tooltype == 3)
    //     {
    //        dstc3.allocator = hal::getAllocator();
    //     }
    //     dstc3.create(Size(1920,1620), CV_8UC1);
    //     printf("dstc3.cols=%d,dstc3.rows=%d \n",dstc3.cols,dstc3.rows);
    //     FILE* fp1 = fopen("y-ffmpeg.bin","rb");
    //     fread(dstc3.data,1,1920*1080,fp1);
    //     fclose(fp1);
    //     FILE* fp2 = fopen("uv-ffmpeg.bin","rb");
    //     fread(dstc3.data+1920*1080,1,1920*540,fp2);
    //     fclose(fp2);

    //     gettimeofday(&tv0,NULL);
    //     bmcv::hwColorCvt(dstc3,  dst1, FORMAT_NV12,FORMAT_RGB_PACKED, Size(dstc3.cols, dstc3.rows * 2 / 3), 3);
    //     gettimeofday(&tv1,NULL);
    //     printf("%s:time_consume(us): %ld\n",argv[2],(tv1.tv_sec-tv0.tv_sec)*1000000+(tv1.tv_usec-tv0.tv_usec));

    //     Mat dstopecv;
    //     gettimeofday(&tv0,NULL);
    //     cv::cvtColor(dstc3,dstopecv,actionCode);
    //     gettimeofday(&tv1,NULL);
    //     printf("%s:time_consume(us): %ld\n",argv[2],(tv1.tv_sec-tv0.tv_sec)*1000000+(tv1.tv_usec-tv0.tv_usec));
    //     int lowdf = 128;
    //     int highdf = 0;
    //     for (int i=0;i<dstopecv.rows;i++)
    //     {
    //         for (int j=0;j< dstopecv.cols*3;j++)
    //         {
    //             int df = *(dstopecv.data+i*dstopecv.step[0]+j)-*(dst1.data+i*dst1.step[0]+j);
    //             if (df < lowdf)
    //             {
    //                 lowdf = df;
    //             }
    //             if (df > highdf)
    //             {
    //                 highdf = df;
    //             }
    //         }
    //     }
    //     printf("vpp-opencv min RGB df=%d:\n",lowdf);
    //     printf("vpp-opencv max RGB df=%d:\n",highdf);
    //     //char* imageName5 = argv[6];
    //     //imwrite(imageName5, dstopecv);
    // }
    else if (strcmp(argv[2],"analysis")==0)
    {
        gettimeofday(&tv0,NULL);
        src = imread(argv[1], -1);
        gettimeofday(&tv1,NULL);
        printf("imread:time_consume(us): %ld\n",(tv1.tv_sec-tv0.tv_sec)*1000000+(tv1.tv_usec-tv0.tv_usec));
        if (!src.data ) {
        cout << "load img err!" << endl;
        return -1;
        }
        //struct timeval tv;
        //long long timestamp;
        //gettimeofday(&tv,NULL);
        //timestamp  = tv.tv_sec*1000 + tv.tv_usec/1000;
        //printf(" in main timestamp : %lld\n", timestamp);
        cv::enableLibyuvConverter(true);
        gettimeofday(&tv0,NULL);
        cv::cvtColor(src,dst1,actionCode);
        gettimeofday(&tv1,NULL);
        printf("%s:libyuv_time_consume(us): %ld\n",argv[2],(tv1.tv_sec-tv0.tv_sec)*1000000+(tv1.tv_usec-tv0.tv_usec));
        cv::enableLibyuvConverter(false);
        Mat dst2;
        gettimeofday(&tv0,NULL);
        cv::cvtColor(src,dst2,actionCode);
        gettimeofday(&tv1,NULL);
        printf("%s:opencv_time_consume(us): %ld\n",argv[2],(tv1.tv_sec-tv0.tv_sec)*1000000+(tv1.tv_usec-tv0.tv_usec));
        int com_width = dst2.cols;
        int abdf = 0;
        for (int i=0;i<dst2.rows;i++)
        {
            for (int j=0;j< com_width;j++)
            {
                int df = *(dst2.data+i*dst2.step[0]+j)-*(dst1.data+i*dst1.step[0]+j);
                if(df <0)
                {
                   df = df*(-1);
                }
                if (abdf < df)
                {
                   abdf = df;
                }

            }
        }
        printf("libyuv-opencv min  df=%d:\n",abdf);
    }
    else if (strcmp(argv[2],"analysisvppBGR2420p")==0)
    {
        gettimeofday(&tv0,NULL);
        src = imread(argv[1], -1);
        gettimeofday(&tv1,NULL);
        printf("imread:time_consume(us): %ld\n",(tv1.tv_sec-tv0.tv_sec)*1000000+(tv1.tv_usec-tv0.tv_usec));
        if (!src.data ) {
        cout << "load img err!" << endl;
        return -1;
        }
        gettimeofday(&tv0,NULL);
        cv::cvtColor(src,dst1,COLOR_BGR2YUV_I420);
        gettimeofday(&tv1,NULL);
        printf("%s:opencv_time_consume(us): %ld\n",argv[2],(tv1.tv_sec-tv0.tv_sec)*1000000+(tv1.tv_usec-tv0.tv_usec));
        Mat dst2;
        gettimeofday(&tv0,NULL);
        bmcv::hwColorCvt(src,  dst2, FORMAT_BGR_PACKED, FORMAT_YUV420P, Size(src.cols, src.rows * 3 / 2), 1);
        gettimeofday(&tv1,NULL);
        printf("%s:vpp_time_consume(us): %ld\n",argv[2],(tv1.tv_sec-tv0.tv_sec)*1000000+(tv1.tv_usec-tv0.tv_usec));
        int com_width = dst2.cols;

        // FILE *dst1_file = fopen("vppout_dst1.yuv","wb");
        // printf("!!write file\n");
        // for (int i=0;i<dst1.rows;i++)
        // {
        //     fwrite(dst1.data+i*dst1.step[0],1,dst1.cols*dst1.channels(),dst1_file);
        // }
        // fclose(dst1_file);

        FILE *dst2_file = fopen("bgr2yuv420.bin","wb");
        printf("!!write file\n");
        for (int i=0;i<dst2.rows;i++)
        {
            fwrite(dst2.data+i*dst2.step[0],1,dst2.cols*dst2.channels(),dst2_file);
        }
        fclose(dst2_file);

        int abdf = 0;
        for (int i=0;i< dst2.rows-1;i++)
        {
            for (int j=0;j< dst2.cols;j++)
            {
                int df = *((unsigned char*)dst2.data+i*dst2.step[0]+j)-*((unsigned char*)dst1.data+i*dst1.step[0]+j);
                if(df <0)
                {
                   df = df*(-1);
                }
                if (abdf < df)
                {
                   abdf = df;
                }
                if(df >5)
                {
                //    printf("(%d,%d),",i,j);
                }
            }
        }
        printf("libyuv-opencv max df=%d:\n",abdf);
        Mat rgb,rgbhw;
        cv::cvtColor(dst2,rgb,COLOR_YUV2BGR_IYUV);
        bmcv::hwColorCvt(dst2, rgbhw, FORMAT_YUV420P,FORMAT_BGR_PACKED,  Size(dst2.cols, dst2.rows * 2 / 3), 3);

        FILE* yuv_file = fopen("bgr2yuv420_2bgr.bin", "wb");
        printf("!!write file\n");
        for (int i = 0; i < rgbhw.rows; i++) {
            fwrite(rgbhw.data + i * rgbhw.step[0], 1,
                   rgbhw.cols * rgbhw.channels(), yuv_file);
        }
        fclose(yuv_file);

        // imwrite("rgbhw.jpg", rgbhw);
        abdf = 0;
        for (int i=0;i< rgb.rows;i++)
        {
            for (int j=0;j< rgb.cols*3;j++)
            {
                int df = *((unsigned char*)rgb.data+i*rgb.step[0]+j)-*((unsigned char*)rgbhw.data+i*rgbhw.step[0]+j);
                if(df <0)
                {
                   df = df*(-1);
                }
                if (abdf < df)
                {
                   abdf = df;
                }
                if(df >5)
                {
                //    printf("(%d,%d),",i,j);
                }
            }
        }
        printf("libyuv-opencv max df=%d:\n",abdf);
    }
    else if (strcmp(argv[2],"analysisfuc")==0)
    {
      uchar buffer[4][64]={0};
      for(int j=0;j<4;j++)
      for(int i=0;i<30;i++)
      {
          buffer[j][i]=j+1;
      }
      for(int j=0;j<4;j++)
      for(int i=0;i<30;i++)
      {
          buffer[j][i+32]=j+1;
      }
      printf("origin data: \n");
      for(int j=0;j<4; j++)
      {
          for(int i=0;i<64;i++)
          {
             printf("%d,",buffer[j][i]);
          }
          printf("\n");
      }
      alignWithStride(&buffer[0][0], 64, 60, 4);
      printf("func data: \n");
      for(int j=0;j<4;j++)
      {
          for(int i=0;i<64;i++)
          {
             printf("%d,",buffer[j][i]);
          }
          printf("\n");
      }
      vpp::enableVPPConverter(true);
      cv::enableLibyuvConverter(true);
      return 0;
    }
    else if (strcmp(argv[2],"analysisvppBGR2gray")==0)  //only private test for me
    {
        gettimeofday(&tv0,NULL);
        src = imread(argv[1], -1);
        gettimeofday(&tv1,NULL);
        printf("imread:time_consume(us): %ld\n",(tv1.tv_sec-tv0.tv_sec)*1000000+(tv1.tv_usec-tv0.tv_usec));
        if (!src.data ) {
        cout << "load img err!" << endl;
        return -1;
        }
        gettimeofday(&tv0,NULL);
        cv::cvtColor(src,dst1,COLOR_BGR2GRAY);
        gettimeofday(&tv1,NULL);
        printf("%s:opencv_time_consume(us): %ld\n",argv[2],(tv1.tv_sec-tv0.tv_sec)*1000000+(tv1.tv_usec-tv0.tv_usec));
        Mat dst2;
        gettimeofday(&tv0,NULL);
        bmcv::hwColorCvt(src,  dst2, FORMAT_BGR_PACKED, FORMAT_GRAY, Size(src.cols, src.rows), 1);
        gettimeofday(&tv1,NULL);
        printf("%s:vpp_time_consume(us): %ld\n",argv[2],(tv1.tv_sec-tv0.tv_sec)*1000000+(tv1.tv_usec-tv0.tv_usec));
        int com_width = dst2.cols;
        int abdf = 0;
        for (int i=0;i< dst2.rows-1;i++)
        {
            for (int j=0;j< dst2.cols;j++)
            {
                int df = *((unsigned char*)dst2.data+i*dst2.step[0]+j)-*((unsigned char*)dst1.data+i*dst1.step[0]+j);
                if(df <0)
                {
                   df = df*(-1);
                }
                if (abdf < df)
                {
                   abdf = df;
                }
                if(df >5)
                {
                //    printf("(%d,%d),",i,j);
                }
            }
        }
        printf("libyuv-opencv max  df=%d:com_width=%d\n",abdf,com_width);

        FILE *gray_file = fopen("bgr2gray.bin","wb");
        printf("!!write file\n");
        printf("dst2: width = %d, height = %d, total() = %d, channels = %d\n",
                dst2.size().width, dst2.size().height, dst2.total(),
                dst2.channels());
        fwrite(dst2.data, 1, dst2.total() * dst2.channels(), gray_file);
        fclose(gray_file);

        Mat rgb,rgbhw;
        cv::cvtColor(dst2,rgb,COLOR_GRAY2RGB);
        bmcv::hwColorCvt(dst2, rgbhw, FORMAT_GRAY,FORMAT_RGB_PACKED,  Size(dst2.cols, dst2.rows), 3);
        imwrite("rgbhw.jpg", rgbhw);
        abdf = 0;
        for (int i=0;i< rgb.rows;i++)
        {
            for (int j=0;j< rgb.cols*3;j++)
            {
                int df = *((unsigned char*)rgb.data+i*rgb.step[0]+j)-*((unsigned char*)rgbhw.data+i*rgbhw.step[0]+j);
                if(df <0)
                {
                   df = df*(-1);
                }
                if (abdf < df)
                {
                   abdf = df;
                }
            }
        }
        printf("libyuv-opencv max  df=%d:\n",abdf);

        gray_file = fopen("bgr2gray2rgb.bin","wb");
        printf("!!write file\n");
        printf("rgbhw: width = %d, height = %d, total() = %d, channels = %d\n",
                rgbhw.size().width, rgbhw.size().height, rgbhw.total(),
                rgbhw.channels());
        fwrite(rgbhw.data, 1, rgbhw.total() * rgbhw.channels(), gray_file);
        fclose(gray_file);
    }
    else if (strcmp(argv[2],"drawrectangle")==0)
    {
          Mat frame = imread(argv[1], IMREAD_AVFRAME);
          gettimeofday(&tv0,NULL);
          rectangle(frame,Point(480,272), Point(1440,816), Scalar(255,0,0),4,0);
          gettimeofday(&tv1,NULL);
          printf("YUV rectangle_time_consume(us): %ld\n",(tv1.tv_sec-tv0.tv_sec)*1000000+(tv1.tv_usec-tv0.tv_usec));
          printf("cvtcolor=%d \n",frame.avFormat());
          bmcv::toMAT(frame, dst1, false);
          /*src = imread(argv[1], -1);
          gettimeofday(&tv0,NULL);
          rectangle(src,Point(480,272), Point(1440,816), Scalar(255,0,0),4,0);
          gettimeofday(&tv1,NULL);
          printf("RGB rectangle_time_consume(us): %ld\n",(tv1.tv_sec-tv0.tv_sec)*1000000+(tv1.tv_usec-tv0.tv_usec));
          imwrite("RGB_rectangle.jpg", src);*/
    }
    else if (strcmp(argv[2],"line")==0)
    {
          Mat frame = imread(argv[1], -1);
          gettimeofday(&tv0,NULL);
          line(frame,Point(480,272), Point(1440,816), Scalar(255,0,0),4,0);
          gettimeofday(&tv1,NULL);
          printf("line_time_consume(us): %ld\n",(tv1.tv_sec-tv0.tv_sec)*1000000+(tv1.tv_usec-tv0.tv_usec));
          dst1=frame;

    }
    else if (strcmp(argv[2],"yuvline")==0)
    {
          Mat frame = imread(argv[1], IMREAD_AVFRAME);
          gettimeofday(&tv0,NULL);
          line(frame,Point(1440,272), Point(480,816), Scalar(255,0,0),4,0);
          gettimeofday(&tv1,NULL);
          printf("YUV line_time_consume(us): %ld\n",(tv1.tv_sec-tv0.tv_sec)*1000000+(tv1.tv_usec-tv0.tv_usec));
          printf("cvtcolor=%d \n",frame.avFormat());
          bmcv::toMAT(frame, dst1, false);

    }
    else if (strcmp(argv[2],"yuvVline")==0)
    {
          Mat frame = imread(argv[1], IMREAD_AVFRAME);
          gettimeofday(&tv0,NULL);
          line(frame,Point(480,272), Point(480,816), Scalar(255,0,0),4,0);
          gettimeofday(&tv1,NULL);
          printf("YUV line_time_consume(us): %ld\n",(tv1.tv_sec-tv0.tv_sec)*1000000+(tv1.tv_usec-tv0.tv_usec));
          printf("cvtcolor=%d \n",frame.avFormat());
          bmcv::toMAT(frame, dst1, false);

    }
    else if (strcmp(argv[2],"yuvHline")==0)
    {
          Mat frame = imread(argv[1], IMREAD_AVFRAME);
          gettimeofday(&tv0,NULL);
          line(frame,Point(1440,272), Point(480,272), Scalar(255,0,0),4,0);
          gettimeofday(&tv1,NULL);
          printf("YUV line_time_consume(us): %ld\n",(tv1.tv_sec-tv0.tv_sec)*1000000+(tv1.tv_usec-tv0.tv_usec));
          printf("cvtcolor=%d \n",frame.avFormat());
          bmcv::toMAT(frame, dst1, false);

    }
    else if (strcmp(argv[2],"dot")==0)
    {
          Mat frame = imread(argv[1], IMREAD_AVFRAME);
          gettimeofday(&tv0,NULL);
          circle(frame, Point(480,816),5,Scalar(255,0,0),-1);
          gettimeofday(&tv1,NULL);
          printf("YUV line_time_consume(us): %ld\n",(tv1.tv_sec-tv0.tv_sec)*1000000+(tv1.tv_usec-tv0.tv_usec));
          printf("cvtcolor=%d \n",frame.avFormat());
          bmcv::toMAT(frame, dst1, false);

    }
#endif
    else
    {
        src = imread(argv[1], -1);
        if (!src.data ) {
        cout << "load img err!" << endl;
        return -1;
        }
        gettimeofday(&tv0,NULL);
        cv::cvtColor(src,dst1,actionCode);
        gettimeofday(&tv1,NULL);
        printf("%s:time_consume(us): %ld\n",argv[2],(tv1.tv_sec-tv0.tv_sec)*1000000+(tv1.tv_usec-tv0.tv_usec));
    }
    printf("outfilename:%s \n",argv[4]);
    if (strstr(argv[4],".yuv") != NULL)
    {
        FILE *yuv_file = fopen(argv[4],"wb");
        if (!yuv_file){
            printf("outfilename:%s\n",argv[4]);
            return 0;
        }
        printf("!!write file\n");
        for (int i=0;i<dst1.rows;i++)
        {
            fwrite(dst1.data+i*dst1.step[0],1,dst1.cols*dst1.channels(),yuv_file);
        }
        fclose(yuv_file);
        printf("close file\n");
    }
    else if (strstr(argv[4],".565") != NULL)
    {
        FILE *RGB_file = fopen(argv[4],"wb");
        if (!RGB_file){
            printf("outfilename:%s \n",argv[4]);
            return 0;
        }
        printf("!!write file dst1.rows=%d,dst1.cols=%d\n",dst1.rows,dst1.cols);
        for (int i=0;i<dst1.rows;i++)
        {
               fwrite(dst1.data+i*dst1.step[0],1,dst1.cols*2,RGB_file);

        }
        fclose(RGB_file);
        printf("close file \n");
    }
    else
    {
        char* imageName1 = argv[4];
        if (actionCode == cv::COLOR_RGB2YUV_IYUV)
        {
            Mat dst2;
            printf("dst1.cols=%d,dst1.rows=%d\n",dst1.cols,dst1.rows);
            gettimeofday(&tv0,NULL);
            cv::cvtColor(dst1,dst2,cv::COLOR_YUV2RGB_IYUV);
            gettimeofday(&tv1,NULL);
            printf("COLOR_YUV2RGB_IYUV:time_consume(us): %ld\n",(tv1.tv_sec-tv0.tv_sec)*1000000+(tv1.tv_usec-tv0.tv_usec));
            // imwrite(imageName1, dst2);
        }
        else if (actionCode == cv::COLOR_BGR2YUV_IYUV)
        {
            Mat dst2;
            printf("dst1.cols=%d,dst1.rows=%d\n",dst1.cols,dst1.rows);
            gettimeofday(&tv0,NULL);

            cv::cvtColor(dst1,dst2,cv::COLOR_YUV2BGR_IYUV);
            gettimeofday(&tv1,NULL);
            printf("COLOR_YUV2RGB_IYUV:time_consume(us): %ld\n",(tv1.tv_sec-tv0.tv_sec)*1000000+(tv1.tv_usec-tv0.tv_usec));

            FILE* file_dst2 = fopen(imageName1, "wb");
            if (!file_dst2) {
                std::cerr << "Failed to open output file!" << std::endl;
                return -1;
            }
            printf("Size of dst.bin:\n Width = %d, Height = %d, Channels = %d\n",
                   dst2.size().width, dst2.size().height, dst2.channels());
            // fwrite(dst1.data, sizeof(uchar), (int)dst1.size().width *
            // (int)dst1.size().height * 3 /2, file_dst2);
            fwrite(dst2.data, sizeof(uchar), dst2.total() * dst2.channels(),
                   file_dst2);
            fclose(file_dst2);

            // imwrite(imageName1, dst2);

        } else {
            FILE* file_dst1 = fopen(imageName1, "wb");
            if (!file_dst1) {
                std::cerr << "Failed to open output file!" << std::endl;
                return -1;
            }
            printf(
                "Size of dst.bin:\n Width = %d, Height = %d, Channels "
                "= %d\n",
                dst1.size().width, dst1.size().height, dst1.channels());
            // fwrite(dst1.data, sizeof(uchar), (int)dst1.size().width *
            // (int)dst1.size().height * 3 /2, file_dst1);
            fwrite(dst1.data, sizeof(uchar), dst1.total() * dst1.channels(),
                   file_dst1);
            fclose(file_dst1);
            // imwrite(imageName1, dst1);
        }
    }
    vpp::enableVPPConverter(true);
    cv::enableLibyuvConverter(true);
    return 0;
#else
    printf("only support for arm\n");
#endif
    return 0;
}
