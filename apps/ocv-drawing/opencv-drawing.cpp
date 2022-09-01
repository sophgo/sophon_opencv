/**
  @file videocapture_basic.cpp
  @brief A very basic sample for using VideoCapture and VideoWriter
  @author PkLab.net
  @date Aug 24, 2016
*/

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/freetype.hpp>
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <sys/time.h>
#include <stdlib.h>

using namespace cv;
using namespace std;
#define MAX_PARAM_NUMS 32



void line_params_parse(char *params,int &startX, int &startY, int &endX, int &endY, int &R, int &G, int &B, int &thickness);
void putText_params_parse(char *params, char *text, int &posX, int &posY, int &fontFace, double &fontScale, int &R, int &G, int &B, int &thickness);
void circle_params_parse(char *params,int &centerX, int &centerY, int &radius, int &R, int &G, int &B, int &thickness);
void ellipse_params_parse(char *params,int &centerX, int &centerY, int &radiusX, int &radiusY, double &angle, int &R, int &G, int &B, int &thickness);
void polylines_params_parse(char *params, int &posnums, int *posX, int *posY, int &R, int &G, int &B, int &thickness);
void fillPoly_params_parse(char *params, int &posnums, int *posX, int *posY, int &R, int &G, int &B);
void freetype_params_parse(char *params, char *text, char *foutpath, int &posX, int &posY, int &fontHeight, int &R, int &G, int &B, int &thickness);

//解析line参数  rectangle适用
//startX_startY_endX_endY_R_G_B_thickness
void line_params_parse(char *params,int &startX, int &startY, int &endX, int &endY, int &R, int &G, int &B, int &thickness){
    //获取line参数
    char *lineparams = strtok(params, "=");
    lineparams = strtok(NULL, "=");
    //解析line参数
    char *temp = strtok(lineparams, "_");
    startX = atoi(temp);
    temp = strtok(NULL, "_");
    startY = atoi(temp);
    temp = strtok(NULL, "_");
    endX = atoi(temp);
    temp = strtok(NULL, "_");
    endY = atoi(temp);
    temp = strtok(NULL, "_");
    R = atoi(temp);
    temp = strtok(NULL, "_");
    G = atoi(temp);
    temp = strtok(NULL, "_");
    B = atoi(temp);
    temp = strtok(NULL, "_");
    thickness = atoi(temp);
    
    return;
}
//解析putText参数
//textfile_potX_potY_fontFace_fontScale_R_G_B_thickness
void putText_params_parse(char *params, char *text, int &posX, int &posY, int &fontFace, double &fontScale, int &R, int &G, int &B, int &thickness){
    char textfile[128] = {0};
    //获取putText参数
    char *lineparams = strtok(params, "=");
    lineparams = strtok(NULL, "=");
    //解析putText参数
    char *temp = strtok(lineparams, "_");
    strcpy(textfile,temp);
    temp = strtok(NULL, "_");
    posX = atoi(temp);
    temp = strtok(NULL, "_");
    posY = atoi(temp);
    temp = strtok(NULL, "_");
    fontFace = atoi(temp);
    temp = strtok(NULL, "_");
    fontScale = atof(temp);
    temp = strtok(NULL, "_");
    R = atoi(temp);
    temp = strtok(NULL, "_");
    G = atoi(temp);
    temp = strtok(NULL, "_");
    B = atoi(temp);
    temp = strtok(NULL, "_");
    thickness = atoi(temp);

    //通过文件获取text内容
    ifstream read_file;
    read_file.open(textfile, ios::binary);
    
    string line;
    getline(read_file, line);
    strcpy(text,line.c_str());
//    FILE *fp = NULL;
//    fp = fopen(textfile,"r");
//    memset(text,0,1024);
//    fgets(text,1024,fp);
//    fclose(fp);
    return;
}
//解析circle的参数
//centerX_centerY_radius_R_G_B_thickness
void circle_params_parse(char *params,int &centerX, int &centerY, int &radius, int &R, int &G, int &B, int &thickness){
    //获取line参数
    char *lineparams = strtok(params, "=");
    lineparams = strtok(NULL, "=");
    //解析line参数
    char *temp = strtok(lineparams, "_");
    centerX = atoi(temp);
    temp = strtok(NULL, "_");
    centerY = atoi(temp);
    temp = strtok(NULL, "_");
    radius = atoi(temp);
    temp = strtok(NULL, "_");
    R = atoi(temp);
    temp = strtok(NULL, "_");
    G = atoi(temp);
    temp = strtok(NULL, "_");
    B = atoi(temp);
    temp = strtok(NULL, "_");
    thickness = atoi(temp);

    return;
}

//解析ellipse参数
//centerX_centerY_radiusX_radiusY_angle_R_G_B_thickness
void ellipse_params_parse(char *params,int &centerX, int &centerY, int &radiusX, int &radiusY, double &angle, int &R, int &G, int &B, int &thickness){
    //获取line参数
    char *lineparams = strtok(params, "=");
    lineparams = strtok(NULL, "=");
    //解析line参数
    char *temp = strtok(lineparams, "_");
    centerX = atoi(temp);
    temp = strtok(NULL, "_");
    centerY = atoi(temp);
    temp = strtok(NULL, "_");
    radiusX = atoi(temp);
    temp = strtok(NULL, "_");
    radiusY = atoi(temp);
    temp = strtok(NULL, "_");
    angle = atof(temp);
    temp = strtok(NULL, "_");
    R = atoi(temp);
    temp = strtok(NULL, "_");
    G = atoi(temp);
    temp = strtok(NULL, "_");
    B = atoi(temp);
    temp = strtok(NULL, "_");
    thickness = atoi(temp);

    return;
}

//解析polylines参数
//pointNums_p1x_p1y_p2x_p2y_p3x_p3y_p4x_p4y_R_G_B_thickness
void polylines_params_parse(char *params, int &posnums, int *posX, int *posY, int &R, int &G, int &B, int &thickness){
    //获取line参数
    char *lineparams = strtok(params, "=");
    lineparams = strtok(NULL, "=");
    //解析line参数
    char *temp = strtok(lineparams, "_");
    posnums = atoi(temp);
    for (int i =0;i< posnums;i++){
        temp = strtok(NULL, "_");
        posX[i] = atoi(temp);
        temp = strtok(NULL, "_");
        posY[i] = atoi(temp);
    }
    temp = strtok(NULL, "_");
    R = atoi(temp);
    temp = strtok(NULL, "_");
    G = atoi(temp);
    temp = strtok(NULL, "_");
    B = atoi(temp);
    temp = strtok(NULL, "_");
    thickness = atoi(temp);

    return;
}

//解析fillPoly参数
//pointNums_p1x_p1y_p2x_p2y_p3x_p3y_p4x_p4y_R_G_B
void fillPoly_params_parse(char *params, int &posnums, int *posX, int *posY, int &R, int &G, int &B){
    //获取line参数
    char *lineparams = strtok(params, "=");
    lineparams = strtok(NULL, "=");
    //解析line参数
    char *temp = strtok(lineparams, "_");
    posnums = atoi(temp);
    for (int i =0;i< posnums;i++){
        temp = strtok(NULL, "_");
        posX[i] = atoi(temp);
        temp = strtok(NULL, "_");
        posY[i] = atoi(temp);
    }
    temp = strtok(NULL, "_");
    R = atoi(temp);
    temp = strtok(NULL, "_");
    G = atoi(temp);
    temp = strtok(NULL, "_");
    B = atoi(temp);

    return;
}

//解析freetype参数
//freetype=textfile_fontpath_fontHeight_linestyle_R_G_B_thickness
void freetype_params_parse(char *params, char *text, char *foutpath, int &posX, int &posY, int &fontHeight, int &R, int &G, int &B, int &thickness, int &linestyle){
    char textfile[256] = {0};
    //获取line参数
    char *lineparams = strtok(params, "=");
    lineparams = strtok(NULL, "=");
    //解析line参数
    char *temp = strtok(lineparams, "_");
    strcpy(textfile,temp);
    temp = strtok(NULL, "_");
    strcpy(foutpath,temp);
    temp = strtok(NULL, "_");
    posX = atoi(temp);
    temp = strtok(NULL, "_");
    posY = atoi(temp);
    temp = strtok(NULL, "_");
    fontHeight = atoi(temp);
    temp = strtok(NULL, "_");
    R = atoi(temp);
    temp = strtok(NULL, "_");
    G = atoi(temp);
    temp = strtok(NULL, "_");
    B = atoi(temp);
    temp = strtok(NULL, "_");
    thickness = atoi(temp);
    temp = strtok(NULL, "_");
    linestyle = atoi(temp);

    //通过文件获取text内容
    ifstream read_file;
    read_file.open(textfile, ios::binary);
    string line;
    getline(read_file, line);
    strcpy(text,line.c_str());

    return;
}




int main(int argc, char* argv[])
{
    if (argc < 8){
        cout << "usage: ocv_drawing input loops dynamic yuv_enable device_id outfile draw_params ......." << endl;
        cout << "params:" << endl;
        cout << "\t" << "<input>: video file." << endl;
        cout << "\t" << "<loops>: 1 or more." << endl;
        cout << "\t" << "<dynamic>: 0:static drawing   1:dynamic drawing." << endl;
        cout << "\t" << "<yuv_enable>: 0:draw on bgr, 1:draw on yuv." << endl;
        cout << "\t" << "<device_id>: must input. " << endl;
        cout << "\t" << "<outfile>: null:no output file. video: output videofile(out.h264).  other: must jpg ort png file. " << endl;
#ifndef USING_SOC
        cout << "\t" << "[bmcpu_enable]: value is bmcpu or none. " << endl;
#endif
        cout << "\t" << "<draw_params>: line=***:putText=***:rectangle=****:circle=***:ellipse=***:polylines=***:fillPoly=***:freetype=***" << endl;
        cout << "\t" << "\t" << "line: startX_startY_endX_endY_R_G_B_thickness" << endl;
        cout << "\t" << "\t" << "\t" << "eg: line=100_100_200_200_255_0_0_5" << endl;
        cout << "\t" << "\t" << "putText: textfile_potX_potY_fontFace_fontScale_R_G_B_thickness" << endl;
        cout << "\t" << "\t" << "\t" << "eg: putText=text.txt_200_200_3_2.5_0_255_0_6" << endl;
        cout << "\t" << "\t" << "\t" << "textfile: txt file one line content, only support eng." << endl;
        cout << "\t" << "\t" << "\t" << "fontFace: 1:normal size sans-serif  2:small size sans-serif  3 4 5 6 7." << endl;
        cout << "\t" << "\t" << "\t" << "fontScale: 1.0  2.2 ....." << endl;
        cout << "\t" << "\t" << "rectangle:startX_startY_endX_endY_R_G_B_thickness " << endl;
        cout << "\t" << "\t" << "\t" << "eg: rectangle=300_200_500_400_0_0_255_5" << endl;
        cout << "\t" << "\t" << "circle: centerX_centerY_radius_R_G_B_thickness" << endl;
        cout << "\t" << "\t" << "\t" << "eg: circle=400_100_20_0_0_0_5" << endl;
        cout << "\t" << "\t" << "ellipse: centerX_centerY_radiusX_radiusY_angle_R_G_B_thickness" << endl;
        cout << "\t" << "\t" << "\t" << "eg: ellipse=500_400_100_40_160_0_255_255_5" << endl;
        cout << "\t" << "\t" << "polylines: pointNums_p1x_p1y_p2x_p2y_p3x_p3y_p4x_p4y_R_G_B_thickness" << endl;
        cout << "\t" << "\t" << "\t" << "eg: polylines=4_400_100_900_150_900_300_400_300_255_255_255_2" << endl;
        cout << "\t" << "\t" << "fillPoly: pointNums_p1x_p1y_p2x_p2y_p3x_p3y_p4x_p4y_R_G_B" << endl;
        cout << "\t" << "\t" << "\t" << "eg: fillPoly=4_450_150_850_100_850_250_350_250_255_0_0" << endl;
        cout << "\t" << "\t" << "freetype: textfile_fontpath_posX_posY_fontHeight_R_G_B_thickness_linestyle" << endl;
        cout << "\t" << "\t" << "\t" << "eg: freetype=text.txt_wqy-microhei.ttc_100_100_50_0_0_250_-1_8" << endl;
        cout << "\t" << "\t" << "\t" << "thickness: -1:only freetype.  1,2...: use polylines." << endl;
        cout << "\t" << "\t" << "\t" << "linestyle: 4,  8, 16." << endl;
        cout << "eg:" << endl;
#ifdef USING_SOC
        cout << "\t" << "ocv-drawing station.avi  100 1 1 0 out.jpg  line=100_100_200_200_255_0_0_5" << endl;
#else
        cout << "\t" << "bmcpu disable: ocv-drawing station.avi  100 1 1 0 out.jpg  line=100_100_200_200_255_0_0_5" << endl;
        cout << "\t" << "bmcpu enable:  ocv-drawing station.avi  100 1 1 0 out.jpg bmcpu line=100_100_200_200_255_0_0_5" << endl;

#endif
        return -1;
    }

    Mat image;
    VideoCapture cap;
    VideoWriter writer;
    struct timeval tv1, tv2;
    int loops                = 1;
    int dynamic              = 0;
    int yuv_enable           = 0;
    int device_id            = 0;
    char inputfile[256]      = {0};
    char outputfile[256]     = {0};
    char drawparams[4*1024]  = {0};
    int bmcpu_enable = 0;
    strcpy(inputfile,argv[1]);
    loops      = atoi(argv[2]);
    dynamic    = atoi(argv[3]);
    yuv_enable = atoi(argv[4]);
    device_id  = atoi(argv[5]);
    strcpy(outputfile,argv[6]);
#ifndef USING_SOC
    if (strcmp(argv[7], "bmcpu") == 0) {
        if(yuv_enable == 0) {
            cout << "bmcpu not support bgr format." << endl;
            return -1;
        }
        bmcpu_enable = 1;
        strcpy(drawparams,argv[8]);
    }else{
        bmcpu_enable = 0;
        strcpy(drawparams,argv[7]);
    }
#else
    bmcpu_enable = 0;
    strcpy(drawparams,argv[7]);

#endif
    string encodeparms = "";
    FILE *fp_out = NULL;
    if (loops < 1) {
        cout << "loops param err." << endl;
        return -1;
    }

    if ((yuv_enable != 0) && (yuv_enable != 1)) {
        cout << "yuv_enable param err." << endl;
        return -1;
    }

    if (device_id < 0) {
        cout << "device_id param err." << endl;
        return -1;
    }

    char params[MAX_PARAM_NUMS][128] = {0};
    char *p = strtok(drawparams, ":");
    int index = 0;
    while(p){
        strcpy(params[index],p);
        index++;
        p = strtok(NULL, ":");
    }

    // open the default camera using default API
    cap.open(inputfile, CAP_FFMPEG, device_id);
    if (!cap.isOpened()) {
        cerr << "ERROR! Unable to open camera\n";
        return -1;
    }

    // Set Resamper
    cap.set(CAP_PROP_OUTPUT_SRC, 1.0);
    double out_sar = cap.get(CAP_PROP_OUTPUT_SRC);
    cout << "CAP_PROP_OUTPUT_SAR: " << out_sar << endl;

    if(yuv_enable == 1){
        cap.set(cv::CAP_PROP_OUTPUT_YUV, PROP_TRUE);
    }

    cout << "orig CAP_PROP_FRAME_HEIGHT: " << (int) cap.get(CAP_PROP_FRAME_HEIGHT) << endl;
    cout << "orig CAP_PROP_FRAME_WIDTH: " << (int) cap.get(CAP_PROP_FRAME_WIDTH) << endl;

    //--- GRAB AND WRITE LOOP
    gettimeofday(&tv1, NULL);
    for (int i=0; i < loops; i++)
    {
        // wait for a new frame from camera and store it into 'frame'
        cap.read(image);

        // check if we succeeded
        if (image.empty()) {
            cerr << "ERROR! blank frame grabbed\n";
            if ((int)cap.get(CAP_PROP_STATUS) == 2) {     // eof
                cout << "file ends!" << endl;
                cap.release();
                cap.open(argv[1], CAP_FFMPEG, device_id);
                if(yuv_enable == 1){
                    cap.set(cv::CAP_PROP_OUTPUT_YUV, PROP_TRUE);
                }
                cout << "loop again " << endl;
            }
            continue;
        }
        for(int j=0;j<MAX_PARAM_NUMS;j++){
            if(strlen(params[j]) <= 0){
                break;
            }
            char temp_typeing[256] = {0};
            strcpy(temp_typeing,params[j]);
            if(0 == strncmp(temp_typeing,"line=",5)){
                int startX = 0;
                int startY = 0;
                int endX   = 0;
                int endY   = 0;
                int R      = 0;
                int G      = 0;
                int B      = 0;
                int thickness  = 0;

                line_params_parse(temp_typeing, startX, startY, endX, endY, R, G, B, thickness);
                //设置坐标动态变化
                if(dynamic == 1){
                    int startX_tmp = startX;
                    int startY_tmp = startY;
                    startX += i;
                    startY += i;
                    startX = startX % (image.cols+50);
                    startY = startY % (image.rows+50);
                    endX   += (startX - startX_tmp);
                    endY   += (startY - startY_tmp);
                }
                if (bmcpu_enable == 1) {
                    bmcpu_line(image, Point(startX,startY), Point(endX,endY), Scalar(B,G,R), thickness);
                }else{
                    line(image, Point(startX,startY), Point(endX,endY), Scalar(B,G,R), thickness);
                }
            }else if(0 == strncmp(temp_typeing,"putText=",8)){
                //"putText: textfile_potX_potY_fontFace_fontScale_R_G_B_thickness" << endl;
                char text[1024] = {0};
                int posX = 0;
                int posY = 0;
                int fontFace = 0;
                double fontScale = 0;
                int R      = 0;
                int G      = 0;
                int B      = 0;
                int thickness  = 0;
                putText_params_parse(temp_typeing, text, posX, posY, fontFace, fontScale, R, G, B, thickness);
                //设置坐标动态变化
                if(dynamic == 1){
                    posX += i;
                    posY += i;
                    posX = posX % (image.cols+50);
                    posY = posY % (image.rows+50);
                }
                if (bmcpu_enable == 1) {
                    bmcpu_putText(image, text, Point(posX, posY), fontFace, fontScale, Scalar(B,G,R), thickness, 16);
                }else {
                    putText(image, text, Point(posX, posY), fontFace, fontScale, Scalar(B,G,R), thickness, 16);
                }
            }else if(0 == strncmp(temp_typeing,"rectangle=",10)){
                int startX = 0;
                int startY = 0;
                int endX   = 0;
                int endY   = 0;
                int R      = 0;
                int G      = 0;
                int B      = 0;
                int thickness  = 0;
                line_params_parse(temp_typeing, startX, startY, endX, endY, R, G, B, thickness);
                //设置坐标动态变化
                if(dynamic == 1){
                    int startX_tmp = startX;
                    int startY_tmp = startY;
                    startX += i;
                    startY += i;
                    startX = startX % (image.cols+50);
                    startY = startY % (image.rows+50);
                    endX   += (startX - startX_tmp);
                    endY   += (startY - startY_tmp);
                }
                if (bmcpu_enable == 1) {
                    bmcpu_rectangle(image, Point(startX, startY), Point(endX, endY), Scalar(B,G,R),thickness);
                } else {
                    rectangle(image, Point(startX, startY), Point(endX, endY), Scalar(B,G,R),thickness);
		}
            }else if(0 == strncmp(temp_typeing,"circle=",7)){
                int centerX= 0;
                int centerY= 0;
                int radius = 0;
                int R      = 0;
                int G      = 0;
                int B      = 0;
                int thickness  = 0;
                circle_params_parse(temp_typeing, centerX, centerY, radius, R, G, B, thickness);
                //设置坐标动态变化
                if(dynamic == 1){
                    centerX += i;
                    centerY += i;
                    centerX = centerX % (image.cols+50);
                    centerY = centerY % (image.rows+50);
                }
                if (bmcpu_enable == 1) {
                    bmcpu_circle(image, Point(centerX, centerY), radius, Scalar(B, G, R), thickness);
                } else {
                    circle(image, Point(centerX, centerY), radius, Scalar(B, G, R), thickness);
                }
            }else if(0 == strncmp(temp_typeing,"ellipse=",8)){
                int centerX= 0;
                int centerY= 0;
                int radiusX = 0;
                int radiusY = 0;
                double angle = 0;
                int R      = 0;
                int G      = 0;
                int B      = 0;
                int thickness  = 0;

                ellipse_params_parse(temp_typeing, centerX, centerY, radiusX, radiusY, angle, R, G, B, thickness);
                //设置坐标动态变化
                if(dynamic == 1){
                    centerX += i;
                    centerY += i;
                    centerX = centerX % (image.cols+50);
                    centerY = centerY % (image.rows+50);
                }
                if (bmcpu_enable == 1) {
                    bmcpu_ellipse( image, RotatedRect(Point(centerX,centerY),Size(radiusX,radiusY),angle), Scalar(B,G,R), thickness );
                } else {
                    ellipse( image, RotatedRect(Point(centerX,centerY),Size(radiusX,radiusY),angle), Scalar(B,G,R), thickness );
                }
            }else if(0 == strncmp(temp_typeing,"polylines=",10)){
                int posnums = 0;
                int posX[32] = {0};
                int posY[32] = {0};
                int R      = 0;
                int G      = 0;
                int B      = 0;
                int thickness  = 0;
                vector<Point> polyline;
                polylines_params_parse(temp_typeing, posnums, posX, posY, R, G, B, thickness);
                if(dynamic == 1){
                    for (int m =0; m < posnums; m++){
                        posX[m] += i;
                        posY[m] += i;
                        posX[m] = posX[m] % (image.cols+50);
                        posY[m] = posY[m] % (image.rows+50);
                        polyline.push_back(Point(posX[m],posY[m]));
                    }
                } else {
                    for (int m =0; m < posnums; m++){
                        polyline.push_back(Point(posX[m],posY[m]));
                    }
                }
                const Point* pts = &polyline[0];
                if (bmcpu_enable == 1) {
                    bmcpu_polylines( image, &pts, &posnums, 1, true, Scalar(B,G,R), thickness, 8 );
                } else {
                    polylines( image, &pts, &posnums, 1, true, Scalar(B,G,R), thickness, 8 );
                }
            }else if(0 == strncmp(temp_typeing,"fillPoly=",9)){
                int posnums = 0;
                int posX[32] = {0};
                int posY[32] = {0};
                int R      = 0;
                int G      = 0;
                int B      = 0;
                vector<Point> polyline;
                fillPoly_params_parse(temp_typeing, posnums, posX, posY, R, G, B);
                if(dynamic == 1){
                    for (int m =0; m < posnums; m++){
                        posX[m] += i;
                        posY[m] += i;
                        posX[m] = posX[m] % (image.cols+50);
                        posY[m] = posY[m] % (image.rows+50);
                        polyline.push_back(Point(posX[m],posY[m]));
                    }
                } else {
                    for (int m =0; m < posnums; m++){
                        polyline.push_back(Point(posX[m],posY[m]));
                    }
                }
                const Point* pts = &polyline[0];
                if (bmcpu_enable == 1) {
                    bmcpu_fillPoly( image, &pts, &posnums, 1, Scalar(B,G,R) );
                } else {
                    fillPoly( image, &pts, &posnums, 1, Scalar(B,G,R) );
                }
            }else if(0 == strncmp(temp_typeing,"freetype=",9)){
                char text[1024] = {0};
                char foutpath[256] = {0};
                int posX = 0;
                int posY = 0;
                int fontHeight = 0;
                int linestyle = 16;
                int R      = 0;
                int G      = 0;
                int B      = 0;
                int thickness  = 0;
                freetype_params_parse(temp_typeing, text, foutpath, posX, posY, fontHeight, R, G, B, thickness, linestyle);
                if (bmcpu_enable == 1) {
                    freetype::BmcpuFreeType2Impl *ft2 = new freetype::BmcpuFreeType2Impl();
                    ft2->setCardId(0);
                    ft2->loadFontData(foutpath, 0 );
                    //设置坐标动态变化
                    if(dynamic == 1){
                        posX += i;
                        posY += i;
                        posX = posX % (image.cols+50);
                        posY = posY % (image.rows+50);
                    }
                    ft2->putText(image, text, Point(posX,posY), fontHeight, Scalar(B, G, R), thickness, linestyle, true);
                    ft2->unloadFontData();
                    delete ft2;
		} else {
                    Ptr<freetype::FreeType2> ft2;
                    ft2 = freetype::createFreeType2();
                    ft2->loadFontData(foutpath, 0 );
                    //设置坐标动态变化
                    if(dynamic == 1){
                        posX += i;
                        posY += i;
                        posX = posX % (image.cols+50);
                        posY = posY % (image.rows+50);
                    }
                    ft2->putText(image, text, Point(posX,posY), fontHeight, Scalar(B, G, R), thickness, linestyle, true);
                    ft2.release();
                }

            }
        }

        if(strcmp(outputfile,"null") == 0){

        }else if(strcmp(outputfile,"video") == 0){
            if (!writer.isOpened()){
               writer.open("output_ocv_drawing.mp4", VideoWriter::fourcc('H', '2', '6', '4'),
                25,
                Size(image.cols, image.rows),
                encodeparms,
                true,
                device_id);
            }
            writer.write(image);
        }else{
            if ((i == 0) || (dynamic == 1)) {
                char filename[256] = {0};
                sprintf(filename,"dump_%d_%s",i,outputfile);
                imwrite(filename,image);
            }
        }

        if ((i+1) % 300 == 0)
        {
            unsigned int time;
            gettimeofday(&tv2, NULL);
            time = (tv2.tv_sec - tv1.tv_sec)*1000 + (tv2.tv_usec - tv1.tv_usec)/1000;
            printf("current process is %f fps!\n", (300 * 1000.0) / (float)time);
            gettimeofday(&tv1, NULL);
        }
    }

    if (writer.isOpened()){
        writer.release();
    }

    // the camera will be deinitialized automatically in VideoCapture destructor
    return 0;
}
