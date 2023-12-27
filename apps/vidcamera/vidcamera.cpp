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
#include <opencv2/videoio/videoio_c.h>
#ifndef WIN32
#include <linux/videodev2.h>
#include <iostream>
#include <stdio.h>
#include <sys/time.h>
#endif
#include <unistd.h>

using namespace cv;
using namespace std;
#define MAX_READ_TIMEOUT 1000*30*50 // 30s

int main(int argc, char* argv[])
{
#ifndef WIN32
    Mat frame;
    //--- INITIALIZE VIDEOCAPTURE
    VideoCapture cap;
    struct timeval tvr1, tvr2, tvw1, tvw2;
    int read_time = 0, write_time = 0;
    int verify_enable = 0;

    if (argc < 4){
        cout << "usage:" << endl;
        cout << "\t" << argv[0] << " camera_id out_name[no suffix] frame_num [width] [height] [fps] [verify]" << endl;
        return -1;
    }

    // open the default camera using default API
    cap.open(strtol(argv[1], NULL, 0));
    if (!cap.isOpened()) {
        cerr << "ERROR! Unable to open camera\n";
        return -1;
    }

    int fourcc[2] = {0};
    fourcc[0] = (int)cap.get(CV_CAP_PROP_FOURCC);
    cout << "default fourcc " << (char *)fourcc << endl;
    cout << "default width  " << (int)cap.get(CV_CAP_PROP_FRAME_WIDTH) << endl;
    cout << "default height " << (int)cap.get(CV_CAP_PROP_FRAME_HEIGHT) << endl;
    cout << "default fps    " << (int)cap.get(CV_CAP_PROP_FPS) << endl;

    cap.set(CV_CAP_PROP_FOURCC, V4L2_PIX_FMT_MJPEG);

    if (argc > 5){
        cap.set(CV_CAP_PROP_FRAME_WIDTH, (double)strtol(argv[4], NULL, 0));
        cap.set(CV_CAP_PROP_FRAME_HEIGHT, (double)strtol(argv[5], NULL, 0));
    }
    if (argc > 6)
        cap.set(CV_CAP_PROP_FPS, (double)strtol(argv[6], NULL, 0));
    if (argc > 7)
        verify_enable = atoi(argv[7]);

    fourcc[0] = (int)cap.get(CV_CAP_PROP_FOURCC);
    cout << "current fourcc " << (char *)fourcc << endl;
    cout << "current width  " << (int)cap.get(CV_CAP_PROP_FRAME_WIDTH) << endl;
    cout << "current height " << (int)cap.get(CV_CAP_PROP_FRAME_HEIGHT) << endl;
    cout << "current fps    " << (int)cap.get(CV_CAP_PROP_FPS) << endl;

    //--- GRAB AND WRITE LOOP
    cout << "Start grabbing" << endl
    << "Press any key to terminate" << endl;
    cap.read(frame);    // drop the first frame because it contains initialization

    int read_times = MAX_READ_TIMEOUT;
    int i_frame_nums = 0;
    while(true)
    {
        if (i_frame_nums >= atoi(argv[3])) {
            break;
        }
        if (read_times <= 0) {
            break;
        }


        // wait for a new frame from camera and store it into 'frame'
        gettimeofday(&tvr1, NULL);
        cap.read(frame);

        // check if we succeeded
        if (frame.empty()) {
            cerr << "ERROR! blank frame grabbed\n";
            if ((int)cap.get(CAP_PROP_STATUS) == 2){     // eof
                cout << "file ends!" << endl;
                cap.release();
                cap.open(argv[1]);
                //cap.set_resampler(5,3);
                cout << "loop again " << endl;
            }

            read_times--;
            usleep(10);
            continue;
        }
        read_times = MAX_READ_TIMEOUT;

        gettimeofday(&tvr2, NULL);
        read_time += (tvr2.tv_sec - tvr1.tv_sec)*1000 + (tvr2.tv_usec - tvr1.tv_usec)/1000;

        // show live and wait for a key with timeout long enough to show images
        gettimeofday(&tvw1, NULL);
        if (verify_enable)
            imwrite(argv[2] + to_string(i_frame_nums) + ".png", frame);

        gettimeofday(&tvw2, NULL);
        write_time += (tvw2.tv_sec - tvw1.tv_sec)*1000 + (tvw2.tv_usec - tvw1.tv_usec)/1000;
        i_frame_nums++;
    }

    cap.release();

    printf("time elascape: total %d frames read %dms write %dms\n", atoi(argv[3]), read_time ,write_time);

    // the camera will be deinitialized automatically in VideoCapture destructor
#endif
    return 0;
}
