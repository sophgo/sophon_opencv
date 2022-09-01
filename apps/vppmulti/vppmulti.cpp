// Copyright 2018 Bitmain Inc.
// License
// Author
// g++ -Wall -O2 -g test_opencv.cpp -o test_opencv -L/usr/local/lib -rdynamic -lopencv_imgproc -lopencv_core -lopencv_highgui -lopencv_imgcodecs -lopencv_videoio
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/videoio/videoio_c.h>
#include <string>
#include <vector>
#include <fstream>
#include <iostream>
#include <thread>
#include <random>
#include <pthread.h>
#include <unistd.h>
#include <sys/time.h>
#include <opencv2/imgproc/vpp.hpp>

using std::vector;
using std::string;
using std::cout;
using std::cin;
using std::endl;

using namespace cv;

#define INTERVAL (1)
#define MAX_THREAD_NUM (16)
#define XUN_DEBUG 0
#define ONE_FRAME_CLOCK 30000

std::default_random_engine generator;
std::uniform_int_distribution<int> distribution(0,2*ONE_FRAME_CLOCK);
string filename[MAX_THREAD_NUM];

uint64_t last_count[MAX_THREAD_NUM], count[MAX_THREAD_NUM];
double fps[MAX_THREAD_NUM];
int g_exit_flag = 0;
string g_string_path;

void *stat_pthread(void *arg);
void *vpp_test(void * arg);
int  kbhit(void);

int kbhit (void)
{
    struct timeval tv;
    fd_set rdfs;

    tv.tv_sec = 0;
    tv.tv_usec = 0;

    FD_ZERO(&rdfs);
    FD_SET (STDIN_FILENO, &rdfs);

    select(STDIN_FILENO+1, &rdfs, NULL, NULL, &tv);
    return FD_ISSET(STDIN_FILENO, &rdfs);

}

void * stat_pthread(void *arg) {
  int thread_num = *(int *)arg;

  while(!g_exit_flag) {
    sleep(INTERVAL);
    for (int i = 0; i < thread_num; i++) {
      printf("[%d], [%10lld], [%2.2f] | ", i, (long long)count[i], fps[i]);
    }
    printf("\r");
    fflush(stdout);
  }

  cout << "stat pthread exits!" << endl;

  return NULL;
}


void *vpp_test(void * arg) {
  int id = *(int *)arg;
  struct timeval tv1, tv2, tv0;
  int flg = 0;

  cout << "vpp_test thread " << id << "Started..." << endl;

  gettimeofday(&tv0, NULL);
  tv2 = tv0;

  cv::Mat image;

  if (0 == flg) {
    image = imread(filename[id]);
    cout<<"imread img only once"<<endl;
    flg = 1;
  }


  if(image.empty()) {
    cout << "load image file error!" << endl;
    exit(-1);
  }

  while (!g_exit_flag) {
    int t;

    int h = image.rows;
    int w = image.cols;

  /***********border*************/
    Mat dst_border = vpp::border(image,32,32,32,32);

  /***********crop*************/
    Mat dst_crop[2];
    vector<Rect> loca;
    loca.push_back(Rect(0, 0, w / 4, h / 4));
    loca.push_back(Rect(w / 4, h / 4, w / 4, h / 4));

    vpp::crop(image, loca, dst_crop);
  /***********resize*************/
    Mat dst_resize(h, w, CV_8UC3);
    vpp::resize(image, dst_resize);

  /***********split*************/
    Mat dst_split[3];
    vpp::split(image, dst_split);

    if(!g_string_path.empty())
    {
      std::string string_border = g_string_path + "border" + std::to_string(id) + ".bmp";
      std::string string_crop0  = g_string_path + "crop0_" + std::to_string(id) + ".bmp";
      std::string string_crop1  = g_string_path + "crop1_" + std::to_string(id) + ".bmp";
      std::string string_resize = g_string_path + "resize_" + std::to_string(id) + ".bmp";
      std::string string_split0 = g_string_path + "split0_" + std::to_string(id) + ".bmp";
      std::string string_split1 = g_string_path + "split1_" + std::to_string(id) + ".bmp";
      std::string string_split2 = g_string_path + "split2_" + std::to_string(id) + ".bmp";

      imwrite(string_border, dst_border);
      imwrite(string_crop0, dst_crop[0]);
      imwrite(string_crop1, dst_crop[1]);
      imwrite(string_resize, dst_resize);
      imwrite(string_split0, dst_split[0]);
      imwrite(string_split1, dst_split[1]);
      imwrite(string_split2, dst_split[2]);
    }

    //image.release();

    count[id]++;
    gettimeofday(&tv1, NULL);
    t = (tv1.tv_sec - tv0.tv_sec) * 1000 + (tv1.tv_usec - tv0.tv_usec) / 1000;
    tv0 = tv1;
    t = t;

    if (count[id] % 100 == 0)
      fps[id] = (double)count[id] * 1000.0 / ((tv1.tv_sec - tv2.tv_sec) * 1000.0 + (tv1.tv_usec - tv2.tv_usec) / 1000.0);

    if (fps[id] > 1000.0)
    {
        cout << "illegal fps happens";
        break;
    }
    int sleep_clock = distribution(generator);
    //cout << "sleep clock: " << sleep_clock << endl;
    usleep(sleep_clock);
  }

  cout << " Download thread " << id << " exit!" << endl;

  return NULL;
}

int main(int argc, char **argv)
{
  if (argc < 2) {
    cout << "Usage: " << argv[0] << " thread_num image_file ..." << endl;
    return -1;
  }
  int thread_num = atoi(argv[1]);
  if (argc < 2 + thread_num)
  {
    cout << "invalid: image_file number is less than thread_num " << endl;
    return -1;
  }
  for (int i = 0; i < thread_num; i++){
    filename[i] = argv[2+i];
    cout<<"filename["<<i<<"]"<<filename[i]<<endl;
  }

  if(argc == 3 + thread_num)
     g_string_path = argv[2+thread_num];

  pthread_t vc_thread[thread_num];
  pthread_t stat_h;
  srand((unsigned)time(NULL));
#if 1 //multi-thread
  for (int i = 0; i < thread_num; i++) {
    int ret = pthread_create(&vc_thread[i], NULL, vpp_test, &i);
    if (ret != 0)
    {
      cout << "pthread create failed" << endl;
      return -1;
    }
    sleep(1);
  }
  pthread_create(&stat_h, NULL, stat_pthread, &thread_num);

  char cmd = 0;
  while(cmd != 'q')
  {
      if (kbhit())
          cin >> cmd;
      else
          sleep(1);
  }
  g_exit_flag = 1;

  for (int i = 0; i < thread_num; i++) {
    pthread_join(vc_thread[i], NULL);
  }
  pthread_join(stat_h, NULL);
#else   // single thread
  cv::VideoCapture cap[MAX_THREAD_NUM];
  struct timeval tv1, tv2, tv0;

  for (int i = 0; i < thread_num; i++)
  {
    if (!cap[i].isOpened()) {
      cout << "thread " << i << "start to open cap";
      cap[i].open(filename[i]);
      cap[i].set(CV_CAP_PROP_FRAME_WIDTH, 1920);
      cap[i].set(CV_CAP_PROP_FRAME_HEIGHT, 1080);
      cout << "thread " << i << "open cap done";
    }

    if (!cap[i].isOpened()) {
      cout << "Failed to open camera!" << endl;
      return -1;
    }
  }

  gettimeofday(&tv2, NULL);
  while(1)
  {
    for (int i = 0; i < thread_num; i++)
    {
      int t;
      cout << "process thread " << i << endl;
      cout << "process thread " << i;
      cv::Mat image;
      cap[i] >> image;
      image.release();

      gettimeofday(&tv0, NULL);

      count[i]++;
      gettimeofday(&tv1, NULL);
      t = (tv1.tv_sec - tv0.tv_sec) * 1000 + (tv1.tv_usec - tv0.tv_usec) / 1000;
      cout << count[i] << " frame decoded in " << t << "ms";
      tv0 = tv1;

      if (count[i] % 100 == 0)
        fps[i] = (double)count[i] * 1000.0 / ((tv1.tv_sec - tv2.tv_sec) * 1000.0 + (tv1.tv_usec - tv2.tv_usec) / 1000.0);
    }
  }
#endif
  return 0;
}
