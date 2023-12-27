/*M///////////////////////////////////////////////////////////////////////////////////////
//
//  IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
//
//  By downloading, copying, installing or using the software you agree to this license.
//  If you do not agree to this license, do not download, install,
//  copy or use the software.
//
//
//                          License Agreement
//                For Open Source Computer Vision Library
//
// Copyright (C) 2000-2008, Intel Corporation, all rights reserved.
// Copyright (C) 2009, Willow Garage Inc., all rights reserved.
// Third party copyrights are property of their respective owners.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
//   * Redistribution's of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//   * Redistribution's in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//   * The name of the copyright holders may not be used to endorse or promote products
//     derived from this software without specific prior written permission.
//
// This software is provided by the copyright holders and contributors "as is" and
// any express or implied warranties, including, but not limited to, the implied
// warranties of merchantability and fitness for a particular purpose are disclaimed.
// In no event shall the Intel Corporation or contributors be liable for any direct,
// indirect, incidental, special, exemplary, or consequential damages
// (including, but not limited to, procurement of substitute goods or services;
// loss of use, data, or profits; or business interruption) however caused
// and on any theory of liability, whether in contract, strict liability,
// or tort (including negligence or otherwise) arising in any way out of
// the use of this software, even if advised of the possibility of such damage.
//
//M*/

#include <opencv2/core/utils/configuration.private.hpp>
#include "cap_ffmpeg_legacy_api.hpp"
#include "opencv2/core/utils/logger.hpp"
#include "cap_interface.hpp"

using namespace cv;

#if !(defined(_WIN32) || defined(WINCE))
# include <pthread.h>
#endif
#include <algorithm>
#include <limits>
#include <string.h>
#include <iostream>
#include <string.h>

#ifndef __OPENCV_BUILD
#define CV_FOURCC(c1, c2, c3, c4) (((c1) & 255) + (((c2) & 255) << 8) + (((c3) & 255) << 16) + (((c4) & 255) << 24))
#endif

#define CALC_FFMPEG_VERSION(a,b,c) ( a<<16 | b<<8 | c )

#if defined _MSC_VER && _MSC_VER >= 1200
#pragma warning( disable: 4244 4510 4610 )
#endif

#ifdef __GNUC__
#  pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif
#ifdef _MSC_VER
#pragma warning(disable: 4996)  // was declared deprecated
#endif

#ifndef CV_UNUSED  // Required for standalone compilation mode (OpenCV defines this in base.hpp)
#define CV_UNUSED(name) (void)name
#endif

#ifdef __cplusplus
extern "C" {
#endif

#include "bm_vpuenc_interface.h"
#include "ffmpeg_codecs.hpp"

#include <libavutil/mathematics.h>
#include <libavutil/opt.h>
// https://github.com/FFmpeg/FFmpeg/blame/d79c240196f43b93bd204363f1facc270029f113/doc/APIchanges#L1689-L1695
#if LIBAVUTIL_BUILD >= (LIBAVUTIL_VERSION_MICRO >= 100 \
    ? CALC_FFMPEG_VERSION(52, 85, 100) : CALC_FFMPEG_VERSION(53, 15, 0))
#include <libavutil/display.h>
#endif

#if LIBAVUTIL_BUILD >= (LIBAVUTIL_VERSION_MICRO >= 100 \
    ? CALC_FFMPEG_VERSION(51, 63, 100) : CALC_FFMPEG_VERSION(54, 6, 0))
#include <libavutil/imgutils.h>
#endif

#include <libavcodec/avcodec.h>
#include <libswscale/swscale.h>
#include "bmlib_runtime.h"
#ifdef HAVE_FFMPEG_LIBAVDEVICE
#include <libavdevice/avdevice.h>
#endif

// https://github.com/FFmpeg/FFmpeg/blob/b6af56c034759b81985f8ea094e41cbd5f7fecfb/doc/APIchanges#L602-L605
#if LIBAVFORMAT_BUILD < CALC_FFMPEG_VERSION(58, 9, 100)
#  define CV_FFMPEG_REGISTER
#endif

// https://github.com/FFmpeg/FFmpeg/blob/b6af56c034759b81985f8ea094e41cbd5f7fecfb/doc/APIchanges#L654-L657
#if LIBAVCODEC_BUILD < CALC_FFMPEG_VERSION(58, 9, 100)
#  define CV_FFMPEG_LOCKMGR
#endif

// https://github.com/FFmpeg/FFmpeg/blob/b6af56c034759b81985f8ea094e41cbd5f7fecfb/doc/APIchanges#L390-L392
#if LIBAVCODEC_BUILD >= CALC_FFMPEG_VERSION(58, 87, 100)
#include <libavcodec/bsf.h>
#endif

#include <libavutil/pixdesc.h>

// https://github.com/FFmpeg/FFmpeg/blob/b6af56c034759b81985f8ea094e41cbd5f7fecfb/doc/APIchanges#L208-L210
#if LIBAVFORMAT_BUILD >= CALC_FFMPEG_VERSION(59, 0, 100)
#  define CV_FFMPEG_FMT_CONST const
#else
#  define CV_FFMPEG_FMT_CONST
#endif

// https://github.com/FFmpeg/FFmpeg/blob/b6af56c034759b81985f8ea094e41cbd5f7fecfb/doc/APIchanges#L623-L624
#if LIBAVFORMAT_BUILD >= CALC_FFMPEG_VERSION(58, 7, 100)
#  define CV_FFMPEG_URL
#endif

// AVStream.codec deprecated in favor of AVStream.codecpar
// https://github.com/FFmpeg/FFmpeg/blob/b6af56c034759b81985f8ea094e41cbd5f7fecfb/doc/APIchanges#L1039-L1040
#if LIBAVFORMAT_BUILD >= CALC_FFMPEG_VERSION(59, 16, 100)
//#if LIBAVFORMAT_BUILD >= CALC_FFMPEG_VERSION(57, 33, 100)
#  define CV_FFMPEG_CODECPAR
#  define CV_FFMPEG_CODEC_FIELD codecpar
#else
#  define CV_FFMPEG_CODEC_FIELD codec
#endif

#if LIBAVFORMAT_BUILD >= CALC_FFMPEG_VERSION(59, 16, 100)
#  define CV_FFMPEG_PTS_FIELD pts
#else
#  define CV_FFMPEG_PTS_FIELD pkt_pts
#endif

// https://github.com/FFmpeg/FFmpeg/blob/b6af56c034759b81985f8ea094e41cbd5f7fecfb/doc/APIchanges#L1757-L1758
#if LIBAVUTIL_BUILD < CALC_FFMPEG_VERSION(52, 63, 100)
inline static AVRational av_make_q(int num, int den)
{
    AVRational res;
    res.num = num;
    res.den = den;
    return res;
}
#endif



#ifdef __cplusplus
}
#endif

// GCC 4.x compilation bug. Details: https://github.com/opencv/opencv/issues/20292
#if (defined(__GNUC__) && __GNUC__ < 5) && !defined(__clang__)
#undef USE_AV_HW_CODECS
#define USE_AV_HW_CODECS 0
#endif

//#define USE_AV_HW_CODECS 0
#ifndef USE_AV_HW_CODECS
#if LIBAVUTIL_VERSION_MAJOR >= 56 // FFMPEG 4.0+
#define USE_AV_HW_CODECS 1
#include "cap_ffmpeg_hw.hpp"
#else
#define USE_AV_HW_CODECS 0
#endif
#endif

#if defined _MSC_VER && _MSC_VER >= 1200
#pragma warning( default: 4244 4510 4610 )
#endif

#ifdef NDEBUG
#define CV_WARN(message)
#else
#define CV_WARN(message) fprintf(stderr, "warning: %s (%s:%d)\n", message, __FILE__, __LINE__)
#endif

#if defined _WIN32
    #include <windows.h>
    #if defined _MSC_VER && _MSC_VER < 1900
    struct timespec
    {
        time_t tv_sec;
        long   tv_nsec;
    };
  #endif
#elif defined __linux__ || defined __APPLE__
    #include <unistd.h>
    #include <stdio.h>
    #include <sys/types.h>
    #include <sys/time.h>
#if defined __APPLE__
    #include <sys/sysctl.h>
    #include <mach/clock.h>
    #include <mach/mach.h>
#endif
#endif

#include "opencv2/imgproc/vpp.hpp"


#if defined(__APPLE__)
#define AV_NOPTS_VALUE_ ((int64_t)0x8000000000000000LL)
#else
#define AV_NOPTS_VALUE_ ((int64_t)AV_NOPTS_VALUE)
#endif

#ifndef AVERROR_EOF
#define AVERROR_EOF (-MKTAG( 'E','O','F',' '))
#endif

#if LIBAVCODEC_BUILD >= CALC_FFMPEG_VERSION(54,25,0)
#  define CV_CODEC_ID AVCodecID
#  define CV_CODEC(name) AV_##name
#else
#  define CV_CODEC_ID CodecID
#  define CV_CODEC(name) name
#endif

#ifndef PKT_FLAG_KEY
#define PKT_FLAG_KEY AV_PKT_FLAG_KEY
#endif

#if LIBAVUTIL_BUILD >= (LIBAVUTIL_VERSION_MICRO >= 100 \
    ? CALC_FFMPEG_VERSION(52, 38, 100) : CALC_FFMPEG_VERSION(52, 13, 0))
#define USE_AV_FRAME_GET_BUFFER 1
#else
#define USE_AV_FRAME_GET_BUFFER 0
#ifndef AV_NUM_DATA_POINTERS // required for 0.7.x/0.8.x ffmpeg releases
#define AV_NUM_DATA_POINTERS 4
#endif
#endif

#define SUPPORT_INPUT_THREADS 1

#ifndef USE_AV_INTERRUPT_CALLBACK
#if SUPPORT_INPUT_THREADS
#define USE_AV_INTERRUPT_CALLBACK 1 // fix deadlock for avformat_open
#else
#define USE_AV_INTERRUPT_CALLBACK 0
#endif
#endif

#ifndef USE_AV_SEND_FRAME_API
// https://github.com/FFmpeg/FFmpeg/commit/7fc329e2dd6226dfecaa4a1d7adf353bf2773726
#if LIBAVCODEC_VERSION_MICRO >= 100 \
    && LIBAVCODEC_BUILD >= CALC_FFMPEG_VERSION(57, 37, 100)
#define USE_AV_SEND_FRAME_API 1
#else
#define USE_AV_SEND_FRAME_API 0
#endif
#endif

#ifdef USING_SOC
#define  DMA_VPU_BASE_BEGIN  0X100000000
#define  DMA_VPU_BASE_END  0X200000000
#else
#define  DMA_VPU_BASE_BEGIN  0X400000000
#define  DMA_VPU_BASE_END  0X500000000
#endif

#define DMA_LIST_MAX_NUMS 8
#define HEAP2_MASK 0x2
#define MALLOC_HEAP_ID 0x6

#if USE_AV_INTERRUPT_CALLBACK
#define LIBAVFORMAT_INTERRUPT_OPEN_DEFAULT_TIMEOUT_MS 30000
#define LIBAVFORMAT_INTERRUPT_READ_DEFAULT_TIMEOUT_MS 30000

#ifdef _WIN32
// http://stackoverflow.com/questions/5404277/porting-clock-gettime-to-windows

static
inline LARGE_INTEGER get_filetime_offset()
{
    SYSTEMTIME s;
    FILETIME f;
    LARGE_INTEGER t;

    s.wYear = 1970;
    s.wMonth = 1;
    s.wDay = 1;
    s.wHour = 0;
    s.wMinute = 0;
    s.wSecond = 0;
    s.wMilliseconds = 0;
    SystemTimeToFileTime(&s, &f);
    t.QuadPart = f.dwHighDateTime;
    t.QuadPart <<= 32;
    t.QuadPart |= f.dwLowDateTime;
    return t;
}

static
inline void get_monotonic_time(timespec *tv)
{
    LARGE_INTEGER           t;
    FILETIME				f;
    double                  microseconds;
    static LARGE_INTEGER    offset;
    static double           frequencyToMicroseconds;
    static int              initialized = 0;
    static BOOL             usePerformanceCounter = 0;

    if (!initialized)
    {
        LARGE_INTEGER performanceFrequency;
        initialized = 1;
        usePerformanceCounter = QueryPerformanceFrequency(&performanceFrequency);
        if (usePerformanceCounter)
        {
            QueryPerformanceCounter(&offset);
            frequencyToMicroseconds = (double)performanceFrequency.QuadPart / 1000000.;
        }
        else
        {
            offset = get_filetime_offset();
            frequencyToMicroseconds = 10.;
        }
    }

    if (usePerformanceCounter)
    {
        QueryPerformanceCounter(&t);
    } else {
        GetSystemTimeAsFileTime(&f);
        t.QuadPart = f.dwHighDateTime;
        t.QuadPart <<= 32;
        t.QuadPart |= f.dwLowDateTime;
    }

    t.QuadPart -= offset.QuadPart;
    microseconds = (double)t.QuadPart / frequencyToMicroseconds;
    t.QuadPart = (LONGLONG)microseconds;
    tv->tv_sec = t.QuadPart / 1000000;
    tv->tv_nsec = (t.QuadPart % 1000000) * 1000;
}
#else
static
inline void get_monotonic_time(timespec *time)
{
#if defined(__APPLE__) && defined(__MACH__)
    clock_serv_t cclock;
    mach_timespec_t mts;
    host_get_clock_service(mach_host_self(), CALENDAR_CLOCK, &cclock);
    clock_get_time(cclock, &mts);
    mach_port_deallocate(mach_task_self(), cclock);
    time->tv_sec = mts.tv_sec;
    time->tv_nsec = mts.tv_nsec;
#else
    clock_gettime(CLOCK_MONOTONIC, time);
#endif
}
#endif

static
inline timespec get_monotonic_time_diff(timespec start, timespec end)
{
    timespec temp;
    if (end.tv_nsec - start.tv_nsec < 0)
    {
        temp.tv_sec = end.tv_sec - start.tv_sec - 1;
        temp.tv_nsec = 1000000000 + end.tv_nsec - start.tv_nsec;
    }
    else
    {
        temp.tv_sec = end.tv_sec - start.tv_sec;
        temp.tv_nsec = end.tv_nsec - start.tv_nsec;
    }
    return temp;
}

static
inline double get_monotonic_time_diff_ms(timespec time1, timespec time2)
{
    timespec delta = get_monotonic_time_diff(time1, time2);
    double milliseconds = delta.tv_sec * 1000 + (double)delta.tv_nsec / 1000000.0;

    return milliseconds;
}
#endif // USE_AV_INTERRUPT_CALLBACK


struct Image_FFMPEG
{
    unsigned char* data;
    int step;
    int width;
    int height;
    int cn;
    int display_width;
    int display_height;
    AVPixelFormat format;
};

// #ifndef AVSEEK_FLAG_FRAME
// #define AVSEEK_FLAG_FRAME 0
// #endif
// #ifndef AVSEEK_FLAG_ANY
// #define AVSEEK_FLAG_ANY 1
// #endif

static inline void _opencv_ffmpeg_av_packet_unref(AVPacket *pkt);
/* BM codec callback function and definitions */
#define MAX_VIDEO_CHL_NUM   1024

struct _Debug_Param{
    int         debug_level;

    FILE      * debug_fp[2];
};

struct _Bmvid_Callback_Info{
    _Debug_Param    debug_param;

    int             chl_id;
    int             used;
};

static _Bmvid_Callback_Info g_bm_callback_info[MAX_VIDEO_CHL_NUM];
static void bm_find_decoder_name(int dec_id, std::string &dec_name);
static void bm_find_encoder_name(int enc_id, std::string &enc_name);
static void callback_init(_Bmvid_Callback_Info *info);
static void dump_write(_Debug_Param *param, AVFrame *frame);
static void dump_init(_Debug_Param *param, int id);
static void dump_close(_Debug_Param *param);


#if SUPPORT_INPUT_THREADS
#define MAX_PKT_BUF_ITEMS   60      // 2 seconds
struct _input_fd_content_
{
    class av_inputs *pThis;
    AVFormatContext *ic;
};

class av_inputs{

public:
    av_inputs() { init(); }
    ~av_inputs() { destroy(); }

    void init(void);
    int start_input_thread(AVFormatContext *ic);
    void destroy(void);
    int pkt_list_packet_send(AVPacket *packet);
    int pkt_list_packet_recv(AVPacket *packet);

    int input_exit;

    void pkt_list_set_err_send(int err);
    void pkt_list_set_err_recv(int err);
    inline int pkt_list_get_err_send(void){ return err_send;};
    inline int pkt_list_get_err_recv(void){ return err_recv;};

    int pkt_list_space(void);
    int pkt_list_size(void);
    void pkt_list_clean(void);
    void thread_resume(void);
    void thread_pause_and_clean(void);

private:
    struct _avpkt_list_
    {
        uint8_t buffer[sizeof(AVPacket) * MAX_PKT_BUF_ITEMS];
        uint8_t *rptr, *wptr, *end;
        uint32_t rndx, wndx;
        int      count;
    }avpkt_list;

    _input_fd_content_ input_fd_content;

#ifdef WIN32
    HANDLE lock;
    HANDLE input_thread;
    CRITICAL_SECTION cs;
    CONDITION_VARIABLE cond_recv;
    CONDITION_VARIABLE cond_send;
#else
    pthread_mutex_t lock;
    pthread_t      input_thread;
    pthread_cond_t cond_recv;
    pthread_cond_t cond_send;
#endif
    int err_send;
    int err_recv;
    int elsize;
    int thread_status; //1 init or resume or running ; 2 pause;

#ifdef WIN32
    static DWORD WINAPI internal_input_thread(void * arg);
#else
    static void * internal_input_thread(void *arg);
#endif
};

void av_inputs::init(void)
{
    elsize = sizeof(AVPacket);
    avpkt_list.wptr =
    avpkt_list.rptr = avpkt_list.buffer;
    avpkt_list.rndx =
    avpkt_list.wndx = 0;
    avpkt_list.count = 0;
    avpkt_list.end = avpkt_list.buffer + elsize * MAX_PKT_BUF_ITEMS;

    input_exit = 0;
    input_thread = 0;
    err_send =
    err_recv = 0;
    input_thread = 0;
    thread_status = 1;

#ifdef WIN32

    InitializeCriticalSection(&(cs));
    InitializeConditionVariable(&(cond_send));
    InitializeConditionVariable(&(cond_recv));

    lock = CreateMutex(NULL, FALSE, NULL);
#else
    pthread_cond_init(&cond_recv, NULL);
    pthread_cond_init(&cond_send, NULL);
    pthread_mutex_init(&lock, NULL);
#endif

    return;
}

void av_inputs::destroy(void)
{
    AVPacket packet;

    if (input_thread)
    {
        thread_status = 1;
        input_exit = 1;
        pkt_list_set_err_send(AVERROR_EOF); // wake up all condition wait event
        while (pkt_list_packet_recv(&packet) >= 0)
            _opencv_ffmpeg_av_packet_unref(&packet); // give up all buffered packet

#ifdef WIN32
        WaitForSingleObject(input_thread, INFINITE);
#else
        pthread_join(input_thread, NULL);
        input_thread = 0;
#endif
    }

#ifdef WIN32

    DeleteCriticalSection(&(cs));
    CloseHandle(lock);
#else
    pthread_cond_destroy(&cond_recv);
    pthread_cond_destroy(&cond_send);
    pthread_mutex_destroy(&lock);
#endif

    memset(&avpkt_list, 0, sizeof(_avpkt_list_));

    return;
}

int av_inputs::start_input_thread(AVFormatContext *ic)
{
    int ret = 0;

    if (ic == NULL)
        return -1;

    input_fd_content.ic = ic;
    input_fd_content.pThis = this;

#ifdef WIN32
    DWORD thread_id = 0;
    input_thread = CreateThread(NULL, 0, internal_input_thread, (void *)&input_fd_content, 0, &thread_id);
    if (!input_thread)
    {
        ret = -1;
        std::cout << "pthread_create failed: " << std::hex << ret << ". Try to increase `ulimit -v` or decrease `ulimit -s`."
            << std::dec << std::endl;
    }
#else
    ret = pthread_create(&input_thread, NULL, internal_input_thread, &input_fd_content);
    if (ret != 0)
        std::cout << "pthread_create failed: "<< std::hex << ret << ". Try to increase `ulimit -v` or decrease `ulimit -s`."
                  << std::dec << std::endl;
    else{
        char thread_name[15];
        sprintf(thread_name, "vin_%08x", (unsigned int)input_thread);
        pthread_setname_np(input_thread, thread_name);
    }
#endif

    return ret;
}

void av_inputs::pkt_list_set_err_send(int err)
{
#ifdef WIN32
    EnterCriticalSection(&(cs));
    err_send = err;
    LeaveCriticalSection(&(cs));
    WakeAllConditionVariable(&cond_send);
#else
    pthread_mutex_lock(&lock);
    err_send = err;
    pthread_mutex_unlock(&lock);
    pthread_cond_broadcast(&cond_send);
#endif
}


void av_inputs::pkt_list_set_err_recv(int err)
{
#ifdef WIN32
    EnterCriticalSection(&(cs));
    err_recv = err;
    LeaveCriticalSection(&(cs));
    WakeAllConditionVariable(&cond_recv);
#else
    pthread_mutex_lock(&lock);
    err_recv = err;
    pthread_mutex_unlock(&lock);
    pthread_cond_broadcast(&cond_recv);
#endif
}

int av_inputs::pkt_list_space(void)
{
    int size, used;

    size = avpkt_list.end - avpkt_list.buffer;
    used = avpkt_list.wndx - avpkt_list.rndx;

    return (size - used);
}

int av_inputs::pkt_list_size(void)
{
    return (int)(avpkt_list.wndx - avpkt_list.rndx);
}

void av_inputs::pkt_list_clean(void)
{
    AVPacket *packet;
#ifdef WIN32
    WaitForSingleObject(lock, INFINITE);
#else
    pthread_mutex_lock(&lock);
#endif
    while(pkt_list_size() >= elsize) {
        packet = (AVPacket*)avpkt_list.rptr;
        avpkt_list.rptr += elsize;
        if (avpkt_list.rptr == avpkt_list.end)
            avpkt_list.rptr = avpkt_list.buffer;

        avpkt_list.rndx += elsize;

        _opencv_ffmpeg_av_packet_unref(packet);
    }
#ifdef WIN32
    ReleaseMutex(lock);
#else
    pthread_mutex_unlock(&lock);
#endif
    return;
}

void av_inputs::thread_resume(void)
{
    thread_status = 1;
#ifdef WIN32
    WakeAllConditionVariable(&cond_send);
#else
    pthread_cond_broadcast(&cond_send);
#endif
    return;
}

void av_inputs::thread_pause_and_clean(void)
{
    thread_status = 2;
    pkt_list_clean();
#ifdef WIN32
    WakeAllConditionVariable(&cond_send);
    Sleep(1); //msec
#else
    pthread_cond_broadcast(&cond_send);
    usleep(1000);
#endif
    return;
}

int av_inputs::pkt_list_packet_send(AVPacket *packet)
{
    int ret = 0;
#ifdef WIN32
    EnterCriticalSection(&(cs));
#else
    pthread_mutex_lock(&lock);
#endif

    while(!err_send && pkt_list_space() < elsize && packet->size > 0)
    {
#ifdef WIN32
        SleepConditionVariableCS(&(cond_send), &(cs), INFINITE);
#else
        pthread_cond_wait(&cond_send, &lock);
#endif
    }
    if (err_send){
#ifdef WIN32
        LeaveCriticalSection(&(cs));
#else
        pthread_mutex_unlock(&lock);
#endif
        return err_send;
    }

    if (packet->size > 0){
        memcpy(avpkt_list.wptr, packet, elsize);
        avpkt_list.wptr += elsize;
        if(avpkt_list.wptr == avpkt_list.end)
            avpkt_list.wptr = avpkt_list.buffer;

        avpkt_list.wndx += elsize;
        ret = elsize;
    } else
        ret = 0;

#ifdef WIN32
    LeaveCriticalSection(&(cs));
    WakeConditionVariable(&cond_recv);
#else
    pthread_mutex_unlock(&lock);
    pthread_cond_signal(&cond_recv);
#endif

    return ret;
}

int av_inputs::pkt_list_packet_recv(AVPacket *packet)
{
    int ret = 0;
#ifdef WIN32
    EnterCriticalSection(&(cs));
#else
    pthread_mutex_lock(&lock);
#endif

    while(!err_recv && pkt_list_size() < elsize)
    {
#ifdef WIN32
        SleepConditionVariableCS(&(cond_recv), &(cs), INFINITE);
#else
        pthread_cond_wait(&cond_recv, &lock);
#endif
    }
    if (err_recv && pkt_list_size() < elsize)
    {
#ifdef WIN32
        LeaveCriticalSection(&(cs));
#else
        pthread_mutex_unlock(&lock);
#endif
        return err_recv;
    }

    memcpy(packet, avpkt_list.rptr, elsize);
    avpkt_list.rptr += elsize;
    if (avpkt_list.rptr == avpkt_list.end)
        avpkt_list.rptr = avpkt_list.buffer;

    avpkt_list.rndx += elsize;
    ret = elsize;

#ifdef WIN32
    LeaveCriticalSection(&(cs));
    WakeConditionVariable(&cond_send);
#else
    pthread_mutex_unlock(&lock);
    pthread_cond_signal(&cond_send);
#endif

    return ret;
}

#ifdef WIN32
DWORD av_inputs::internal_input_thread(void * arg)
#else
void * av_inputs::internal_input_thread(void *arg)
#endif
{
    _input_fd_content_ *fd = (_input_fd_content_ *)arg;
    AVFormatContext *ic = fd->ic;
    av_inputs *t = fd->pThis;
    int ret = 0;
    int prev_rec_err = 0;

    if (ic == NULL) return NULL;
    if (t == NULL) return NULL;

    while(1 /*!t->input_exit*/)
    {
        if(t->thread_status == 2){
#ifdef WIN32
            Sleep(1);
#else
            usleep(1000);
#endif
            continue;
        }
        AVPacket packet;
        ret = av_read_frame(ic, &packet);
        // printf("<<<<<<<<<< av_inputs::internal_input_thread av_read_frame.\n");
        if (ret == AVERROR(EAGAIN)) {
#ifdef WIN32
            Sleep(10);
#else
            usleep(10000);
#endif
            continue;
        }
        if (ret < 0){
            _opencv_ffmpeg_av_packet_unref (&packet);
            t->pkt_list_set_err_recv(ret);
#ifdef WIN32
            Sleep(10);
#else
            usleep(10000);
#endif
            if (ret == AVERROR(ETIMEDOUT)) {
                prev_rec_err = ret;
                std::cout << "warning: recv error, ret " << std::hex << ret << std::dec << std::endl;
            } else {
                if (ret != AVERROR_EOF)
                    std::cout << "exit: recv error, ret " << std::hex << ret << std::dec << std::endl;
                break;      //exit thread
            }
        }

        if (prev_rec_err && ret >= 0){
            t->pkt_list_set_err_recv(0);    //clear rec_err flag when receive recovers.
            prev_rec_err = 0;
        }
        ret = t->pkt_list_packet_send(&packet);
        if (ret < 0)
        {
            if (ret != AVERROR_EOF)
                std::cout << "exit: packet list overflow, ret " << std::hex << ret << std::dec << std::endl;
            else
                std::cout << "exit: stream EOF" << std::endl;
            _opencv_ffmpeg_av_packet_unref(&packet);
            t->pkt_list_set_err_recv(ret);
            break;      // exit thread
        }
    }
    return NULL;
}
#endif


#if USE_AV_INTERRUPT_CALLBACK
struct AVInterruptCallbackMetadata
{
    timespec value;
    unsigned int timeout_after_ms;
    int timeout;
#ifdef WIN32
    HANDLE interrupt_lock;
#else
    pthread_mutex_t interrupt_lock;
#endif
};

// https://github.com/opencv/opencv/pull/12693#issuecomment-426236731
static
inline const char* _opencv_avcodec_get_name(CV_CODEC_ID id)
{
#if LIBAVCODEC_VERSION_MICRO >= 100 \
    && LIBAVCODEC_BUILD >= CALC_FFMPEG_VERSION(53, 47, 100)
    return avcodec_get_name(id);
#else
    const AVCodecDescriptor *cd;
    AVCodec *codec;

    if (id == AV_CODEC_ID_NONE)
    {
        return "none";
    }
    cd = avcodec_descriptor_get(id);
    if (cd)
    {
        return cd->name;
    }
    codec = avcodec_find_decoder(id);
    if (codec)
    {
        return codec->name;
    }
    codec = avcodec_find_encoder(id);
    if (codec)
    {
        return codec->name;
    }

    return "unknown_codec";
#endif
}


static
inline int _opencv_ffmpeg_interrupt_callback(void *ptr)
{
    AVInterruptCallbackMetadata* metadata = (AVInterruptCallbackMetadata*)ptr;
    CV_Assert(metadata);
#ifdef WIN32
    WaitForSingleObject(metadata->interrupt_lock, INFINITE);
#else
    pthread_mutex_lock(&metadata->interrupt_lock);
#endif

    if (metadata->timeout_after_ms == 0)
    {
#ifdef WIN32
        ReleaseMutex(metadata->interrupt_lock);
#else
        pthread_mutex_unlock(&metadata->interrupt_lock);
#endif
        return 0; // timeout is disabled
    }

    timespec now;
    get_monotonic_time(&now);

    metadata->timeout = get_monotonic_time_diff_ms(metadata->value, now) > metadata->timeout_after_ms;
#ifdef WIN32
    ReleaseMutex(metadata->interrupt_lock);
#else
    pthread_mutex_unlock(&metadata->interrupt_lock);
#endif

    return metadata->timeout ? -1 : 0;
}
#endif

static
inline void _opencv_ffmpeg_av_packet_unref(AVPacket *pkt)
{
#if LIBAVCODEC_BUILD >= (LIBAVCODEC_VERSION_MICRO >= 100 \
    ? CALC_FFMPEG_VERSION(55, 25, 100) : CALC_FFMPEG_VERSION(55, 16, 0))
    av_packet_unref(pkt);
#else
    av_free_packet(pkt);
#endif
};

static
inline void _opencv_ffmpeg_av_image_fill_arrays(void *frame, uint8_t *ptr, enum AVPixelFormat pix_fmt, int width, int height)
{
#if LIBAVUTIL_BUILD >= (LIBAVUTIL_VERSION_MICRO >= 100 \
    ? CALC_FFMPEG_VERSION(51, 63, 100) : CALC_FFMPEG_VERSION(54, 6, 0))
    av_image_fill_arrays(((AVFrame*)frame)->data, ((AVFrame*)frame)->linesize, ptr, pix_fmt, width, height, 1);
#else
    avpicture_fill((AVPicture*)frame, ptr, pix_fmt, width, height);
#endif
};

static
inline int _opencv_ffmpeg_av_image_get_buffer_size(enum AVPixelFormat pix_fmt, int width, int height)
{
#if LIBAVUTIL_BUILD >= (LIBAVUTIL_VERSION_MICRO >= 100 \
    ? CALC_FFMPEG_VERSION(51, 63, 100) : CALC_FFMPEG_VERSION(54, 6, 0))
    return av_image_get_buffer_size(pix_fmt, width, height, 1);
#else
    return avpicture_get_size(pix_fmt, width, height);
#endif
};

// static AVRational _opencv_ffmpeg_get_sample_aspect_ratio(AVStream *stream)
// {
// #if LIBAVUTIL_VERSION_MICRO >= 100 && LIBAVUTIL_BUILD >= CALC_FFMPEG_VERSION(54, 5, 100)
//     return av_guess_sample_aspect_ratio(NULL, stream, NULL);
// #else
//     AVRational undef = {0, 1};
//
//     // stream
//     AVRational ratio = stream ? stream->sample_aspect_ratio : undef;
//     av_reduce(&ratio.num, &ratio.den, ratio.num, ratio.den, INT_MAX);
//     if (ratio.num > 0 && ratio.den > 0)
//         return ratio;
//
//     // codec
//     ratio  = stream && stream->codec ? stream->codec->sample_aspect_ratio : undef;
//     av_reduce(&ratio.num, &ratio.den, ratio.num, ratio.den, INT_MAX);
//     if (ratio.num > 0 && ratio.den > 0)
//         return ratio;
//
//     return undef;
// #endif
// }
//
inline static std::string _opencv_ffmpeg_get_error_string(int error_code)
{
    char buf[255] = {0};
    const int err = av_strerror(error_code, buf, 254);
    if (err == 0)
        return std::string(buf);
    else
        return std::string("Unknown error");
}

struct CvCapture_FFMPEG
{
    bool open(const char* filename, const VideoCaptureParameters& params, int id);
    void close();

    double getProperty(int) const;
    bool setProperty(int, double);
    bool grabFrame(char *buf=NULL, unsigned int len_in=0, unsigned int *len_out=NULL);
    bool retrieveFrame(int, cv::OutputArray image);
    bool retrieveFrameSoft(int, cv::OutputArray image);
    bool retrieveHWFrame(cv::OutputArray output);
    void rotateFrame(cv::Mat &mat) const;

    void init();

    void    seek(int64_t frame_number);
    void    seek(double sec);
    bool    slowSeek( int framenumber );

    int64_t get_total_frames() const;
    double  get_duration_sec() const;
    double  get_fps() const;
    int64_t get_bitrate() const;
    AVRational get_sample_aspect_ratio(AVStream *stream) const;

    double  r2d(AVRational r) const;
    int64_t dts_to_frame_number(int64_t dts);
    double  dts_to_sec(int64_t dts) const;
    void    get_rotation_angle();

    /* bm vpu private function */
    bool    is_bm_video_codec(int codec_id);
    bool    is_bm_image_codec(int codec_id);
    bool    set_resampler(int den, int num);
    bool    get_resampler(int *den, int *num);

    AVFormatContext * ic;
    AVCodec         * avcodec;
    AVCodecContext  * context;
    int               video_stream;
    AVStream        * video_st;
    AVFrame         * picture;
    AVFrame           rgb_picture;
    int64_t           picture_pts;
    int64_t           picture_raw_pts;

    AVPacket          packet;
    Image_FFMPEG      frame;
    struct SwsContext *img_convert_ctx;

    int               picture_width;
    int               picture_height;
    int64_t frame_number, first_frame_number;
    int               play_status;          // 0 stop, 1 play, 2 end-of-file

    bool   rotation_auto;
    int    rotation_angle; // valid 0, 90, 180, 270
    double eps_zero;
/*
   'filename' contains the filename of the videosource,
   'filename==NULL' indicates that ffmpeg's seek support works
   for the particular file.
   'filename!=NULL' indicates that the slow fallback function is used for seeking,
   and so the filename is needed to reopen the file on backward seeking.
*/
    char              * filename;

    AVDictionary *dict;
#if USE_AV_INTERRUPT_CALLBACK
    int open_timeout;
    int read_timeout;
    AVInterruptCallbackMetadata interrupt_metadata;
#endif

    bool setRaw();
    bool processRawPacket();
    bool rawMode;
    bool rawModeInitialized;
    bool convertRGB;
    AVPacket packet_filtered;
#if LIBAVFORMAT_BUILD >= CALC_FFMPEG_VERSION(58, 20, 100)
    AVBSFContext* bsfc;
 #else
    AVBitStreamFilterContext* bsfc;
#endif
    VideoAccelerationType va_type;
    int hw_device;
    int use_opencl;
    int extraDataIdx;

#if SUPPORT_INPUT_THREADS
    av_inputs           input_fd;
#endif
    int                 chl_id;
    int                 card;

    // resampler parameter
    AVRational      sampler_r;
    double          sampler_step;
    double          sampler_cnt;
    double          out_yuv;

    // output buf
    unsigned char *pkt_buf;
    int   pkt_len;
    int   pkt_len_max;
};

void CvCapture_FFMPEG::init()
{
#ifdef HAVE_FFMPEG_LIBAVDEVICE
    //libavdevice is available, so let's register all input and output devices (e.g v4l2)
    avdevice_register_all();
#endif
    ic = 0;
    video_stream = -1;
    video_st = 0;
    picture = 0;
    picture_pts = AV_NOPTS_VALUE_;
    picture_raw_pts = AV_NOPTS_VALUE_;
    picture_width = -1;
    picture_height = -1;
    first_frame_number = -1;
    memset( &rgb_picture, 0, sizeof(rgb_picture) );
    memset( &frame, 0, sizeof(frame) );
    filename = 0;
    memset(&packet, 0, sizeof(packet));
    av_init_packet(&packet);
    img_convert_ctx = 0;

    avcodec = 0;
    context = 0;
    frame_number = 0;
    eps_zero = 0.000025;

    play_status = 0;
    rotation_angle = 0;

#if (LIBAVUTIL_BUILD >= CALC_FFMPEG_VERSION(52, 92, 100))
    rotation_auto = true;
#else
    rotation_auto = false;
#endif
    dict = NULL;

#if USE_AV_INTERRUPT_CALLBACK
    open_timeout = LIBAVFORMAT_INTERRUPT_OPEN_DEFAULT_TIMEOUT_MS;
    read_timeout = LIBAVFORMAT_INTERRUPT_READ_DEFAULT_TIMEOUT_MS;
#endif

    rawMode = false;
    rawModeInitialized = false;
    convertRGB = true;
    memset(&packet_filtered, 0, sizeof(packet_filtered));
    av_init_packet(&packet_filtered);
    bsfc = NULL;
    va_type = cv::VIDEO_ACCELERATION_NONE;  // TODO OpenCV 5.0: change to _ANY?
    hw_device = -1;
    use_opencl = 0;
    extraDataIdx = 1;

#if SUPPORT_INPUT_THREADS
    input_fd.init();
#endif

    chl_id = -1;
    card = 0;

    // resampler parameter
    sampler_r.num = 1;
    sampler_r.den = 1;
    sampler_cnt = 0.0;
    sampler_step  = 1.0;
    out_yuv = PROP_FALSE;
#ifdef WIN32
    interrupt_metadata.interrupt_lock = NULL;
#else
    memset(&interrupt_metadata, 0, sizeof(interrupt_metadata));
#endif
    pkt_buf     = NULL;
    pkt_len     = 0;
    pkt_len_max = 0;

}


void CvCapture_FFMPEG::close()
{
#if SUPPORT_INPUT_THREADS
    input_fd.destroy();
#endif

    if( img_convert_ctx )
    {
        sws_freeContext(img_convert_ctx);
        img_convert_ctx = 0;
    }

    if( picture )
    {
#if LIBAVCODEC_BUILD >= (LIBAVCODEC_VERSION_MICRO >= 100 \
    ? CALC_FFMPEG_VERSION(55, 45, 101) : CALC_FFMPEG_VERSION(55, 28, 1))
        av_frame_free(&picture);
#elif LIBAVCODEC_BUILD >= (LIBAVCODEC_VERSION_MICRO >= 100 \
    ? CALC_FFMPEG_VERSION(54, 59, 100) : CALC_FFMPEG_VERSION(54, 28, 0))
        avcodec_free_frame(&picture);
#else
        av_free(picture);
#endif
    }

    if( video_st )
    {
#ifdef CV_FFMPEG_CODECPAR
        avcodec_close( context );
#endif
        video_st = NULL;
    }

    if (context)
    {
#ifdef CV_FFMPEG_CODECPAR
        avcodec_free_context(&context);
#endif
    }

    if( ic )
    {
        avformat_close_input(&ic);
        ic = NULL;
    }

#if USE_AV_FRAME_GET_BUFFER
    av_frame_unref(&rgb_picture);
#else
    if( rgb_picture.data[0] )
    {
        free( rgb_picture.data[0] );
        rgb_picture.data[0] = 0;
    }
#endif

    // free last packet if exist
    if (packet.data) {
        _opencv_ffmpeg_av_packet_unref (&packet);
        packet.data = NULL;
    }

    if (dict != NULL)
       av_dict_free(&dict);

    if (packet_filtered.data)
    {
        _opencv_ffmpeg_av_packet_unref(&packet_filtered);
        packet_filtered.data = NULL;
    }

    if (bsfc)
    {
#if LIBAVFORMAT_BUILD >= CALC_FFMPEG_VERSION(58, 20, 100)
        av_bsf_free(&bsfc);
#else
        av_bitstream_filter_close(bsfc);
#endif
    }
#if USE_AV_INTERRUPT_CALLBACK
#ifdef WIN32
    if(interrupt_metadata.interrupt_lock)
        CloseHandle(interrupt_metadata.interrupt_lock);
#else
    pthread_mutex_destroy(&interrupt_metadata.interrupt_lock);
#endif
#endif

    // bm codec private function
    if (chl_id >= 0 && chl_id < MAX_VIDEO_CHL_NUM &&
        g_bm_callback_info[chl_id].used &&
        g_bm_callback_info[chl_id].chl_id == chl_id)
    {
        dump_close(&g_bm_callback_info[chl_id].debug_param);

        callback_init(&g_bm_callback_info[chl_id]);
    }

    if (pkt_buf != NULL) {
        av_free(pkt_buf);
        pkt_len_max = 0;
        pkt_len = 0;
    }

    init(); // will init chl_id=-1, keep init() at the end
}


#ifndef AVSEEK_FLAG_FRAME
#define AVSEEK_FLAG_FRAME 0
#endif
#ifndef AVSEEK_FLAG_ANY
#define AVSEEK_FLAG_ANY 1
#endif

#if defined(__OPENCV_BUILD) || defined(BUILD_PLUGIN)
typedef cv::Mutex ImplMutex;
#else
class ImplMutex
{
public:
    ImplMutex() { init(); }
    ~ImplMutex() { destroy(); }

    void init();
    void destroy();

    void lock();
    bool trylock();
    void unlock();

    struct Impl;
protected:
    Impl* impl;

private:
    ImplMutex(const ImplMutex&);
    ImplMutex& operator = (const ImplMutex& m);
};

#if defined _WIN32 || defined WINCE

struct ImplMutex::Impl
{
    void init()
    {
#if (_WIN32_WINNT >= 0x0600)
        ::InitializeCriticalSectionEx(&cs, 1000, 0);
#else
        ::InitializeCriticalSection(&cs);
#endif
        refcount = 1;
    }
    void destroy() { DeleteCriticalSection(&cs); }

    void lock() { EnterCriticalSection(&cs); }
    bool trylock() { return TryEnterCriticalSection(&cs) != 0; }
    void unlock() { LeaveCriticalSection(&cs); }

    CRITICAL_SECTION cs;
    int refcount;
};


#elif defined __APPLE__

#include <libkern/OSAtomic.h>

struct ImplMutex::Impl
{
    void init() { sl = OS_SPINLOCK_INIT; refcount = 1; }
    void destroy() { }

    void lock() { OSSpinLockLock(&sl); }
    bool trylock() { return OSSpinLockTry(&sl); }
    void unlock() { OSSpinLockUnlock(&sl); }

    OSSpinLock sl;
    int refcount;
};

#elif defined __linux__ && !defined __ANDROID__

struct ImplMutex::Impl
{
    void init() { pthread_spin_init(&sl, 0); refcount = 1; }
    void destroy() { pthread_spin_destroy(&sl); }

    void lock() { pthread_spin_lock(&sl); }
    bool trylock() { return pthread_spin_trylock(&sl) == 0; }
    void unlock() { pthread_spin_unlock(&sl); }

    pthread_spinlock_t sl;
    int refcount;
};

#else

struct ImplMutex::Impl
{
    void init() { pthread_mutex_init(&sl, 0); refcount = 1; }
    void destroy() { pthread_mutex_destroy(&sl); }

    void lock() { pthread_mutex_lock(&sl); }
    bool trylock() { return pthread_mutex_trylock(&sl) == 0; }
    void unlock() { pthread_mutex_unlock(&sl); }

    pthread_mutex_t sl;
    int refcount;
};

#endif

void ImplMutex::init()
{
    impl = new Impl();
    impl->init();
}
void ImplMutex::destroy()
{
    impl->destroy();
    delete(impl);
    impl = NULL;
}
void ImplMutex::lock() { impl->lock(); }
void ImplMutex::unlock() { impl->unlock(); }
bool ImplMutex::trylock() { return impl->trylock(); }

class AutoLock
{
public:
    AutoLock(ImplMutex& m) : mutex(&m) { mutex->lock(); }
    ~AutoLock() { mutex->unlock(); }
protected:
    ImplMutex* mutex;
private:
    AutoLock(const AutoLock&); // disabled
    AutoLock& operator = (const AutoLock&); // disabled
};
#endif


static ImplMutex _mutex;
static ImplMutex _mutex_writer;
static bool _initialized = false;

#ifdef CV_FFMPEG_LOCKMGR
#if LIBAVCODEC_BUILD < CALC_FFMPEG_VERSION(58, 9, 100)
static int LockCallBack(void **mutex, AVLockOp op)
{
    ImplMutex* localMutex = reinterpret_cast<ImplMutex*>(*mutex);
    switch (op)
    {
        case AV_LOCK_CREATE:
            localMutex = new ImplMutex();
            if (!localMutex)
                return 1;
            *mutex = localMutex;
            if (!*mutex)
                return 1;
        break;

        case AV_LOCK_OBTAIN:
            localMutex->lock();
        break;

        case AV_LOCK_RELEASE:
            localMutex->unlock();
        break;

        case AV_LOCK_DESTROY:
            delete localMutex;
            localMutex = NULL;
            *mutex = NULL;
        break;
    }
    return 0;
}
#endif
#endif

// static void ffmpeg_log_callback(void *ptr, int level, const char *fmt, va_list vargs)
// {
//     static bool skip_header = false;
//     static int prev_level = -1;
//     CV_UNUSED(ptr);
//     if (level>av_log_get_level()) return;
//     if (!skip_header || level != prev_level) printf("[OPENCV:FFMPEG:%02d] ", level);
//     vprintf(fmt, vargs);
//     size_t fmt_len = strlen(fmt);
//     skip_header = fmt_len > 0 && fmt[fmt_len - 1] != '\n';
//     prev_level = level;
// }

class InternalFFMpegRegister
{
public:
    // static void init(const bool threadSafe)
    // {
    //     std::unique_lock<cv::Mutex> lock(_mutex, std::defer_lock);
    //     if(!threadSafe)
    //         lock.lock();
    //     static InternalFFMpegRegister instance;
    //     initLogger_();  // update logger setup unconditionally (GStreamer's libav plugin may override these settings)
    // }
    static void initLogger_()
    {
// #ifndef NO_GETENV
//         char* debug_option = getenv("OPENCV_FFMPEG_DEBUG");
//         char* level_option = getenv("OPENCV_FFMPEG_LOGLEVEL");
//         int level = AV_LOG_VERBOSE;
//         if (level_option != NULL)
//         {
//             level = atoi(level_option);
//         }
//         if ( (debug_option != NULL) || (level_option != NULL) )
//         {
//             av_log_set_level(level);
//             av_log_set_callback(ffmpeg_log_callback);
//         }
//         else
// #endif
        {
            av_log_set_level(AV_LOG_ERROR);
        }
    }

public:
    InternalFFMpegRegister()
    {
        AutoLock lock(_mutex);
        if (!_initialized)
        {

            avformat_network_init();

#ifdef CV_FFMPEG_REGISTER
#if LIBAVFORMAT_BUILD < CALC_FFMPEG_VERSION(58, 9, 100)

            /* register all codecs, demux and protocols */
            av_register_all();
#endif
#endif

#ifdef CV_FFMPEG_LOCKMGR
#if LIBAVCODEC_BUILD < CALC_FFMPEG_VERSION(58, 9, 100)
            /* register a callback function for synchronization */
            av_lockmgr_register(&LockCallBack);
#endif
#endif

            av_log_set_level(AV_LOG_ERROR);

            _initialized = true;

            /* initialize g_bm_callback_info array */
            for (int i = 0; i < MAX_VIDEO_CHL_NUM; i++)
                callback_init(&g_bm_callback_info[i]);

        }
    }
    ~InternalFFMpegRegister()
    {
#ifdef CV_FFMPEG_LOCKMGR
        _initialized = false;
#if LIBAVCODEC_BUILD < CALC_FFMPEG_VERSION(58, 9, 100)

        av_lockmgr_register(NULL);
#endif
#endif
        av_log_set_callback(NULL);
    }
};

static InternalFFMpegRegister _init;

static void dump_init(_Debug_Param *param, int id)
{
    char *debug_env = getenv("BMIVA_DEBUG");
    param->debug_level = 0;
    if (debug_env)
        param->debug_level = atoi(debug_env);

    if (param->debug_level > 0)
    {
        char vid_filename[256];
        sprintf(vid_filename, "video_y_%d.bin", id);
        param->debug_fp[0] = fopen(vid_filename, "wb");
        CV_Assert(param->debug_fp[0]);
        sprintf(vid_filename, "video_uv_%d.bin", id);
        param->debug_fp[1] = fopen(vid_filename, "wb");
        CV_Assert(param->debug_fp[1]);
    }
}

static void dump_close(_Debug_Param *param)
{
    if (param->debug_level > 0)
    {
        if (param->debug_fp[0]) {
            fclose(param->debug_fp[0]);
        }
        if (param->debug_fp[1]) {
            fclose(param->debug_fp[1]);
        }
    }
}

static void dump_write(_Debug_Param *param, AVFrame *frame)
{
    if (param->debug_level > 0)
    {
        for (int h = 0; h < frame->height; h++)
            fwrite((void*)(frame->data[0] + frame->linesize[0] * h),
                   1, frame->width, param->debug_fp[0]);

        std::cout << "write Y done" << std::endl;

        for (int h = 0; h < (frame->height+1)/2; h++)
            fwrite((void*)(frame->data[1] + frame->linesize[1] * h),
                   1, frame->width, param->debug_fp[1]);

        std::cout << "Write CbCr done" << std::endl;
    }
}

static void callback_init(_Bmvid_Callback_Info *info)
{
    info->debug_param.debug_level = 0;
    info->chl_id = -1;
    info->used = 0;
}

static void bm_find_decoder_name(int dec_id, std::string &dec_name)
{
    switch (dec_id)
    {
    case AV_CODEC_ID_MJPEG:      dec_name = "jpeg_bm";    break;
    case AV_CODEC_ID_H264:       dec_name = "h264_bm";    break;
    case AV_CODEC_ID_HEVC:       dec_name = "hevc_bm";    break;
#ifdef VPP_BM1682
    case AV_CODEC_ID_MPEG1VIDEO: dec_name = "mpeg1_bm";   break;
    case AV_CODEC_ID_MPEG2VIDEO: dec_name = "mpeg2_bm";   break;
    case AV_CODEC_ID_MPEG4:      dec_name = "mpeg4_bm";   break;
    case AV_CODEC_ID_MSMPEG4V3:  dec_name = "mpeg4v3_bm"; break;
    case AV_CODEC_ID_FLV1:       dec_name = "flv1_bm";    break;
    case AV_CODEC_ID_H263:       dec_name = "h263_bm";    break;
    case AV_CODEC_ID_CAVS:       dec_name = "cavs_bm";    break;
    case AV_CODEC_ID_AVS:        dec_name = "avs_bm";     break;
    case AV_CODEC_ID_VP3:        dec_name = "vp3_bm";     break;
    case AV_CODEC_ID_VP8:        dec_name = "vp8_bm";     break;
    case AV_CODEC_ID_VC1:        dec_name = "vc1_bm";     break;
    case AV_CODEC_ID_WMV1:       dec_name = "wmv1_bm";    break;
    case AV_CODEC_ID_WMV2:       dec_name = "wmv2_bm";    break;
    case AV_CODEC_ID_WMV3:       dec_name = "wmv3_bm";    break;
#endif
    default:                     dec_name = "";           break;
    }
}

inline void fill_codec_context(AVCodecContext * enc, AVDictionary * dict)
{
    if (!enc->thread_count)
    {
        int nCpus = cv::getNumberOfCPUs();
        int requestedThreads = std::min(nCpus, 16);  // [OPENCV:FFMPEG:24] Application has requested XX threads. Using a thread count greater than 16 is not recommended.
        char* threads_option = getenv("OPENCV_FFMPEG_THREADS");
        if (threads_option != NULL)
        {
            requestedThreads = atoi(threads_option);
        }
        enc->thread_count = requestedThreads;
    }

    AVDictionaryEntry* avdiscard_entry = av_dict_get(dict, "avdiscard", NULL, 0);

    if (avdiscard_entry)
    {
        if(strcmp(avdiscard_entry->value, "all") == 0)
            enc->skip_frame = AVDISCARD_ALL;
        else if (strcmp(avdiscard_entry->value, "bidir") == 0)
            enc->skip_frame = AVDISCARD_BIDIR;
        else if (strcmp(avdiscard_entry->value, "default") == 0)
            enc->skip_frame = AVDISCARD_DEFAULT;
        else if (strcmp(avdiscard_entry->value, "none") == 0)
            enc->skip_frame = AVDISCARD_NONE;
        // NONINTRA flag was introduced with version bump at revision:
        // https://github.com/FFmpeg/FFmpeg/commit/b152152df3b778d0a86dcda5d4f5d065b4175a7b
        // This key is supported only for FFMPEG version
#if LIBAVCODEC_VERSION_MICRO >= 100 && LIBAVCODEC_BUILD >= CALC_FFMPEG_VERSION(55, 67, 100)
        else if (strcmp(avdiscard_entry->value, "nonintra") == 0)
            enc->skip_frame = AVDISCARD_NONINTRA;
#endif
        else if (strcmp(avdiscard_entry->value, "nonkey") == 0)
            enc->skip_frame = AVDISCARD_NONKEY;
        else if (strcmp(avdiscard_entry->value, "nonref") == 0)
            enc->skip_frame = AVDISCARD_NONREF;
    }
}

static bool isThreadSafe() {
    const bool threadSafe = utils::getConfigurationParameterBool("OPENCV_FFMPEG_IS_THREAD_SAFE", false);
    if (threadSafe) {
        CV_LOG_WARNING(NULL, "VIDEOIO/FFMPEG: OPENCV_FFMPEG_IS_THREAD_SAFE == 1, all OpenCV locks removed, relying on FFmpeg to provide thread safety.  If FFmpeg is not thread safe isOpened() may return false when multiple threads try to call open() at the same time.");
    }
    return threadSafe;
}

bool CvCapture_FFMPEG::open(const char* _filename, const VideoCaptureParameters& params, int id)
{
    const bool threadSafe = isThreadSafe();
    // InternalFFMpegRegister::init(threadSafe);

    std::unique_lock<cv::Mutex> lock(_mutex, std::defer_lock);
    if(!threadSafe)
        lock.lock();

    unsigned i;
    bool valid = false;
    int nThreads = 0;

    close();
    card = id;

    if (!params.empty())
    {
        convertRGB = params.get<bool>(CAP_PROP_CONVERT_RGB, true);
        if (!convertRGB)
        {
            CV_LOG_WARNING(NULL, "VIDEOIO/FFMPEG: BGR conversion turned OFF, decoded frame will be "
                                 "returned in its original format. "
                                 "Multiplanar formats are not supported by the backend. "
                                 "Only GRAY8/GRAY16LE pixel formats have been tested. "
                                 "Use at your own risk.");
        }
        if (params.has(CAP_PROP_FORMAT))
        {
            int value = params.get<int>(CAP_PROP_FORMAT);
            if (value == -1)
            {
                CV_LOG_INFO(NULL, "VIDEOIO/FFMPEG: enabled demuxer only mode: '" << (_filename ? _filename : "<NULL>") << "'");
                rawMode = true;
            }
            else
            {
                CV_LOG_ERROR(NULL, "VIDEOIO/FFMPEG: CAP_PROP_FORMAT parameter value is invalid/unsupported: " << value);
                return false;
            }
        }
        if (params.has(CAP_PROP_HW_ACCELERATION))
        {
            va_type = params.get<VideoAccelerationType>(CAP_PROP_HW_ACCELERATION);
#if !USE_AV_HW_CODECS
            if (va_type != VIDEO_ACCELERATION_NONE && va_type != VIDEO_ACCELERATION_ANY)
            {
                CV_LOG_ERROR(NULL, "VIDEOIO/FFMPEG: FFmpeg backend is build without acceleration support. Can't handle CAP_PROP_HW_ACCELERATION parameter. Bailout");
                return false;
            }
#endif
        }
        if (params.has(CAP_PROP_HW_DEVICE))
        {
            hw_device = params.get<int>(CAP_PROP_HW_DEVICE);
            if (va_type == VIDEO_ACCELERATION_NONE && hw_device != -1)
            {
                CV_LOG_ERROR(NULL, "VIDEOIO/FFMPEG: Invalid usage of CAP_PROP_HW_DEVICE without requested H/W acceleration. Bailout");
                return false;
            }
            if (va_type == VIDEO_ACCELERATION_ANY && hw_device != -1)
            {
                CV_LOG_ERROR(NULL, "VIDEOIO/FFMPEG: Invalid usage of CAP_PROP_HW_DEVICE with 'ANY' H/W acceleration. Bailout");
                return false;
            }
        }
        if (params.has(CAP_PROP_HW_ACCELERATION_USE_OPENCL)) {
            use_opencl = params.get<int>(CAP_PROP_HW_ACCELERATION_USE_OPENCL);
        }
#if USE_AV_INTERRUPT_CALLBACK
        if (params.has(CAP_PROP_OPEN_TIMEOUT_MSEC))
        {
            open_timeout = params.get<int>(CAP_PROP_OPEN_TIMEOUT_MSEC);
        }
        if (params.has(CAP_PROP_READ_TIMEOUT_MSEC))
        {
            read_timeout = params.get<int>(CAP_PROP_READ_TIMEOUT_MSEC);
        }
#endif
        if (params.has(CAP_PROP_N_THREADS))
        {
            nThreads = params.get<int>(CAP_PROP_N_THREADS);
        }
        if (params.warnUnusedParameters())
        {
            CV_LOG_ERROR(NULL, "VIDEOIO/FFMPEG: unsupported parameters in .open(), see logger INFO channel for details. Bailout");
            return false;
        }
    }

#if USE_AV_INTERRUPT_CALLBACK
    /* interrupt callback */
    char* time_out = NULL;
    time_out = getenv("OPENCV_FFMPEG_OPEN_TIMEOUT_MS");
    if (time_out!=NULL && atoi(time_out)>0)
        open_timeout = atoi(time_out);

    time_out = getenv("OPENCV_FFMPEG_READ_TIMEOUT_MS");
    if (time_out!=NULL && atoi(time_out)>0)
        read_timeout = atoi(time_out);

    interrupt_metadata.timeout_after_ms = open_timeout;
    get_monotonic_time(&interrupt_metadata.value);
#ifdef WIN32
    interrupt_metadata.interrupt_lock = CreateMutex(NULL, FALSE, NULL);
#else
    pthread_mutex_init(&interrupt_metadata.interrupt_lock, NULL);
#endif

    ic = avformat_alloc_context();
    ic->interrupt_callback.callback = _opencv_ffmpeg_interrupt_callback;
    ic->interrupt_callback.opaque = &interrupt_metadata;
#endif

#ifndef NO_GETENV
    char* options = getenv("OPENCV_FFMPEG_CAPTURE_OPTIONS");
    if(options == NULL)
    {
        av_dict_set_int(&dict, "buffer_size", 1024000, 0);
        av_dict_set_int(&dict, "max_delay", 500000, 0);
#if LIBAVFORMAT_VERSION_MICRO >= 100  && LIBAVFORMAT_BUILD >= CALC_FFMPEG_VERSION(55, 48, 100)
        av_dict_set(&dict, "rtsp_flags", "prefer_tcp", 0);
#else
        av_dict_set_int(&dict, "stimeout", 20000000, 0);
#endif
    }
    else
    {
        CV_LOG_DEBUG(NULL, "VIDEOIO/FFMPEG: using capture options from environment: " << options);
#if LIBAVUTIL_BUILD >= (LIBAVUTIL_VERSION_MICRO >= 100 ? CALC_FFMPEG_VERSION(52, 17, 100) : CALC_FFMPEG_VERSION(52, 7, 0))
        av_dict_parse_string(&dict, options, ";", "|", 0);
#else
#endif
    }
#else
#endif

    av_dict_set(&dict, "refcounted_frames", "1" , 0);
    av_dict_set(&dict, "rtsp_flags", "prefer_tcp", 0);
    //av_dict_set(&dict, "handle_packet_loss", "1", 0); // avoid packet loss
#ifdef HAVE_BMCV
    av_dict_set_int(&dict, "output_format", 101, 0);
    av_dict_set_int(&dict, "extra_frame_buffer_num", 2, 0);
#endif
#ifndef USING_SOC
    av_dict_set_int(&dict, "zero_copy", 1, 0);
    av_dict_set_int(&dict, "sophon_idx", BM_CARD_ID(card), 0);
#endif

    CV_FFMPEG_FMT_CONST AVInputFormat* input_format = NULL;
    AVDictionaryEntry* entry = av_dict_get(dict, "input_format", NULL, 0);
    if (entry != 0)
    {
      input_format = av_find_input_format(entry->value);
    }

    int err = avformat_open_input(&ic, _filename, input_format, &dict);

    if (err < 0)
    {
        CV_WARN("Error opening file");
        CV_WARN(_filename);
        goto exit_func;
    }
    err = avformat_find_stream_info(ic, NULL);
    if (err < 0)
    {
        CV_LOG_WARNING(NULL, "Unable to read codec parameters from stream (" << _opencv_ffmpeg_get_error_string(err) << ")");
        goto exit_func;
    }
    for(i = 0; i < ic->nb_streams; i++)
    {
#ifndef CV_FFMPEG_CODECPAR
        context = ic->streams[i]->codec;
        AVCodecID codec_id = context->codec_id;
        AVMediaType codec_type = context->codec_type;
#else
        AVCodecParameters* par = ic->streams[i]->codecpar;
        AVCodecID codec_id = par->codec_id;
        AVMediaType codec_type = par->codec_type;
#endif

        if( AVMEDIA_TYPE_VIDEO == codec_type && video_stream < 0)
        {
            // backup encoder' width/height
#ifndef CV_FFMPEG_CODECPAR
            int enc_width = context->width;
            int enc_height = context->height;
#else
            int enc_width = par->width;
            int enc_height = par->height;
#endif

            std::string bm_dec_name = "";  // 调试下 才知道具体调到哪里
            CV_LOG_DEBUG(NULL, "FFMPEG: stream[" << i << "] is video stream with codecID=" << (int)codec_id
                    << " width=" << enc_width
                    << " height=" << enc_height
            );
            bm_find_decoder_name(codec_id, bm_dec_name);

#if !USE_AV_HW_CODECS
            va_type = VIDEO_ACCELERATION_NONE;
#endif

            // find and open decoder, try HW acceleration types specified in 'hw_acceleration' list (in order)
            const AVCodec *codec = NULL;
            err = -1;
#if USE_AV_HW_CODECS
            HWAccelIterator accel_iter(va_type, false/*isEncoder*/, dict);
            while (accel_iter.good())
            {
#else
            do {
#endif
#if USE_AV_HW_CODECS
                accel_iter.parse_next();
                AVHWDeviceType hw_type = accel_iter.hw_type();
                if (hw_type != AV_HWDEVICE_TYPE_NONE)
                {
                    CV_LOG_DEBUG(NULL, "FFMPEG: trying to configure H/W acceleration: '" << accel_iter.hw_type_device_string() << "'");
                    AVPixelFormat hw_pix_fmt = AV_PIX_FMT_NONE;
                    codec = hw_find_codec(codec_id, hw_type, av_codec_is_decoder, accel_iter.disabled_codecs().c_str(), &hw_pix_fmt);
                    if (codec)
                    {
#ifdef CV_FFMPEG_CODECPAR
                        context = avcodec_alloc_context3(codec);
#endif
                        CV_Assert(context);
                        context->get_format = avcodec_default_get_format;
                        if (context->hw_device_ctx) {
                            av_buffer_unref(&context->hw_device_ctx);
                        }
                        if (hw_pix_fmt != AV_PIX_FMT_NONE)
                            context->get_format = hw_get_format_callback; // set callback to select HW pixel format, not SW format
                        context->hw_device_ctx = hw_create_device(hw_type, hw_device, accel_iter.device_subname(), use_opencl != 0);
                        if (!context->hw_device_ctx)
                        {
                            context->get_format = avcodec_default_get_format;
                            CV_LOG_DEBUG(NULL, "FFMPEG: ... can't create H/W device: '" << accel_iter.hw_type_device_string() << "'");
                            codec = NULL;
                        }
                    }
                }
                else if (hw_type == AV_HWDEVICE_TYPE_NONE)
#endif // USE_AV_HW_CODECS
                {
                    AVDictionaryEntry* video_codec_param = av_dict_get(dict, "video_codec", NULL, 0);
                    if (video_codec_param == NULL)
                    {
                        if (!bm_dec_name.empty())
                        {
                            codec = avcodec_find_decoder_by_name(bm_dec_name.c_str());
                        }
                        if (!codec)
                        {
                            codec = avcodec_find_decoder(codec_id);
                        }
                        if (!codec)
                        {
                            CV_LOG_ERROR(NULL, "Could not find decoder for codec_id=" << (int)codec_id);
                        }
                    }
                    else
                    {
                        CV_LOG_DEBUG(NULL, "FFMPEG: Using video_codec='" << video_codec_param->value << "'");
                        codec = avcodec_find_decoder_by_name(video_codec_param->value);
                        if (!codec)
                        {
                            CV_LOG_ERROR(NULL, "Could not find decoder '" << video_codec_param->value << "'");
                        }
                    }
                    if (codec)
                    {
#ifdef CV_FFMPEG_CODECPAR
                        context = avcodec_alloc_context3(codec);
#endif
                        CV_Assert(context);
                    }
                }
                if (!codec)
                {
#ifdef CV_FFMPEG_CODECPAR
                    avcodec_free_context(&context);
#endif
                    continue;
                }
                context->thread_count = nThreads;
                fill_codec_context(context, dict);

                if (!bm_dec_name.compare("jpeg_bm"))
                {
                    /* The output of bm jpeg decoder is NV12 */
                    av_dict_set(&dict, "chroma_interleave", "1", 0);
                    /* The bitstream buffer is 3100KB,
                     * assuming the max dimension is 1920x1080 */
                    av_dict_set(&dict, "bs_buffer_size", "3100", 0);
                    /* Extra frame buffers for mjpeg video */
                    av_dict_set(&dict, "num_extra_framebuffers", "2", 0);
                }

#ifdef CV_FFMPEG_CODECPAR
                avcodec_parameters_to_context(context, par);
#endif
                err = avcodec_open2(context, codec, &dict);
                if (err >= 0) {
#if USE_AV_HW_CODECS
                    va_type = hw_type_to_va_type(hw_type);
                    if (hw_type != AV_HWDEVICE_TYPE_NONE && hw_device < 0)
                        hw_device = 0;
#endif
                    break;
                } else {
                    CV_LOG_ERROR(NULL, "Could not open codec " << codec->name << ", error: " << err);
                }
#if USE_AV_HW_CODECS
            }  // while (accel_iter.good())
#else
            } while (0);
#endif
            if (err < 0) {
                CV_LOG_ERROR(NULL, "VIDEOIO/FFMPEG: Failed to initialize VideoCapture");
                goto exit_func;
            }

            // checking width/height (since decoder can sometimes alter it, eg. vp6f)
            if (enc_width && (context->width != enc_width))
                context->width = enc_width;
            if (enc_height && (context->height != enc_height))
                context->height = enc_height;

            video_stream = i;
            video_st = ic->streams[i];
#if LIBAVCODEC_BUILD >= (LIBAVCODEC_VERSION_MICRO >= 100 \
    ? CALC_FFMPEG_VERSION(55, 45, 101) : CALC_FFMPEG_VERSION(55, 28, 1))
            picture = av_frame_alloc();
#else
            picture = avcodec_alloc_frame();
#endif

            frame.width = frame.display_width = context->width;
            frame.height = frame.display_height = context->height;
            frame.step = 0;
            frame.data = NULL;
            frame.format = context->pix_fmt;
            get_rotation_angle();

            /* bm codec private code */
            {
                /* get g_bm_callback_info resource */
                for (chl_id = 0; chl_id < MAX_VIDEO_CHL_NUM; chl_id++)
                    if (!g_bm_callback_info[chl_id].used)
                        break;
                if (chl_id >= MAX_VIDEO_CHL_NUM)      // no resource
                    goto exit_func;

                g_bm_callback_info[chl_id].used = 1;
                g_bm_callback_info[chl_id].chl_id = chl_id;

                dump_init(&g_bm_callback_info[chl_id].debug_param, chl_id);
            }

#if SUPPORT_INPUT_THREADS
            input_fd.start_input_thread(ic);
#endif

            break;
        }
    }

    if (video_stream >= 0) {
        valid = true;
        play_status = 1;
    }

exit_func:

#if USE_AV_INTERRUPT_CALLBACK
    // deactivate interrupt callback
#ifdef WIN32
    WaitForSingleObject(interrupt_metadata.interrupt_lock, INFINITE);
    interrupt_metadata.timeout_after_ms = 0;
    ReleaseMutex(interrupt_metadata.interrupt_lock);
#else
    pthread_mutex_lock(&interrupt_metadata.interrupt_lock);
    interrupt_metadata.timeout_after_ms = 0;
    pthread_mutex_unlock(&interrupt_metadata.interrupt_lock);
#endif
#endif

    if( !valid )
        close();

    return valid;
}

bool CvCapture_FFMPEG::setRaw()
{
    if (!rawMode)
    {
        if (frame_number != 0)
        {
            CV_WARN("Incorrect usage: do not grab frames before .set(CAP_PROP_FORMAT, -1)");
        }
        // binary stream filter creation is moved into processRawPacket()
        rawMode = true;
    }
    return true;
}

static inline bool h26xContainer(const char* formatLongName) {
    return !strcmp(formatLongName, "QuickTime / MOV") || !strcmp(formatLongName, "FLV (Flash Video)") || !strcmp(formatLongName, "Matroska / WebM");
}

bool CvCapture_FFMPEG::processRawPacket()
{
    if (packet.data == NULL)  // EOF
        return false;
    if (!rawModeInitialized)
    {
        rawModeInitialized = true;
#if LIBAVFORMAT_BUILD >= CALC_FFMPEG_VERSION(58, 20, 100)
        CV_CODEC_ID eVideoCodec = ic->streams[video_stream]->codecpar->codec_id;
#else
        CV_CODEC_ID eVideoCodec = video_st->codec->codec_id;
#endif
        const char* filterName = NULL;
        if (eVideoCodec == CV_CODEC(CODEC_ID_H264)
#if LIBAVCODEC_VERSION_MICRO >= 100 \
    && LIBAVCODEC_BUILD >= CALC_FFMPEG_VERSION(57, 24, 102)  // FFmpeg 3.0
            || eVideoCodec == CV_CODEC(CODEC_ID_H265)
#elif LIBAVCODEC_VERSION_MICRO < 100 \
    && LIBAVCODEC_BUILD >= CALC_FFMPEG_VERSION(55, 34, 1)  // libav v10+
            || eVideoCodec == CV_CODEC(CODEC_ID_HEVC)
#endif
        )
        {
            if(h26xContainer(ic->iformat->long_name))
                filterName = eVideoCodec == CV_CODEC(CODEC_ID_H264) ? "h264_mp4toannexb" : "hevc_mp4toannexb";
        }
        if (filterName)
        {
#if LIBAVFORMAT_BUILD >= CALC_FFMPEG_VERSION(58, 20, 100)
            const AVBitStreamFilter * bsf = av_bsf_get_by_name(filterName);
            if (!bsf)
            {
                CV_WARN(cv::format("Bitstream filter is not available: %s", filterName).c_str());
                return false;
            }
            int err = av_bsf_alloc(bsf, &bsfc);
            if (err < 0)
            {
                CV_WARN("Error allocating context for bitstream buffer");
                return false;
            }
            avcodec_parameters_copy(bsfc->par_in, ic->streams[video_stream]->codecpar);
            err = av_bsf_init(bsfc);
            if (err < 0)
            {
                CV_WARN("Error initializing bitstream buffer");
                return false;
            }
#else
            bsfc = av_bitstream_filter_init(filterName);
            if (!bsfc)
            {
                CV_WARN(cv::format("Bitstream filter is not available: %s", filterName).c_str());
                return false;
            }
#endif
        }
    }
    if (bsfc)
    {
        if (packet_filtered.data)
        {
            _opencv_ffmpeg_av_packet_unref(&packet_filtered);
        }

#if LIBAVFORMAT_BUILD >= CALC_FFMPEG_VERSION(58, 20, 100)
        int err = av_bsf_send_packet(bsfc, &packet);
        if (err < 0)
        {
            CV_WARN("Packet submission for filtering failed");
            return false;
        }
        err = av_bsf_receive_packet(bsfc, &packet_filtered);
        if (err < 0)
        {
            CV_WARN("Filtered packet retrieve failed");
            return false;
        }
#else
        AVCodecContext* ctx = ic->streams[video_stream]->codec;
        int err = av_bitstream_filter_filter(bsfc, ctx, NULL, &packet_filtered.data,
            &packet_filtered.size, packet.data, packet.size, packet_filtered.flags & AV_PKT_FLAG_KEY);
        if (err < 0)
        {
            CV_WARN("Packet filtering failed");
            return false;
        }
#endif
        return packet_filtered.data != NULL;
    }
    return packet.data != NULL;
}

bool CvCapture_FFMPEG::grabFrame(char *buf, unsigned int len_in, unsigned int *len_out)
{
    bool valid = false;
    int count_errs = 0;
    const int max_number_of_attempts = 1 << 9;

    static const size_t max_read_attempts = cv::utils::getConfigurationParameterSizeT("OPENCV_FFMPEG_READ_ATTEMPTS", 4096);
    static const size_t max_decode_attempts = cv::utils::getConfigurationParameterSizeT("OPENCV_FFMPEG_DECODE_ATTEMPTS", 64);
    size_t cur_read_attempts = 0;
    size_t cur_decode_attempts = 0;
    int prev_err = 0;

    if( !ic || !video_st || !context )  return false;

    if( ic->streams[video_stream]->nb_frames > 0 &&
        frame_number > ic->streams[video_stream]->nb_frames )
        return false;

    picture_pts = AV_NOPTS_VALUE_;
    picture_raw_pts = AV_NOPTS_VALUE_;

#if USE_AV_INTERRUPT_CALLBACK
    // activate interrupt callback
    interrupt_metadata.timeout = 0;
    get_monotonic_time(&interrupt_metadata.value);
#ifdef WIN32
    WaitForSingleObject(interrupt_metadata.interrupt_lock, INFINITE);
    interrupt_metadata.timeout_after_ms = read_timeout;
    ReleaseMutex(interrupt_metadata.interrupt_lock);
#else
    pthread_mutex_lock(&interrupt_metadata.interrupt_lock);
    interrupt_metadata.timeout_after_ms = read_timeout;
    pthread_mutex_unlock(&interrupt_metadata.interrupt_lock);
#endif
#endif

#if USE_AV_SEND_FRAME_API
    // check if we can receive frame from previously decoded packet
    if (picture) av_frame_free(&picture);
    picture = av_frame_alloc();
    valid = avcodec_receive_frame(context, picture) >= 0;
#endif

    // get the next frame
    while (!valid)
    {
        if (count_errs > max_number_of_attempts) {
            std::cout << "maybe grab ends normally, retry count = " << count_errs << std::endl;
            break;
        }

        _opencv_ffmpeg_av_packet_unref (&packet);

#if USE_AV_INTERRUPT_CALLBACK
        if (interrupt_metadata.timeout)
        {
            valid = false;
            std::cout << "AV_INTERRRUPT_TIMEOUT recevied" << std::endl;
            play_status = 2;    // end of file
#ifdef WIN32
            WaitForSingleObject(interrupt_metadata.interrupt_lock, INFINITE);
            interrupt_metadata.timeout_after_ms = 0;
            interrupt_metadata.timeout = 0; // avoid repeat timeout
            ReleaseMutex(interrupt_metadata.interrupt_lock);
#else
            pthread_mutex_lock(&interrupt_metadata.interrupt_lock);
            interrupt_metadata.timeout_after_ms = 0;
            interrupt_metadata.timeout = 0; // avoid repeat timeout
            pthread_mutex_unlock(&interrupt_metadata.interrupt_lock);
#endif
            break;
        }
#endif

#if SUPPORT_INPUT_THREADS
        int ret = input_fd.pkt_list_packet_recv(&packet);
#else
        int ret = av_read_frame(ic, &packet);
#endif

        if (ret == AVERROR(EAGAIN))
            continue;

        if (ret == AVERROR_EOF || ret == AVERROR_INVALIDDATA)
        {
            if (rawMode)
                break;

            // flush cached frames from video decoder
            packet.data = NULL;
            packet.size = 0;
            packet.stream_index = video_stream;
            play_status = 2;    // end of file
        } else if (ret < 0){
            if (ret != AVERROR(ETIMEDOUT)){ // abnormal error
                if (ret != prev_err)
	                fprintf(stderr, "warning: unknown error 0x%08x (%s:%d)\n", ret, __FILE__, __LINE__);
                play_status = 2; // treate as eof
			}
            else if (ret != prev_err)
                fprintf(stderr, "warning: %s (%s:%d)\n", "connection timeout", __FILE__, __LINE__);
            prev_err = ret;
        }


        if((packet.stream_index != video_stream || packet.size == 0) &&
           (ret != AVERROR_EOF))    // bypass EOF to flush frame
        {
            _opencv_ffmpeg_av_packet_unref (&packet);
            if (++cur_read_attempts > max_read_attempts)
            {
                CV_LOG_WARNING(NULL,
                    "packet read max attempts exceeded, if your video have "
                    "multiple streams (video, audio) try to increase attempt "
                    "limit by setting environment variable OPENCV_FFMPEG_READ_ATTEMPTS "
                    "(current value is " << max_read_attempts << ")");
                break;
            }
            continue;
        }

        if (rawMode)
        {
            valid = processRawPacket();
            break;
        }

        if ((buf != NULL) && (pkt_buf == NULL)) {
            pkt_buf = (unsigned char*)av_mallocz(video_st->codecpar->width * video_st->codecpar->height*3);
            pkt_len = 0;
            pkt_len_max = video_st->codecpar->width * video_st->codecpar->height * 3;
        }
        if (video_st->codecpar->codec_id == AV_CODEC_ID_MPEG4){  // other codec could be added later
            if ((packet.flags & AV_PKT_FLAG_KEY) && video_st->codecpar->extradata_size && video_st->codecpar->extradata) {
                if (pkt_len + packet.size + video_st->codecpar->extradata_size < pkt_len_max){
                    memcpy(pkt_buf+pkt_len, video_st->codecpar->extradata, video_st->codecpar->extradata_size);
                    pkt_len +=  video_st->codecpar->extradata_size;
                }
            }
        }

        if ((pkt_len + packet.size) < pkt_len_max) {
            memcpy(pkt_buf+pkt_len, packet.data, packet.size);
            pkt_len = pkt_len + packet.size;
        }

        if (picture) av_frame_free(&picture);
        picture = av_frame_alloc();

        // Decode video frame
#if USE_AV_SEND_FRAME_API
        if (avcodec_send_packet(context, &packet) < 0) {
            break;
        }
        ret = avcodec_receive_frame(context, picture);
#else
        int got_picture = 0;
        ret = avcodec_decode_video2(context, picture, &got_picture, &packet);
        ret = got_picture ? 0 : -1;
#endif
        if (ret >= 0) {
            valid = true;
        } else if (ret == AVERROR(EAGAIN)) {
            continue;
        }
        else
        {
            play_status = 2;
            std::cout << "avcodec decode maybe blocked, reconnect please!" << std::endl;

            if (++cur_decode_attempts > max_decode_attempts)
            {
                CV_LOG_WARNING(NULL,
                    "frame decode max attempts exceeded, try to increase attempt "
                    "limit by setting environment variable OPENCV_FFMPEG_DECODE_ATTEMPTS "
                    "(current value is " << max_decode_attempts << ")");
                break;
            }
        }
        if (ret) {
            if (round(sampler_cnt + sampler_step) == round(sampler_cnt))
                ret = 0;

            sampler_cnt += sampler_step;
        } else {
            count_errs++;
            continue;
        }
        if (ret) {
            picture_raw_pts = picture->pkt_dts;
            dump_write(&g_bm_callback_info[chl_id].debug_param, picture);
        }
    }

    if (valid) {
        if( picture_pts == AV_NOPTS_VALUE_ )
            picture_pts = picture->CV_FFMPEG_PTS_FIELD != AV_NOPTS_VALUE_ && picture->CV_FFMPEG_PTS_FIELD != 0 ? picture->CV_FFMPEG_PTS_FIELD : picture->pkt_dts;
        frame_number++;
    }

    if (!rawMode && valid && first_frame_number < 0)
        first_frame_number = dts_to_frame_number(picture_pts);

#if USE_AV_INTERRUPT_CALLBACK
    // deactivate interrupt callback
#ifdef WIN32
    WaitForSingleObject(interrupt_metadata.interrupt_lock, INFINITE);
    interrupt_metadata.timeout_after_ms = 0;
    ReleaseMutex(interrupt_metadata.interrupt_lock);
#else
    pthread_mutex_lock(&interrupt_metadata.interrupt_lock);
    interrupt_metadata.timeout_after_ms = 0;
    pthread_mutex_unlock(&interrupt_metadata.interrupt_lock);
#endif
#endif

    if ((buf != NULL) && (len_out != NULL)) {
        if(valid) {
            if (len_in > pkt_len) {
                memcpy(buf, pkt_buf, pkt_len);
                *len_out = pkt_len;
            } else {
                memcpy(buf, pkt_buf, len_in);
                *len_out = len_in;
            }
            memset(pkt_buf, 0, pkt_len_max);
            pkt_len = 0;
        }else{
            *len_out = 0;
        }
    }

    // return if we have a new frame or not
    return valid;
}

bool CvCapture_FFMPEG::retrieveFrame(int, cv::OutputArray image)
{
    if (!video_st) {
        return false;
    }

    if (!is_bm_video_codec(video_st->codecpar->codec_id) &&
        !is_bm_image_codec(video_st->codecpar->codec_id)) {
        return retrieveFrameSoft(0, image);
    }

    if ((out_yuv > 0) && ((picture->data == NULL) || (picture->data[4] == NULL) || (picture->buf == NULL) || (picture->buf[0] == NULL))) {
        return false;
    } else if ((out_yuv == 0) && ((picture->data == NULL) || (picture->data[0] == NULL) || (picture->buf == NULL) || (picture->buf[0] == NULL))) {
        return false;
    }

    if (frame.height != picture->height) frame.height = picture->height;
    if (frame.width != picture->width) frame.width = picture->width;

    int height = picture_height < 0 ? frame.height : picture_height;
    int width = picture_width < 0 ? frame.width : picture_width;

    if (frame.display_height != height) frame.display_height = height;
    if (frame.display_width != width) frame.display_width = width;

    cv::Mat& myimage = image.getMatRef();

#ifdef HAVE_BMCV
    /*  Hardcoded Fix: under compression mode, compressoin data use coded_width
     *  and coded_height. But avframe only has one set of w/h, that is display
     *  width and height. This is no problem if output is linear mode becasue we
     *  only take care valid display context. For compression mode, there is
     *  problem. We must use coded_width and height for compression raw data and
     *  show in display width and height. So we hardcode avframe width and
     *  height to coded set. [xun]*/
    picture->width = context->coded_width;
    picture->height = context->coded_height;
    cv::Mat comp(picture, card);

    /*  hardcoded Fix: mat.rows for display height, mat.cols for display width [xun]*/
    comp.cols = video_st->codecpar->width;
    comp.rows = video_st->codecpar->height;

    if (myimage.empty() || (out_yuv > 0 && !myimage.avOK()) ||
        (out_yuv > 0 && (myimage.avFormat() != AV_PIX_FMT_YUV420P)) ||
        !(myimage.u && myimage.u->addr) || (BM_CARD_ID(myimage.card) != BM_CARD_ID(card)) ||
        (myimage.rows != height) || (myimage.cols != width)){
        // create mat buffer
        if (out_yuv > 0){
            AVFrame *f = cv::av::create(height, width, card);
            myimage.create(f, card);
        } else {
            if (myimage.allocator && myimage.allocator != cv::hal::getAllocator()){
                myimage.release();
                myimage.allocator = cv::hal::getAllocator();
            }

            myimage.create(height, width, CV_8UC3, card);
        }
    }

    if (comp.avComp()) {
        cv::bmcv::decomp(comp, myimage, out_yuv > 0 ? false : true);
    } else if (out_yuv > 0) {
        image.assign(comp);
    } else {
        cv::bmcv::toMAT(comp, myimage);
    }

#else
    cv::Mat myframe(picture, card);

    if (out_yuv > 0) {
        image.assign(myframe);
    } else {
        if (myimage.empty() || !(myimage.u && myimage.u->addr) ||
            (myimage.rows != height) || (myimage.cols != width) ||
            (myimage.card != card)) {
            myimage.create(height, width, CV_8UC3, card);
        }

        cv::vpp::toMat(myframe, myimage);
    }
#endif

    picture = 0;
    return true;
}


bool CvCapture_FFMPEG::retrieveFrameSoft(int, cv::OutputArray image)
{
    if( !video_st || !picture->data[0] )
        return false;

    // if (rawMode || flag == extraDataIdx)
    // {
    //     bool ret = true;
    //     if (flag == 0) {
    //         AVPacket& p = bsfc ? packet_filtered : packet;
    //         *data = p.data;
    //         *step = p.size;
    //         ret = p.data != NULL;
    //     }
    //     else if (flag == extraDataIdx) {
    //         *data = ic->streams[video_stream]->CV_FFMPEG_CODEC_FIELD->extradata;
    //         *step = ic->streams[video_stream]->CV_FFMPEG_CODEC_FIELD->extradata_size;
    //     }
    //     *width = *step;
    //     *height = 1;
    //     *cn = 1;
    //     *depth = CV_8U;
    //     return  ret;
    // }

    AVFrame* sw_picture = picture;
#if USE_AV_HW_CODECS
    // if hardware frame, copy it to system memory
    if (picture && picture->hw_frames_ctx) {
        sw_picture = av_frame_alloc();
        //if (av_hwframe_map(sw_picture, picture, AV_HWFRAME_MAP_READ) < 0) {
        if (av_hwframe_transfer_data(sw_picture, picture, 0) < 0) {
            CV_LOG_ERROR(NULL, "Error copying data from GPU to CPU (av_hwframe_transfer_data)");
            return false;
        }
    }
#endif

    if (!sw_picture || !sw_picture->data[0])
        return false;

    CV_LOG_DEBUG(NULL, "Input picture format: " << av_get_pix_fmt_name((AVPixelFormat)sw_picture->format));
    // const AVPixelFormat result_format = convertRGB ? AV_PIX_FMT_BGR24 : (AVPixelFormat)sw_picture->format;
    // switch (result_format)
    // {
    // case AV_PIX_FMT_BGR24: *depth = CV_8U; *cn = 3; break;
    // case AV_PIX_FMT_GRAY8: *depth = CV_8U; *cn = 1; break;
    // case AV_PIX_FMT_GRAY16LE: *depth = CV_16U; *cn = 1; break;
    // default:
    //     CV_LOG_WARNING(NULL, "Unknown/unsupported picture format: " << av_get_pix_fmt_name(result_format)
    //                    << ", will be treated as 8UC1.");
    //     *depth = CV_8U;
    //     *cn = 1;
    //     break; // TODO: return false?
    // }


    /* According to retrieveFrame, output Mat buffer is seperate from internal decoding bufer. So
     * if Image.data is given by outside, we reuse it, otherwise we assign another Mat.data, and
     * then copyTo it */
    cv::Mat& myimage = image.getMatRef();
    int height = picture_height < 0 ? frame.height : picture_height;
    int width = picture_width < 0 ? frame.width : picture_width;
    AVPixelFormat format = out_yuv > 0 ? AV_PIX_FMT_YUV420P : AV_PIX_FMT_BGR24;

    int aligns[AV_NUM_DATA_POINTERS];
    int align_width = width;
    int align_height= height;
    avcodec_align_dimensions2(context, &align_width, &align_height, aligns);

    if( myimage.empty() || (out_yuv > 0 && !myimage.avOK())
     || !(myimage.u && myimage.u->addr)
     || (myimage.rows != height) || (myimage.cols != width)
     || (!out_yuv && ((myimage.step[0] < align_width*myimage.step[1])
         ||(myimage.u->size < myimage.step[0] * align_height))) )
    {
        if (out_yuv > 0)
        {
            AVFrame *f = cv::av::create(height, width, card);
            myimage.create(f, card);
        }else {
	        if (myimage.allocator && myimage.allocator != cv::hal::getAllocator()){
                myimage.release();
                myimage.allocator = cv::hal::getAllocator();
            }
            myimage.create(align_height, align_width, CV_8UC3, card);
            if((width < align_width) || (height < align_height)) {
                myimage = cv::Mat(myimage, cv::Rect(0, 0, width, height));
            }
        }
    }

    if( img_convert_ctx == NULL ||
        frame.width != video_st->codecpar->width ||
        frame.height != video_st->codecpar->height ||
        frame.display_width != width ||
        frame.display_height != height ||
        frame.format != format)
    {
        // Some sws_scale optimizations have some assumptions about alignment of data/step/width/height
        // Also we use coded_width/height to workaround problem with legacy ffmpeg versions (like n0.8)
        int buffer_width  = video_st->codecpar->width;
        int buffer_height = video_st->codecpar->height;
        width = picture_width < 0 ? video_st->codecpar->width : picture_width;
        height = picture_height < 0 ? video_st->codecpar->height : picture_height;

        img_convert_ctx = sws_getCachedContext(
                img_convert_ctx,
                buffer_width, buffer_height,
                (AVPixelFormat)sw_picture->format,
                width, height,
                format,
                SWS_BICUBIC,
                NULL, NULL, NULL
                );

        if (img_convert_ctx == NULL)
            return false;//CV_Error(0, "Cannot initialize the conversion context!");

        frame.width = video_st->codecpar->width;
        frame.height = video_st->codecpar->height;
        frame.display_width = width;
        frame.display_height = height;
        frame.format = format;
        frame.cn = 3;
    }

    if (out_yuv > 0)
        sws_scale(
                img_convert_ctx,
                picture->data,
                picture->linesize,
                0, video_st->codecpar->height,
                myimage.u->frame->data,
                myimage.u->frame->linesize
                );
    else{
        rgb_picture.data[0] = myimage.data;
        rgb_picture.linesize[0] = myimage.step[0];
        sws_scale(
                img_convert_ctx,
                picture->data,
                picture->linesize,
                0, video_st->codecpar->height,
                rgb_picture.data,
                rgb_picture.linesize
                );
        rgb_picture.data[0] = 0;
        rgb_picture.linesize[0] = 0;
    }
#ifdef USING_SOC
    myimage.allocator = cv::hal::getAllocator();
    myimage.allocator->flush(myimage.u, myimage.u->size);
#else
    cv::bmcv::uploadMat(myimage); // to keep sync up with hardware decoder, do upload here
#endif

    return true;
}

bool CvCapture_FFMPEG::retrieveHWFrame(cv::OutputArray output)
{
#if USE_AV_HW_CODECS
    // check that we have HW frame in GPU memory
    if (!picture || !picture->hw_frames_ctx || !context) {
        return false;
    }

    // GPU color conversion NV12->BGRA, from GPU media buffer to GPU OpenCL buffer
    return hw_copy_frame_to_umat(context->hw_device_ctx, picture, output);
#else
    CV_UNUSED(output);
    return false;
#endif
}

static inline double getCodecTag(const AVCodecID codec_id) {
    const struct AVCodecTag* fallback_tags[] = {
        // APIchanges:
        // 2012-01-31 - dd6d3b0 - lavf 54.01.0
        //   Add avformat_get_riff_video_tags() and avformat_get_riff_audio_tags().
        avformat_get_riff_video_tags(),
#if LIBAVFORMAT_BUILD >= CALC_FFMPEG_VERSION(55, 25, 100) && defined LIBAVFORMAT_VERSION_MICRO && LIBAVFORMAT_VERSION_MICRO >= 100
        // APIchanges: ffmpeg only
        // 2014-01-19 - 1a193c4 - lavf 55.25.100 - avformat.h
        //   Add avformat_get_mov_video_tags() and avformat_get_mov_audio_tags().
        avformat_get_mov_video_tags(),
#endif
        codec_bmp_tags, // fallback for avformat < 54.1
        NULL };
    return av_codec_get_tag(fallback_tags, codec_id);
}

static inline double getCodecIdFourcc(const AVCodecID codec_id)
{
    if (codec_id == AV_CODEC_ID_NONE) return -1;
    const char* codec_fourcc = _opencv_avcodec_get_name(codec_id);
    if (!codec_fourcc || strcmp(codec_fourcc, "unknown_codec") == 0 || strlen(codec_fourcc) != 4)
        return getCodecTag(codec_id);
    return (double)CV_FOURCC(codec_fourcc[0], codec_fourcc[1], codec_fourcc[2], codec_fourcc[3]);
}

double CvCapture_FFMPEG::getProperty( int property_id ) const
{
    if( !video_st || !context ) return 0;

    switch( property_id )
    {
#if USE_AV_INTERRUPT_CALLBACK
    case cv::CAP_PROP_OPEN_TIMEOUT_MSEC:
        return (double)open_timeout;
    case cv::CAP_PROP_READ_TIMEOUT_MSEC:
        return (double)read_timeout;
#endif
    case CAP_PROP_POS_MSEC:
        if (picture_pts == AV_NOPTS_VALUE_)
        {
            return 0;
        }
        return (dts_to_sec(picture_pts) * 1000);
    case CAP_PROP_POS_FRAMES:
        return (double)frame_number;
    case CAP_PROP_POS_AVI_RATIO:
        return r2d(ic->streams[video_stream]->time_base);
    case cv::CAP_PROP_POS_PTS:
        return (double)picture_pts;
    case CAP_PROP_FRAME_COUNT:
        return (double)get_total_frames();
    case CAP_PROP_FRAME_WIDTH:
        if (picture_width == -1) {
            return (double)((rotation_auto && ((rotation_angle%180) != 0)) ? frame.height : frame.width);
        } else {
            return (double)((rotation_auto && ((rotation_angle%180) != 0)) ? picture_height : picture_width);
        }
    case CAP_PROP_FRAME_HEIGHT:
        return (double)((rotation_auto && ((rotation_angle%180) != 0)) ? frame.width : frame.height);
    case CAP_PROP_FRAME_TYPE:
        return (double)av_get_picture_type_char(picture->pict_type);
    case CAP_PROP_FPS:
        return get_fps();
    case CAP_PROP_FOURCC: {
        const double fourcc = getCodecIdFourcc(video_st->CV_FFMPEG_CODEC_FIELD->codec_id);
        if (fourcc != -1) return fourcc;
        const double codec_tag = (double)video_st->CV_FFMPEG_CODEC_FIELD->codec_tag;
        if (codec_tag) return codec_tag;
        else return -1;
    }
    case CAP_PROP_SAR_NUM:
        return get_sample_aspect_ratio(ic->streams[video_stream]).num;
    case CAP_PROP_SAR_DEN:
        return get_sample_aspect_ratio(ic->streams[video_stream]).den;
    case CAP_PROP_CODEC_PIXEL_FORMAT:
    {
#ifdef CV_FFMPEG_CODECPAR
        AVPixelFormat pix_fmt = (AVPixelFormat)video_st->codecpar->format;
#else
        AVPixelFormat pix_fmt = video_st->codecpar->pix_fmt;
#endif
        unsigned int fourcc_tag = avcodec_pix_fmt_to_codec_tag(pix_fmt);
        return (fourcc_tag == 0) ? (double)-1 : (double)fourcc_tag;
    }
    case CAP_PROP_FORMAT:
        if (rawMode)
            return -1;
        break;
    case CAP_PROP_CONVERT_RGB:
        return convertRGB;
    case CAP_PROP_LRF_HAS_KEY_FRAME: {
        const AVPacket& p = bsfc ? packet_filtered : packet;
        return ((p.flags & AV_PKT_FLAG_KEY) != 0) ? 1 : 0;
    }
    case CAP_PROP_CODEC_EXTRADATA_INDEX:
            return extraDataIdx;
    case CAP_PROP_BITRATE:
        return static_cast<double>(get_bitrate());
    case CAP_PROP_ORIENTATION_META:
        return static_cast<double>(rotation_angle);
    case CAP_PROP_ORIENTATION_AUTO:
#if LIBAVUTIL_BUILD >= CALC_FFMPEG_VERSION(52, 94, 100)
        return static_cast<double>(rotation_auto);
#else
        return 0;
#endif
#if USE_AV_HW_CODECS
    case CAP_PROP_HW_ACCELERATION:
        return static_cast<double>(va_type);
    case CAP_PROP_HW_DEVICE:
        return static_cast<double>(hw_device);
    case CAP_PROP_HW_ACCELERATION_USE_OPENCL:
        return static_cast<double>(use_opencl);
#endif  // USE_AV_HW_CODECS
    case CAP_PROP_STREAM_OPEN_TIME_USEC:
        //ic->start_time_realtime is in microseconds
        return ((double)ic->start_time_realtime);
    case CAP_PROP_N_THREADS:
        return static_cast<double>(context->thread_count);
    case cv::CAP_PROP_STATUS:
        return (double)play_status;
    case cv::CAP_PROP_TIMESTAMP:
        return (double)picture_raw_pts;
    case cv::CAP_PROP_OUTPUT_YUV:
        return out_yuv;
    case cv::CAP_PROP_OUTPUT_SRC:
        return sampler_step;
    default:
        break;
    }

    return 0;
}

AVRational CvCapture_FFMPEG::get_sample_aspect_ratio(AVStream *stream) const
{
    AVRational undef = {0, 1};
    AVRational stream_sample_aspect_ratio = stream ? stream->sample_aspect_ratio : undef;
    AVRational frame_sample_aspect_ratio  = stream && stream->codecpar ? stream->codecpar->sample_aspect_ratio : undef;

    av_reduce(&stream_sample_aspect_ratio.num, &stream_sample_aspect_ratio.den,
        stream_sample_aspect_ratio.num,  stream_sample_aspect_ratio.den, INT_MAX);
    if (stream_sample_aspect_ratio.num <= 0 || stream_sample_aspect_ratio.den <= 0)
        stream_sample_aspect_ratio = undef;

    av_reduce(&frame_sample_aspect_ratio.num, &frame_sample_aspect_ratio.den,
        frame_sample_aspect_ratio.num,  frame_sample_aspect_ratio.den, INT_MAX);
    if (frame_sample_aspect_ratio.num <= 0 || frame_sample_aspect_ratio.den <= 0)
        frame_sample_aspect_ratio = undef;

    if (stream_sample_aspect_ratio.num)
        return stream_sample_aspect_ratio;
    else
        return frame_sample_aspect_ratio;
}

double CvCapture_FFMPEG::r2d(AVRational r) const
{
    return r.num == 0 || r.den == 0 ? 0. : (double)r.num / (double)r.den;
}

double CvCapture_FFMPEG::get_duration_sec() const
{
    double sec = (double)ic->duration / (double)AV_TIME_BASE;

    if (sec < eps_zero)
    {
        sec = (double)ic->streams[video_stream]->duration * r2d(ic->streams[video_stream]->time_base);
    }

    return sec;
}

int64_t CvCapture_FFMPEG::get_bitrate() const
{
    return ic->bit_rate / 1000;
}

double CvCapture_FFMPEG::get_fps() const
{
#if LIBAVFORMAT_BUILD >= CALC_FFMPEG_VERSION(55, 1, 100) && LIBAVFORMAT_VERSION_MICRO >= 100
    double fps = r2d(av_guess_frame_rate(ic, ic->streams[video_stream], NULL));
#else
    double fps = r2d(ic->streams[video_stream]->avg_frame_rate);

#if LIBAVFORMAT_BUILD >= CALC_FFMPEG_VERSION(52, 111, 0)
    if (fps < eps_zero)
    {
        fps = r2d(ic->streams[video_stream]->avg_frame_rate);
    }
#endif

    if (fps < eps_zero)
    {
        fps = 1.0 / r2d(ic->streams[video_stream]->time_base);
    }
#endif
    return fps;
}

int64_t CvCapture_FFMPEG::get_total_frames() const
{
    int64_t nbf = ic->streams[video_stream]->nb_frames;

    if (nbf == 0)
    {
        nbf = (int64_t)floor(get_duration_sec() * get_fps() + 0.5);
    }
    return nbf;
}

int64_t CvCapture_FFMPEG::dts_to_frame_number(int64_t dts)
{
    double sec = dts_to_sec(dts);
    return (int64_t)(get_fps() * sec + 0.5);
}

double CvCapture_FFMPEG::dts_to_sec(int64_t dts) const
{
    return (double)(dts - ic->streams[video_stream]->start_time) *
        r2d(ic->streams[video_stream]->time_base);
}

void CvCapture_FFMPEG::get_rotation_angle()
{
    rotation_angle = 0;
#if LIBAVFORMAT_BUILD >= CALC_FFMPEG_VERSION(57, 68, 100)
    const uint8_t *data = 0;
    data = av_stream_get_side_data(video_st, AV_PKT_DATA_DISPLAYMATRIX, NULL);
    if (data)
    {
        rotation_angle = -cvRound(av_display_rotation_get((const int32_t*)data));
        if (rotation_angle < 0)
            rotation_angle += 360;
    }
#elif LIBAVUTIL_BUILD >= CALC_FFMPEG_VERSION(52, 94, 100)
    AVDictionaryEntry *rotate_tag = av_dict_get(video_st->metadata, "rotate", NULL, 0);
    if (rotate_tag != NULL)
        rotation_angle = atoi(rotate_tag->value);
#endif
}

void CvCapture_FFMPEG::seek(int64_t _frame_number)
{
    CV_Assert(context);
    _frame_number = std::min(_frame_number, get_total_frames());
    int delta = 16;

    // if we have not grabbed a single frame before first seek, let's read the first frame
    // and get some valuable information during the process
    if( first_frame_number < 0 && get_total_frames() > 1 )
        grabFrame();

    for(;;)
    {
        int64_t _frame_number_temp = std::max(_frame_number-delta, (int64_t)0);
        double sec = (double)_frame_number_temp / get_fps();
        int64_t time_stamp = ic->streams[video_stream]->start_time;
        double  time_base  = r2d(ic->streams[video_stream]->time_base);
        time_stamp += (int64_t)(sec / time_base + 0.5);
        if (get_total_frames() > 1) {
#if SUPPORT_INPUT_THREADS
            input_fd.thread_pause_and_clean();
            input_fd.pkt_list_clean();
#endif
            av_seek_frame(ic, video_stream, time_stamp, AVSEEK_FLAG_BACKWARD);
#if SUPPORT_INPUT_THREADS
            input_fd.thread_resume();
#endif
        }
        //avcodec_flush_buffers(ic->streams[video_stream]->codec);
        if( _frame_number > 0 )
        {
            grabFrame();

            if( _frame_number > 1 )
            {
                frame_number = dts_to_frame_number(picture_pts) - first_frame_number;
                //printf("_frame_number = %d, frame_number = %d, delta = %d\n",
                //       (int)_frame_number, (int)frame_number, delta);

                if( frame_number < 0 || frame_number > _frame_number-1 )
                {
                    if( _frame_number_temp == 0 || delta >= INT_MAX/4 )
                        break;
                    delta = delta < 16 ? delta*2 : delta*3/2;
                    continue;
                }
                while( frame_number < _frame_number-1 )
                {
                    if(!grabFrame())
                        break;
                }
                frame_number++;
                break;
            }
            else
            {
                frame_number = 1;
                break;
            }
        }
        else
        {
            frame_number = 0;
            break;
        }
    }
}

void CvCapture_FFMPEG::seek(double sec)
{
    seek((int64_t)(sec * get_fps() + 0.5));
}

bool CvCapture_FFMPEG::setProperty( int property_id, double value )
{
    if( !video_st ) return false;

    switch( property_id )
    {
#if USE_AV_INTERRUPT_CALLBACK
    case cv::CAP_PROP_OPEN_TIMEOUT_MSEC:
        open_timeout = (int)value;
        break;
    case cv::CAP_PROP_READ_TIMEOUT_MSEC:
        read_timeout = (int)value;
        break;
#endif
    case CAP_PROP_POS_MSEC:
    case CAP_PROP_POS_FRAMES:
    case CAP_PROP_POS_AVI_RATIO:
    case cv::CAP_PROP_POS_PTS:
        {
            switch( property_id )
            {
            case CAP_PROP_POS_FRAMES:
                seek((int64_t)value);
                break;

            case CAP_PROP_POS_MSEC:
                seek(value/1000.0);
                break;

            case CAP_PROP_POS_AVI_RATIO:
                seek(value*(ic->duration/1000.0/1000.0));
                break;
            case cv::CAP_PROP_POS_PTS:
#if SUPPORT_INPUT_THREADS
                input_fd.thread_pause_and_clean();
                input_fd.pkt_list_clean();
#endif
                av_seek_frame(ic, video_stream, (int64_t )(value + 0.5), AVSEEK_FLAG_BACKWARD);
#if SUPPORT_INPUT_THREADS
                input_fd.thread_resume();
#endif
                break;
            }

            picture_pts=(int64_t)value;
        }
        break;
    case CAP_PROP_FORMAT:
        if (value == -1)
            return setRaw();
        return false;
    case CAP_PROP_CONVERT_RGB:
        convertRGB = (value != 0);
        return true;
    case CAP_PROP_ORIENTATION_AUTO:
#if LIBAVUTIL_BUILD >= CALC_FFMPEG_VERSION(52, 94, 100)
        rotation_auto = value != 0 ? true : false;
        return true;
#else
        rotation_auto = false;
        return false;
#endif
    case cv::CAP_PROP_OUTPUT_YUV:
        out_yuv = value;
        break;
    case cv::CAP_PROP_OUTPUT_SRC:
        sampler_step = value;
        break;
    case CV_FFMPEG_CAP_PROP_FRAME_WIDTH:
        picture_width = (int)value;
        break;
    case CV_FFMPEG_CAP_PROP_FRAME_HEIGHT:
        picture_height = (int)value;
        break;
    default:
        return false;
    }

    return true;
}

/* bm codec private function */
bool CvCapture_FFMPEG::is_bm_video_codec(int codec_id)
{
    //Support bm1684. In bm1684 only support H264 and H265, For performance optimize use the code.
    if ((codec_id == AV_CODEC_ID_H264)          ||
#ifdef VPP_BM1682
        (codec_id == AV_CODEC_ID_H263)          ||
        (codec_id == AV_CODEC_ID_MSMPEG4V3)     ||
        (codec_id == AV_CODEC_ID_MPEG1VIDEO)    ||
        (codec_id == AV_CODEC_ID_FLV1)          ||
        (codec_id == AV_CODEC_ID_MPEG2VIDEO)    ||
        (codec_id == AV_CODEC_ID_VC1)           ||
        (codec_id == AV_CODEC_ID_CAVS)          ||
        (codec_id == AV_CODEC_ID_AVS)           ||
        (codec_id == AV_CODEC_ID_VP8)           ||
        (codec_id == AV_CODEC_ID_VP3)           ||
        (codec_id == AV_CODEC_ID_MPEG4)         ||
        (codec_id == AV_CODEC_ID_WMV1)          ||
        (codec_id == AV_CODEC_ID_WMV2)          ||
        (codec_id == AV_CODEC_ID_WMV3)          ||
#endif
        (codec_id == AV_CODEC_ID_HEVC))
        return true;

    return false;
}

bool CvCapture_FFMPEG::set_resampler(int den, int num)
{
    sampler_r.den = den;
    sampler_r.num = num;
    sampler_step = r2d(sampler_r);

    return true;
}

bool CvCapture_FFMPEG::get_resampler(int *den, int *num)
{
    *den = sampler_r.den;
    *num = sampler_r.num;

    return true;
}

/* bm codec jpeg private function */
bool CvCapture_FFMPEG::is_bm_image_codec(int codec_id)
{
    if (codec_id == AV_CODEC_ID_MJPEG)
        return true;

    return false;
}

typedef struct {
    int       memFd;     /* Valid for soc mode */
    uint64_t  paddr;
    uint8_t*  vaddr;
    int       size;
    int       map_flags; /* Valid for soc mode */
    int       devFd;
    int       soc_idx;   /* Valid for pcie mode */
    /* Don't touch! */
    void*     context;
} bm_ion_context;

///////////////// FFMPEG CvVideoWriter implementation //////////////////////////
struct CvVideoWriter_FFMPEG
{
    bool open( const char* filename, int fourcc,
               double fps, int width, int height, const VideoWriterParameters& params,
               int id = 0, const char *encodeparms = "" );
    void close();
    bool writeFrame( const unsigned char* data, int step, int width, int height, int cn, int origin,
                     char *data_out=NULL, int *len=NULL,  void *roiinfo=NULL );
    bool writeFrame( cv::InputArray image, char *data=NULL, int *len=NULL, void *roiinfo = NULL);
    bool writeHWFrame(cv::InputArray input);
    double getProperty(int propId) const;

    void init(bool is_first_init);
    bool AddRoiInfo(AVFrame * picture,  void *roiinfo);
#ifdef HAVE_BMCV
    AVFrame* getIdleAvFrame(AVFrame *f_in);
    bool convertAndEncode(cv::Mat m_in, char *data, int *len, void *roiinfo);
    void releaseAvFrame();
    AVFrame * copyAvFrame(AVFrame *f_in);
#endif

    CV_FFMPEG_FMT_CONST AVOutputFormat  * fmt;
    AVFormatContext * oc;
    uint8_t         * outbuf;
    uint32_t          outbuf_size;
    FILE            * outfile;
    AVFrame         * picture;
    AVFrame         * input_picture;
    AVFrame         * dma_picture[DMA_LIST_MAX_NUMS];
    bm_device_mem_t   mem_info[DMA_LIST_MAX_NUMS*3];
    uint8_t         * picbuf;
    AVStream        * video_st;
    AVCodecContext  * context;
    AVPixelFormat     input_pix_fmt;
    unsigned char   * aligned_input;
    size_t            aligned_input_size;
    int               frame_width, frame_height;
    int               frame_idx;
    int               card_idx;
    bool              ok;
    struct SwsContext *img_convert_ctx;
    VideoAccelerationType va_type;
    int               hw_device;
    int               use_opencl;

    CV_CODEC_ID       m_codec_id;
    char              m_filename[256];
    int               m_fourcc;
    double            m_fps;
    int               m_width;
    int               m_height;
    bool              m_is_color;
    VideoWriterParameters*  m_writeparams = NULL;
    int               is_dma_buffer;
    int               is_fmt_nv12;
    bool              output_flags;

    int               encode_gop_size;
    int               roi_enable;
    char              encode_params[1024];
    char              m_encodeparms[1024];
    unsigned int      total_heap_num;
};

static const char * icvFFMPEGErrStr(int err)
{
    switch(err) {
    case AVERROR_BSF_NOT_FOUND:
        return "Bitstream filter not found";
    case AVERROR_DECODER_NOT_FOUND:
        return "Decoder not found";
    case AVERROR_DEMUXER_NOT_FOUND:
        return "Demuxer not found";
    case AVERROR_ENCODER_NOT_FOUND:
        return "Encoder not found";
    case AVERROR_EOF:
        return "End of file";
    case AVERROR_EXIT:
        return "Immediate exit was requested; the called function should not be restarted";
    case AVERROR_FILTER_NOT_FOUND:
        return "Filter not found";
    case AVERROR_INVALIDDATA:
        return "Invalid data found when processing input";
    case AVERROR_MUXER_NOT_FOUND:
        return "Muxer not found";
    case AVERROR_OPTION_NOT_FOUND:
        return "Option not found";
    case AVERROR_PATCHWELCOME:
        return "Not yet implemented in FFmpeg, patches welcome";
    case AVERROR_PROTOCOL_NOT_FOUND:
        return "Protocol not found";
    case AVERROR_STREAM_NOT_FOUND:
        return "Stream not found";
    default:
        break;
    }

    return "Unspecified error";
}

/* function internal to FFMPEG (libavformat/riff.c) to lookup codec id by fourcc tag*/
extern "C" {
    enum CV_CODEC_ID codec_get_bmp_id(unsigned int tag);
}

void CvVideoWriter_FFMPEG::init(bool is_first_init)
{
    fmt = 0;
    oc = 0;
    outbuf = 0;
    outbuf_size = 0;
    outfile = 0;
    picture = 0;
    input_picture = 0;
    picbuf = 0;
    video_st = 0;
    context = 0;
    input_pix_fmt = AV_PIX_FMT_NONE;
    aligned_input = NULL;
    aligned_input_size = 0;
    img_convert_ctx = 0;
    frame_width = frame_height = 0;
    frame_idx = 0;
    va_type = VIDEO_ACCELERATION_NONE;
    hw_device = -1;
    use_opencl = 0;
    ok = false;

    output_flags = false;
    if (is_first_init) {
        is_dma_buffer = 0;
        is_fmt_nv12 = 0;
    }

    for(int i=0;i<DMA_LIST_MAX_NUMS;i++)
        dma_picture[i] = NULL;

    encode_gop_size = -1;
    roi_enable = 0;
    memset(encode_params,0,sizeof(encode_params));
}

/**
 * the following function is a modified version of code
 * found in ffmpeg-0.4.9-pre1/output_example.c
 */
static AVFrame * icv_alloc_picture_FFMPEG(int pix_fmt, int width, int height, bool alloc)
{
    AVFrame * picture;
    uint8_t * picture_buf = 0;
    int size;

#if LIBAVCODEC_BUILD >= (LIBAVCODEC_VERSION_MICRO >= 100 \
    ? CALC_FFMPEG_VERSION(55, 45, 101) : CALC_FFMPEG_VERSION(55, 28, 1))
    picture = av_frame_alloc();
#else
    picture = avcodec_alloc_frame();
#endif
    if (!picture)
        return NULL;

    picture->format = pix_fmt;
    picture->width = width;
    picture->height = height;

    size = _opencv_ffmpeg_av_image_get_buffer_size( (AVPixelFormat) pix_fmt, width, height);
    if(alloc){
        picture_buf = (uint8_t *) malloc(size);
        if (!picture_buf)
        {
            av_free(picture);
            return NULL;
        }
        _opencv_ffmpeg_av_image_fill_arrays(picture, picture_buf,
                       (AVPixelFormat) pix_fmt, width, height);
    }

    return picture;
}

/* configure video stream */
static AVCodecContext * icv_configure_video_stream_FFMPEG(AVFormatContext *oc,
                                                   AVStream *st,
                                                   const AVCodec* codec,
                                                   int w, int h, int bitrate,
                                                   double fps, AVPixelFormat pixel_format, int fourcc,
                                                   int encode_gop_size, char *encode_params)
{
#ifdef CV_FFMPEG_CODECPAR
    AVCodecContext *c = avcodec_alloc_context3(codec);
#else
    AVCodecContext *c = st->codec;
#endif
    CV_Assert(c);

    int frame_rate, frame_rate_base;

    c->codec_id = codec->id;
    c->codec_type = AVMEDIA_TYPE_VIDEO;
    c->codec_tag = fourcc;

#ifndef CV_FFMPEG_CODECPAR
    // Set per-codec defaults
    CV_CODEC_ID c_id = c->codec_id;
    avcodec_get_context_defaults3(c, codec);
    // avcodec_get_context_defaults3 erases codec_id for some reason
    c->codec_id = c_id;
#endif

    /* put sample parameters */
    int64_t lbit_rate = (int64_t)bitrate;
    lbit_rate += (bitrate / 2);
    lbit_rate = std::min(lbit_rate, (int64_t)INT_MAX);
    c->bit_rate = lbit_rate;

    // took advice from
    // http://ffmpeg-users.933282.n4.nabble.com/warning-clipping-1-dct-coefficients-to-127-127-td934297.html
    c->qmin = 3;

    /* resolution must be a multiple of two */
    c->width = w;
    c->height = h;

    /* time base: this is the fundamental unit of time (in seconds) in terms
       of which frame timestamps are represented. for fixed-fps content,
       timebase should be 1/framerate and timestamp increments should be
       identically 1. */
    frame_rate=(int)(fps+0.5);
    frame_rate_base=1;
    while (fabs(((double)frame_rate/frame_rate_base) - fps) > 0.001){
        frame_rate_base*=10;
        frame_rate=(int)(fps*frame_rate_base + 0.5);
    }
    c->time_base.den = frame_rate;
    c->time_base.num = frame_rate_base;
    /* adjust time base for supported framerates */
    if(codec && codec->supported_framerates){
        const AVRational *p= codec->supported_framerates;
        AVRational req = {frame_rate, frame_rate_base};
        const AVRational *best=NULL;
        AVRational best_error= {INT_MAX, 1};
        for(; p->den!=0; p++){
            AVRational error= av_sub_q(req, *p);
            if(error.num <0) error.num *= -1;
            if(av_cmp_q(error, best_error) < 0){
                best_error= error;
                best= p;
            }
        }
        if (best == NULL)
        {
#ifdef CV_FFMPEG_CODECPAR
            avcodec_free_context(&c);
#endif
            return NULL;
        }
        c->time_base.den= best->num;
        c->time_base.num= best->den;
    }

    if (encode_gop_size > 0){
        c->gop_size = encode_gop_size;
    }else{
        c->gop_size = 12; /* emit one intra frame every twelve frames at most */
    }
    c->pix_fmt = pixel_format;
    if (c->codec_id == CV_CODEC(CODEC_ID_MPEG2VIDEO)) {
        c->max_b_frames = 2;
    }
    if (c->codec_id == CV_CODEC(CODEC_ID_MPEG1VIDEO) || c->codec_id == CV_CODEC(CODEC_ID_MSMPEG4V3)){
        /* needed to avoid using macroblocks in which some coeffs overflow
           this doesn't happen with normal video, it just happens here as the
           motion of the chroma plane doesn't match the luma plane */
        /* avoid FFMPEG warning 'clipping 1 dct coefficients...' */
        c->mb_decision=2;
    }

    /* Some settings for libx264 encoding, restore dummy values for gop_size
     and qmin since they will be set to reasonable defaults by the libx264
     preset system. Also, use a crf encode with the default quality rating,
     this seems easier than finding an appropriate default bitrate. */
    if (c->codec_id == AV_CODEC_ID_H264) {
      if (encode_gop_size > 0){
          c->gop_size = encode_gop_size;
      }else{
          c->gop_size = -1;
      }
      c->qmin = -1;
      c->bit_rate = 0;
      if (c->priv_data)
          av_opt_set(c->priv_data,"crf","23", 0);
    }


    if (strlen(encode_params) > 0){
        av_opt_set(c->priv_data,"enc-params",encode_params, 0);
    }
    if(oc->oformat != NULL){
        // some formats want stream headers to be separate
        if(oc->oformat->flags & AVFMT_GLOBALHEADER)
        {
            // flags were renamed: https://github.com/libav/libav/commit/7c6eb0a1b7bf1aac7f033a7ec6d8cacc3b5c2615
#if LIBAVCODEC_BUILD >= (LIBAVCODEC_VERSION_MICRO >= 100 \
     ? CALC_FFMPEG_VERSION(56, 60, 100) : CALC_FFMPEG_VERSION(56, 35, 0))
            c->flags |= AV_CODEC_FLAG_GLOBAL_HEADER;
#else
            c->flags |= CODEC_FLAG_GLOBAL_HEADER;
#endif
        }
    }

#ifdef WIN32
    st->avg_frame_rate = {frame_rate, frame_rate_base};
#else
    st->avg_frame_rate = av_make_q(frame_rate, frame_rate_base);
#endif
#if LIBAVFORMAT_BUILD >= CALC_FFMPEG_VERSION(55, 20, 0)
    st->time_base = c->time_base;
#endif

    return c;
}

static const int OPENCV_NO_FRAMES_WRITTEN_CODE = 1000;

static int icv_av_write_frame_FFMPEG( AVFormatContext * oc, AVStream * video_st, AVCodecContext * c,
                                      uint8_t *, uint32_t,
                                      AVFrame * picture, int frame_idx,
                                      char *data, int *len, bool output_flags)
{
    int ret = OPENCV_NO_FRAMES_WRITTEN_CODE;

#if LIBAVFORMAT_BUILD < CALC_FFMPEG_VERSION(57, 0, 0)
    if (oc->oformat->flags & AVFMT_RAWPICTURE)
    {
        /* raw video case. The API will change slightly in the near
           futur for that */
        AVPacket pkt;
        av_init_packet(&pkt);

        pkt.flags |= PKT_FLAG_KEY;
        pkt.stream_index= video_st->index;
        pkt.data= (uint8_t *)picture;
        pkt.size= sizeof(AVPicture);

        if(output_flags) {
            if (data != NULL) {
                *len = pkt.size;
                memcpy(data,pkt.data,pkt.size);
            }
        }else{
            ret = av_write_frame(oc, &pkt);
        }
    }
    else
#endif
    {
        /* encode the image */
#if USE_AV_SEND_FRAME_API
        if (picture == NULL && frame_idx == 0) {
            ret = 0;
        } else {
            ret = avcodec_send_frame(c, picture);
            if (ret < 0)
                CV_LOG_ERROR(NULL, "Error sending frame to encoder (avcodec_send_frame)");
        }
        while (ret >= 0)
        {
            AVPacket* pkt = av_packet_alloc();
            pkt->stream_index = video_st->index;
            ret = avcodec_receive_packet(c, pkt);

            if(!ret)
            {
                av_packet_rescale_ts(pkt, c->time_base, video_st->time_base);
                if(output_flags) {
                    if (data != NULL) {
                        *len = pkt->size;
                        memcpy(data,pkt->data,pkt->size);
                    }
                }else{
                    ret = av_write_frame(oc, pkt);
                }
                av_packet_free(&pkt);
                continue;
            }

            av_packet_free(&pkt);
            break;
        }
#else
        CV_UNUSED(frame_idx);
        AVPacket pkt;
        av_init_packet(&pkt);
        int got_output = 0;
        pkt.data = NULL;
        pkt.size = 0;
        ret = avcodec_encode_video2(c, &pkt, picture, &got_output);
        if (ret < 0)
            ;
        else if (got_output) {
            if (pkt.pts != (int64_t)AV_NOPTS_VALUE)
                pkt.pts = av_rescale_q(pkt.pts, c->time_base, video_st->time_base);
            if (pkt.dts != (int64_t)AV_NOPTS_VALUE)
                pkt.dts = av_rescale_q(pkt.dts, c->time_base, video_st->time_base);
            if (pkt.duration)
                pkt.duration = av_rescale_q(pkt.duration, c->time_base, video_st->time_base);
            pkt.stream_index= video_st->index;
            if(output_flags) {
                if (data != NULL) {
                    *len = pkt.size;
                    memcpy(data,pkt.data,pkt.size);
                }
            }else{
                ret = av_write_frame(oc, &pkt);
            }
            // ret = av_write_frame(oc, &pkt);
            _opencv_ffmpeg_av_packet_unref(&pkt);
        }
        else
            ret = OPENCV_NO_FRAMES_WRITTEN_CODE;
#endif
    }
    return ret;
}

/// write a frame with FFMPEG
bool CvVideoWriter_FFMPEG::writeFrame( const unsigned char* data, int step, int width, int height, int cn, int origin, char *data_out, int *len, void *roiinfo )
{
    // check parameters
    if (input_pix_fmt == AV_PIX_FMT_BGR24) {
        if (cn != 3) {
            return false;
        }
    }
    else if (input_pix_fmt == AV_PIX_FMT_GRAY8 || input_pix_fmt == AV_PIX_FMT_GRAY16LE) {
        if (cn != 1) {
            return false;
        }
    }
    else {
        CV_LOG_WARNING(NULL, "Input data does not match selected pixel format: "
                       << av_get_pix_fmt_name(input_pix_fmt)
                       << ", number of channels: " << cn);
        CV_Assert(false);
    }

    if( (width & -2) != frame_width || (height & -2) != frame_height || !data )
        return false;
    width = frame_width;
    height = frame_height;

    // FFmpeg contains SIMD optimizations which can sometimes read data past
    // the supplied input buffer.
    // Related info: https://trac.ffmpeg.org/ticket/6763
    // 1. To ensure that doesn't happen, we pad the step to a multiple of 32
    // (that's the minimal alignment for which Valgrind doesn't raise any warnings).
    // 2. (dataend - SIMD_SIZE) and (dataend + SIMD_SIZE) is from the same 4k page
    const int CV_STEP_ALIGNMENT = 32;
    const size_t CV_SIMD_SIZE = 32;
    const size_t CV_PAGE_MASK = ~(size_t)(4096 - 1);
    const uchar* dataend = data + ((size_t)height * step);
    if (step % CV_STEP_ALIGNMENT != 0 ||
        (((size_t)dataend - CV_SIMD_SIZE) & CV_PAGE_MASK) != (((size_t)dataend + CV_SIMD_SIZE) & CV_PAGE_MASK))
    {
        int aligned_step = (step + CV_STEP_ALIGNMENT - 1) & ~(CV_STEP_ALIGNMENT - 1);

        size_t new_size = (aligned_step * height + CV_SIMD_SIZE);

        if (!aligned_input || aligned_input_size < new_size)
        {
            if (aligned_input)
                av_freep(&aligned_input);
            aligned_input_size = new_size;
            aligned_input = (unsigned char*)av_mallocz(aligned_input_size);
        }

        if (origin == 1)
            for( int y = 0; y < height; y++ )
                memcpy(aligned_input + y*aligned_step, data + (height-1-y)*step, step);
        else
            for( int y = 0; y < height; y++ )
                memcpy(aligned_input + y*aligned_step, data + y*step, step);

        data = aligned_input;
        step = aligned_step;
    }

    AVPixelFormat sw_pix_fmt = context->pix_fmt;
#if USE_AV_HW_CODECS
    if (context->hw_frames_ctx)
        sw_pix_fmt = ((AVHWFramesContext*)context->hw_frames_ctx->data)->sw_format;
#endif
    if ( sw_pix_fmt != input_pix_fmt ) {
        CV_Assert( input_picture );
        // let input_picture point to the raw data buffer of 'image'
        _opencv_ffmpeg_av_image_fill_arrays(input_picture, (uint8_t *) data,
                       (AVPixelFormat)input_pix_fmt, width, height);
        input_picture->linesize[0] = step;

        if( !img_convert_ctx )
        {
            img_convert_ctx = sws_getContext(width,
                                             height,
                                             (AVPixelFormat)input_pix_fmt,
                                             context->width,
                                             context->height,
                                             sw_pix_fmt,
                                             SWS_BICUBIC,
                                             NULL, NULL, NULL);
            if( !img_convert_ctx )
                return false;
        }

        if ( sws_scale(img_convert_ctx, input_picture->data,
                       input_picture->linesize, 0,
                       height,
                       picture->data, picture->linesize) < 0 )
            return false;
    }
    else{
        _opencv_ffmpeg_av_image_fill_arrays(picture, (uint8_t *) data,
                       (AVPixelFormat)input_pix_fmt, width, height);
        picture->linesize[0] = step;
    }

    bool ret;
#if USE_AV_HW_CODECS
    if (context->hw_device_ctx) {
        // copy data to HW frame
        AVFrame* hw_frame = av_frame_alloc();
        if (!hw_frame) {
            CV_LOG_ERROR(NULL, "Error allocating AVFrame (av_frame_alloc)");
            return false;
        }
        if (av_hwframe_get_buffer(context->hw_frames_ctx, hw_frame, 0) < 0) {
            CV_LOG_ERROR(NULL, "Error obtaining HW frame (av_hwframe_get_buffer)");
            av_frame_free(&hw_frame);
            return false;
        }
        if (av_hwframe_transfer_data(hw_frame, picture, 0) < 0) {
            CV_LOG_ERROR(NULL, "Error copying data from CPU to GPU (av_hwframe_transfer_data)");
            av_frame_free(&hw_frame);
            return false;
        }
        hw_frame->pts = frame_idx;
        int ret_write = icv_av_write_frame_FFMPEG(oc, video_st, context, outbuf, outbuf_size, hw_frame, frame_idx, data_out, len, output_flags);
        ret = ret_write >= 0 ? true : false;
        av_frame_free(&hw_frame);
    } else
#endif
    {
        picture->pts = frame_idx;

        if (roiinfo != NULL) {
            AddRoiInfo(picture, roiinfo);
        }

        int ret_write = icv_av_write_frame_FFMPEG(oc, video_st, context, outbuf, outbuf_size, picture, frame_idx, data_out, len, output_flags);
        ret = ret_write >= 0 ? true : false;
    }

    frame_idx++;

    return ret;
}

bool CvVideoWriter_FFMPEG::AddRoiInfo(AVFrame * picture,  void *roi_in) {
    if (roi_in == NULL) {
        return false;
    }

    AVFrameSideData *fside = av_frame_get_side_data(picture, AV_FRAME_DATA_BM_ROI_INFO);
    if (fside == NULL) {
        fside = av_frame_new_side_data(picture, AV_FRAME_DATA_BM_ROI_INFO, sizeof(AVBMRoiInfo));
        if (fside == NULL) {
            fprintf(stderr, "call av_frame_new_side_data failed.\n");
            return false;
        }
    }
    fside = av_frame_get_side_data(picture, AV_FRAME_DATA_BM_ROI_INFO);
    cv::CV_RoiInfo  *roi = (cv::CV_RoiInfo*)roi_in;
    AVBMRoiInfo *roiinfo = (AVBMRoiInfo*)fside->data;
    memset(roiinfo, 0, sizeof(AVBMRoiInfo));

    if (m_codec_id == AV_CODEC_ID_H264) {
        roiinfo->customRoiMapEnable  = roi->customRoiMapEnable;
        roiinfo->customModeMapEnable = roi->customModeMapEnable;
        for (int i=0;i<roi->numbers;i++) {
            roiinfo->field[i].H264.mb_qp         =    roi->field[i].H264.mb_qp;
            roiinfo->field[i].H264.mb_force_mode =    roi->field[i].H264.mb_force_mode;
        }
    } else if (m_codec_id == AV_CODEC_ID_H265) {
        roiinfo->customRoiMapEnable    = roi->customRoiMapEnable;
        roiinfo->customModeMapEnable   = roi->customModeMapEnable;
        roiinfo->customLambdaMapEnable = roi->customLambdaMapEnable;
        roiinfo->customCoefDropEnable  = roi->customCoefDropEnable;

        for (int i=0;i<roi->numbers;i++) {
            roiinfo->field[i].HEVC.ctu_force_mode = roi->field[i].HEVC.ctu_force_mode;
            roiinfo->field[i].HEVC.ctu_coeff_drop = roi->field[i].HEVC.ctu_coeff_drop;
            roiinfo->field[i].HEVC.sub_ctu_qp_0   = roi->field[i].HEVC.sub_ctu_qp_0;
            roiinfo->field[i].HEVC.sub_ctu_qp_1   = roi->field[i].HEVC.sub_ctu_qp_1;
            roiinfo->field[i].HEVC.sub_ctu_qp_2   = roi->field[i].HEVC.sub_ctu_qp_2;
            roiinfo->field[i].HEVC.sub_ctu_qp_3   = roi->field[i].HEVC.sub_ctu_qp_3;
            roiinfo->field[i].HEVC.lambda_sad_0   = roi->field[i].HEVC.lambda_sad_0;
            roiinfo->field[i].HEVC.lambda_sad_1   = roi->field[i].HEVC.lambda_sad_1;
            roiinfo->field[i].HEVC.lambda_sad_2   = roi->field[i].HEVC.lambda_sad_2;
            roiinfo->field[i].HEVC.lambda_sad_3   = roi->field[i].HEVC.lambda_sad_3;
        }
    }
    return true;
}



bool CvVideoWriter_FFMPEG::writeHWFrame(cv::InputArray input) {
#if USE_AV_HW_CODECS
    if (!video_st || !context || !context->hw_frames_ctx || !context->hw_device_ctx)
        return false;

    // Get hardware frame from frame pool
    AVFrame* hw_frame = av_frame_alloc();
    if (!hw_frame) {
        return false;
    }
    if (av_hwframe_get_buffer(context->hw_frames_ctx, hw_frame, 0) < 0) {
        av_frame_free(&hw_frame);
        return false;
    }

    // GPU to GPU copy
    if (!hw_copy_umat_to_frame(context->hw_device_ctx, input, hw_frame)) {
        av_frame_free(&hw_frame);
        return false;
    }

    // encode
    hw_frame->pts = frame_idx;
    icv_av_write_frame_FFMPEG( oc, video_st, context, outbuf, outbuf_size, hw_frame, frame_idx, NULL, NULL, output_flags);
    frame_idx++;

    av_frame_free(&hw_frame);

    return true;
#else
    CV_UNUSED(input);
    return false;
#endif
}

bool CvVideoWriter_FFMPEG::writeFrame( cv::InputArray image, char *data, int *len , void *roiinfo)
{
    bool ret = false;
    cv::Mat m_in = image.getMat();

    if (m_in.avOK()) {
#ifdef HAVE_BMCV
        if ((m_in.avFormat() == AV_PIX_FMT_NV12) || (m_in.avFormat() == AV_PIX_FMT_YUV420P)){
            bool reopen = false;
            if((m_in.avFormat() == AV_PIX_FMT_NV12) && (is_fmt_nv12 != 0xFF)){
                is_fmt_nv12 = 0xFF;
                reopen = true;
            }

            if(is_dma_buffer != 1){
                is_dma_buffer = 1;
                reopen = true;
            }

            if(reopen){
                open(m_filename, m_fourcc, m_fps, m_width, m_height, *m_writeparams, BM_CARD_ID(card_idx), m_encodeparms);
            }

            AVFrame *f_in = m_in.u->frame;

            unsigned int top_addr_offset = (bm_uint64)f_in->data[4]>>32&0xf;
            if(top_addr_offset == bmvpu_get_ext_addr())
            {
                f_in->pts = frame_idx;
                if (roiinfo != NULL) {
                    AddRoiInfo(f_in, roiinfo);
                }
                ret = icv_av_write_frame_FFMPEG( oc, video_st, context, outbuf, outbuf_size, f_in, frame_idx, data, len, output_flags) >= 0;
                frame_idx++;
            }else{
                AVFrame *f = copyAvFrame(f_in);
                if (roiinfo != NULL) {
                    AddRoiInfo(f, roiinfo);
                }
                ret = icv_av_write_frame_FFMPEG( oc, video_st, context, outbuf, outbuf_size, f, frame_idx, data, len, output_flags) >= 0;
            }
        }else if(m_in.avFormat() == AV_PIX_FMT_NV16 || m_in.avFormat() == AV_PIX_FMT_YUV422P){
            if(is_dma_buffer != 1) {
                is_dma_buffer = 1;
                open(m_filename, m_fourcc, m_fps, m_width, m_height, *m_writeparams, BM_CARD_ID(card_idx), m_encodeparms);
            }
            cv::Mat myimage;
            cv::bmcv::toMAT(m_in, myimage, false);
            ret = convertAndEncode(myimage,data,len, roiinfo);
        }else {
            if(is_dma_buffer != 1) {
                is_dma_buffer = 1;
                open(m_filename, m_fourcc, m_fps, m_width, m_height, *m_writeparams, BM_CARD_ID(card_idx), m_encodeparms);
            }
            cv::Mat myimage;
            cv::Rect rt0(0, 0, m_in.cols, m_in.rows);
            std::vector<cv::Rect> vrt= {rt0};
            cv::Size sz0(frame_width, frame_height);
            std::vector<cv::Size> vsz= {sz0};
            std::vector<cv::Mat> out;
            out.push_back(myimage);
            csc_type_t csc = CSC_YPbPr2RGB_BT601;

            CV_Assert(BM_SUCCESS == cv::bmcv::convert(m_in, vrt, vsz, out, false, \
                        csc, NULL, BMCV_INTER_NEAREST));
            ret = convertAndEncode(myimage, data, len, roiinfo);
        }
#endif
    }else if((m_codec_id == AV_CODEC_ID_H265) || (m_codec_id == AV_CODEC_ID_H264)){
#ifdef HAVE_BMCV
        if(is_dma_buffer != 1) {
            is_dma_buffer = 1;
            open(m_filename, m_fourcc, m_fps, m_width, m_height, *m_writeparams, BM_CARD_ID(card_idx), m_encodeparms);
        }

        cv::Mat m_enc;
        if ((m_in.u == 0) || (m_in.u->addr == 0)){
#ifdef USING_SOC
            m_in.copyTo(m_enc);
#else
            if (m_in.allocator && m_in.allocator != cv::hal::getAllocator())
                m_in.copyTo(m_enc);
            else{
                m_enc = m_in;
                cv::bmcv::attachDeviceMemory(m_enc);
            }
#endif
        }else
            m_enc = m_in;
        cv::bmcv::uploadMat(m_enc);
        ret = convertAndEncode(m_enc, data, len, roiinfo);
#else
        if(is_dma_buffer != 0) {
            is_dma_buffer = 0;
            open(m_filename, m_fourcc, m_fps, m_width, m_height, *m_writeparams, BM_CARD_ID(card_idx), m_encodeparms);
        }
        ret = writeFrame((const uchar*)image.getMat().ptr(), (int)image.step(), image.cols(), image.rows(), image.channels(), 0, data, len, roiinfo);
#endif
    }else{
        if(is_dma_buffer != 0) {
            is_dma_buffer = 0;
            open(m_filename, m_fourcc, m_fps, m_width, m_height, *m_writeparams, BM_CARD_ID(card_idx), m_encodeparms);
        }
        ret = writeFrame((const uchar*)image.getMat().ptr(), (int)image.step(), image.cols(), image.rows(), image.channels(), 0, data, len, roiinfo);
    }

    return ret;
}

#ifdef HAVE_BMCV
bool CvVideoWriter_FFMPEG::convertAndEncode( cv::Mat m_in, char *data, int *len, void *roiinfo )
{
    bool ret = false;
    cv::Mat myimage;
    unsigned int id = BM_MAKEFLAG(0, HEAP2_MASK, BM_CARD_ID(card_idx));
    AVFrame *f = cv::av::create(frame_height, frame_width, id);
    myimage.create(f, id);

    cv::Rect rt0(0, 0, m_in.cols, m_in.rows);
    std::vector<cv::Rect> vrt= {rt0};

    cv::Size sz0(frame_width, frame_height);
    std::vector<cv::Size> vsz= {sz0};

    std::vector<cv::Mat> out;
    out.push_back(myimage);
    cv::bmcv::convert(m_in, vrt, vsz, out,true,CSC_RGB2YCbCr_BT601, nullptr, BMCV_INTER_NEAREST);

    unsigned int top_addr_offset = (bm_uint64)f->data[4]>>32&0xf;
    if(top_addr_offset == bmvpu_get_ext_addr())
    {
        f->pts = frame_idx;
        if (roiinfo != NULL) {
            AddRoiInfo(f, roiinfo);
        }
        ret = icv_av_write_frame_FFMPEG( oc, video_st, context, outbuf, outbuf_size, f, frame_idx, data, len, output_flags) >= 0;
        frame_idx++;
    }else{
        AVFrame *f_out = copyAvFrame(f);
        if (roiinfo != NULL) {
            AddRoiInfo(f_out, roiinfo);
        }
        ret = icv_av_write_frame_FFMPEG( oc, video_st, context, outbuf, outbuf_size, f_out, frame_idx, data, len, output_flags) >= 0;
    }

    return ret;
}

void CvVideoWriter_FFMPEG::releaseAvFrame()
{
    bm_handle_t mem_handle = cv::bmcv::getCard(card_idx);
    for(int i=0;i<DMA_LIST_MAX_NUMS;i++){
        if(dma_picture[i] == NULL){
            continue;
        }


        bm_free_device(mem_handle, mem_info[3*i]);
        bm_free_device(mem_handle, mem_info[3*i+1]);
        if(dma_picture[i]->format == AV_PIX_FMT_YUV420P){
            bm_free_device(mem_handle, mem_info[3*i+2]);
        }

        av_frame_unref(dma_picture[i]);
        av_frame_free(&dma_picture[i]);
    }
    return;
}

AVFrame* CvVideoWriter_FFMPEG::getIdleAvFrame(AVFrame *f_in)
{
    int ret = 0;
    int heap_id = -1;
    if(f_in == NULL){
        return NULL;
    }

    for (int i=total_heap_num-1; i>=0; i--) {
        if (MALLOC_HEAP_ID & (0x1<<i)) {
            heap_id = i;
            break;
        }
        if (i == 0) {
            CV_Error(CV_StsError, "can not find heap_id!");
            return NULL;
        }
    }
    bm_handle_t mem_handle = cv::bmcv::getCard(card_idx);
    for(int i =0;i< DMA_LIST_MAX_NUMS;i++){
        if(dma_picture[i] == NULL){
            dma_picture[i] = av_frame_alloc();
            dma_picture[i]->format = f_in->format;
            dma_picture[i]->height = f_in->height;
            dma_picture[i]->width  = f_in->width;

            for(int j =0;j<8;j++){
                dma_picture[i]->linesize[j] = f_in->linesize[j];
            }

            dma_picture[i]->buf[0] = av_buffer_create(NULL,0,NULL,NULL,AV_BUFFER_FLAG_READONLY);

            ret = bm_malloc_device_byte_heap(mem_handle, &mem_info[3*i] ,heap_id, f_in->linesize[4]*f_in->height);
            if (ret != 0) {
                av_buffer_unref(&(dma_picture[i]->buf[0]));
                return NULL;
            }
            unsigned long long vmemy;
            ret = bm_mem_mmap_device_mem_no_cache(mem_handle, &mem_info[3*i], &vmemy);
            if (ret != BM_SUCCESS) {
                CV_Error(CV_StsError, "bm_mem_mmap_device_mem_no_cache failed\n");
                return NULL;
            }
            memset((void*)vmemy, 0, mem_info[3*i].size);
            bm_mem_unmap_device_mem(mem_handle, (void*)vmemy, mem_info[3*i].size);

            dma_picture[i]->data[4] = (uint8_t *)bm_mem_get_device_addr(mem_info[3*i]);

            ret = bm_malloc_device_byte_heap(mem_handle, &mem_info[3*i+1],heap_id, f_in->linesize[5]*f_in->height / 2);
            if (ret != 0) {
                av_buffer_unref(&(dma_picture[i]->buf[0]));
                bm_free_device(mem_handle, mem_info[3*i]);
                return NULL;
            }
            unsigned long long vmemu;
            ret = bm_mem_mmap_device_mem_no_cache(mem_handle, &mem_info[3*i+1], &vmemu);
            if (ret != BM_SUCCESS) {
                CV_Error(CV_StsError, "bm_mem_mmap_device_mem_no_cache failed\n");
                return NULL;
            }
            memset((void*)vmemu, 0, mem_info[3*i+1].size);
            bm_mem_unmap_device_mem(mem_handle, (void*)vmemu, mem_info[3*i+1].size);
            dma_picture[i]->data[5] = (uint8_t *)bm_mem_get_device_addr(mem_info[3*i+1]);

            if(f_in->format == AV_PIX_FMT_YUV420P) {
                ret = bm_malloc_device_byte_heap(mem_handle, &mem_info[3*i+2], heap_id, f_in->linesize[6]*f_in->height / 2);
                if (ret != 0) {
                    av_buffer_unref(&(dma_picture[i]->buf[0]));
                    bm_free_device(mem_handle, mem_info[3*i]);
                    bm_free_device(mem_handle, mem_info[3*i+1]);
                    return NULL;
                }
                unsigned long long vmemv;
                ret = bm_mem_mmap_device_mem_no_cache(mem_handle, &mem_info[3*i+2], &vmemv);
                if (ret != BM_SUCCESS) {
                    CV_Error(CV_StsError, "bm_mem_mmap_device_mem_no_cache V failed\n");
                    return NULL;
                }
                memset((void*)vmemv, 0, mem_info[3*i+2].size);
                bm_mem_unmap_device_mem(mem_handle, (void*)vmemv, mem_info[3*i+2].size);
                dma_picture[i]->data[6] =(uint8_t *) bm_mem_get_device_addr(mem_info[3*i+2]);
            }

            return dma_picture[i];
        }else{
            if(av_buffer_get_ref_count(dma_picture[i]->buf[0]) < 2){
                return dma_picture[i];
            }
        }
    }

    return NULL;
}

AVFrame* CvVideoWriter_FFMPEG::copyAvFrame(AVFrame *f_in)
{
    if(f_in == NULL){
        return NULL;
    }

    AVFrame * f = getIdleAvFrame(f_in);
    if(f == NULL){
        return NULL;
    }

    uint len0 = f_in->linesize[4]*f_in->height;
    uint len1 = f_in->linesize[5]*f_in->height/2;
    uint len2 = 0;
    if(f_in->format == AV_PIX_FMT_YUV420P) {
        len2 = f_in->linesize[6]*f_in->height/2;
    }

    bm_handle_t handle = cv::bmcv::getCard(card_idx);
    bm_device_mem_t mem_src[3];
    memset(mem_src, 0, sizeof(mem_src));
    bm_device_mem_t mem_dst[3];
    memset(mem_dst, 0, sizeof(mem_dst));

    mem_src[0] = bm_mem_from_device((bm_uint64)f_in->data[4], len0);
    mem_src[1] = bm_mem_from_device((bm_uint64)f_in->data[5], len1);
    if(f_in->format == AV_PIX_FMT_YUV420P) {
        mem_src[2] = bm_mem_from_device((bm_uint64)f_in->data[6], len2);
    }

    mem_dst[0] = bm_mem_from_device((bm_uint64)f->data[4], len0);
    mem_dst[1] = bm_mem_from_device((bm_uint64)f->data[5], len1);
    if(f_in->format == AV_PIX_FMT_YUV420P) {
        mem_dst[2] = bm_mem_from_device((bm_uint64)f->data[6], len2);
    }

    bm_memcpy_d2d_byte(handle, mem_dst[0], 0, mem_src[0], 0, len0);
    bm_memcpy_d2d_byte(handle, mem_dst[1], 0, mem_src[1], 0, len1);
    if(f_in->format == AV_PIX_FMT_YUV420P) {
        bm_memcpy_d2d_byte(handle, mem_dst[2], 0, mem_src[2], 0, len2);
    }

    f->pts = frame_idx;
    frame_idx++;
    return f;
}
#endif


double CvVideoWriter_FFMPEG::getProperty(int propId) const
{
    CV_UNUSED(propId);
#if USE_AV_HW_CODECS
    if (propId == VIDEOWRITER_PROP_HW_ACCELERATION)
    {
        return static_cast<double>(va_type);
    }
    else if (propId == VIDEOWRITER_PROP_HW_DEVICE)
    {
        return static_cast<double>(hw_device);
    }
    else if (propId == VIDEOWRITER_PROP_HW_ACCELERATION_USE_OPENCL)
    {
        return static_cast<double>(use_opencl);
    }
#endif
    return 0;
}

/// close video output stream and free associated memory
void CvVideoWriter_FFMPEG::close()
{
    /* no more frame to compress. The codec has a latency of a few
       frames if using B frames, so we get the last frames by
       passing the same picture again */
    // TODO -- do we need to account for latency here?

    /* write the trailer, if any */
    if (picture && ok && oc && oc->oformat)
    {
#if LIBAVFORMAT_BUILD < CALC_FFMPEG_VERSION(57, 0, 0)
        if (!(oc->oformat->flags & AVFMT_RAWPICTURE))
#endif
        {
            for(;;)
            {
                int ret = icv_av_write_frame_FFMPEG( oc, video_st, context, outbuf, outbuf_size, NULL, frame_idx, NULL, NULL, output_flags);
                if( ret == OPENCV_NO_FRAMES_WRITTEN_CODE || ret < 0 )
                    break;
            }
        }
        av_write_trailer(oc);
    }

    if( img_convert_ctx )
    {
        sws_freeContext(img_convert_ctx);
        img_convert_ctx = 0;
    }

    // free pictures
    if (picture && context && context->pix_fmt != input_pix_fmt)
    {
        if(picture->data[0])
            free(picture->data[0]);
        picture->data[0] = 0;
    }
    av_free(picture);

    if (input_picture)
        av_free(input_picture);

#ifdef CV_FFMPEG_CODECPAR
    avcodec_free_context(&context);
#else
    /* close codec */
    if (context)  // fixed after https://github.com/FFmpeg/FFmpeg/commit/3e1f507f3e8f16b716aa115552d243b48ae809bd
        avcodec_close(context);
    context = NULL;
#endif

    av_free(outbuf);

    if (oc)
    {
        if ((fmt) && (!(fmt->flags & AVFMT_NOFILE)))
        {
            /* close the output file */
            avio_close(oc->pb);
        }

        /* free the stream */
        avformat_free_context(oc);
    }

    av_freep(&aligned_input);

#ifdef HAVE_BMCV
    releaseAvFrame();
#endif

    init(false);
}

#define CV_PRINTABLE_CHAR(ch) ((ch) < 32 ? '?' : (ch))
#define CV_TAG_TO_PRINTABLE_CHAR4(tag) CV_PRINTABLE_CHAR((tag) & 255), CV_PRINTABLE_CHAR(((tag) >> 8) & 255), CV_PRINTABLE_CHAR(((tag) >> 16) & 255), CV_PRINTABLE_CHAR(((tag) >> 24) & 255)

static inline bool cv_ff_codec_tag_match(const AVCodecTag *tags, CV_CODEC_ID id, unsigned int tag)
{
    while (tags->id != AV_CODEC_ID_NONE)
    {
        if (tags->id == id && tags->tag == tag)
            return true;
        tags++;
    }
    return false;
}

static inline bool cv_ff_codec_tag_list_match(const AVCodecTag *const *tags, CV_CODEC_ID id, unsigned int tag)
{
    int i;
    for (i = 0; tags && tags[i]; i++) {
        bool res = cv_ff_codec_tag_match(tags[i], id, tag);
        if (res)
            return res;
    }
    return false;
}

static inline void cv_ff_codec_tag_dump(const AVCodecTag *const *tags)
{
    int i;
    for (i = 0; tags && tags[i]; i++) {
        const AVCodecTag * ptags = tags[i];
        while (ptags->id != AV_CODEC_ID_NONE)
        {
            unsigned int tag = ptags->tag;
            printf("fourcc tag 0x%08x/'%c%c%c%c' codec_id %04X\n", tag, CV_TAG_TO_PRINTABLE_CHAR4(tag), ptags->id);
            ptags++;
        }
    }
}

static void bm_find_encoder_name(int enc_id, std::string &enc_name)
{
    switch (enc_id)
    {
        case AV_CODEC_ID_H264:       enc_name = "h264_bm";    break;
        case AV_CODEC_ID_H265:       enc_name = "h265_bm";    break;
        default:                     enc_name = "";           break;
    }
}

/// Create a video writer object that uses FFMPEG
bool CvVideoWriter_FFMPEG::open( const char * filename, int fourcc,
                                 double fps, int width, int height, const VideoWriterParameters& params,
                                 int id, const char *encodeparms)
{
    const bool threadSafe = isThreadSafe();
    // InternalFFMpegRegister::init(threadSafe);

    std::unique_lock<cv::Mutex> lock(_mutex_writer, std::defer_lock);
    if (!threadSafe)
        lock.lock();

    // CV_CODEC_ID codec_id = CV_CODEC(CODEC_ID_NONE);
    AVPixelFormat codec_pix_fmt;
    double bitrate_scale = 1;

    m_codec_id = CV_CODEC(CODEC_ID_NONE);
    strcpy(m_filename,filename);
    m_fourcc = fourcc;
    m_fps = fps;
    m_width = width;
    m_height= height;
    // m_is_color = is_color;

    card_idx = id;
    strcpy(m_encodeparms,encodeparms);
    close();

    const bool is_color = params.get(VIDEOWRITER_PROP_IS_COLOR, true);
    const int depth = params.get(VIDEOWRITER_PROP_DEPTH, CV_8U);
    const bool is_supported = depth == CV_8U || (depth == CV_16U && !is_color);
    if (!is_supported)
    {
        CV_LOG_WARNING(NULL, "Unsupported depth/isColor combination is selected, "
                             "only CV_8UC1/CV_8UC3/CV_16UC1 are supported.");
        return false;
    }
    if (params.has(VIDEOWRITER_PROP_HW_ACCELERATION))
    {
        va_type = params.get<VideoAccelerationType>(VIDEOWRITER_PROP_HW_ACCELERATION, VIDEO_ACCELERATION_NONE);
#if !USE_AV_HW_CODECS
        if (va_type != VIDEO_ACCELERATION_NONE && va_type != VIDEO_ACCELERATION_ANY)
        {
            CV_LOG_ERROR(NULL, "VIDEOIO/FFMPEG: FFmpeg backend is build without acceleration support. Can't handle VIDEOWRITER_PROP_HW_ACCELERATION parameter. Bailout");
            return false;
        }
#endif
    }
    if (params.has(VIDEOWRITER_PROP_HW_DEVICE))
    {
        hw_device = params.get<int>(VIDEOWRITER_PROP_HW_DEVICE, -1);
        if (va_type == VIDEO_ACCELERATION_NONE && hw_device != -1)
        {
            CV_LOG_ERROR(NULL, "VIDEOIO/FFMPEG: Invalid usage of VIDEOWRITER_PROP_HW_DEVICE without requested H/W acceleration. Bailout");
            return false;
        }
        if (va_type == VIDEO_ACCELERATION_ANY && hw_device != -1)
        {
            CV_LOG_ERROR(NULL, "VIDEOIO/FFMPEG: Invalid usage of VIDEOWRITER_PROP_HW_DEVICE with 'ANY' H/W acceleration. Bailout");
            return false;
        }
    }
    if (params.has(VIDEOWRITER_PROP_HW_ACCELERATION_USE_OPENCL)) {
        use_opencl = params.get<int>(VIDEOWRITER_PROP_HW_ACCELERATION_USE_OPENCL);
    }

    if (params.warnUnusedParameters())
    {
        CV_LOG_ERROR(NULL, "VIDEOIO/FFMPEG: unsupported parameters in VideoWriter, see logger INFO channel for details");
        return false;
    }

    char temp[1024] = {0};
    strcpy(temp,m_encodeparms);
    char *p = strtok(temp, ":");
    while(p){
        if(strncmp(p,"gop=",4) == 0){
            p += 4;
            encode_gop_size = atoi(p);
        } else if(strncmp(p,"roi_enable=",11) == 0){
            p += 11;
            roi_enable = atoi(p);
        } else {
            if(strlen(encode_params)>0){
                strcat(encode_params, ":");
            }
            strcat(encode_params, p);
        }
        p = strtok(NULL, ":");
    }

    // check arguments
    if( !filename ) {
        return false;
    }
    if(fps <= 0) {
        return false;
    }

    if(strlen(filename) <= 0){
        output_flags = true;
    }else{
        output_flags = false;
    }

    // we allow frames of odd width or height, but in this case we truncate
    // the rightmost column/the bottom row. Probably, this should be handled more elegantly,
    // but some internal functions inside FFMPEG swscale require even width/height.
    width &= -2;
    height &= -2;
    if( width <= 0 || height <= 0 ) {
        return false;
    }

    /* auto detect the output format from the name and fourcc code. */

    if(!output_flags && (strncmp(filename,"rtmp",4) != 0) && (strncmp(filename,"rtsp",4) != 0)){
        fmt = av_guess_format(NULL, filename, NULL);

        if (!fmt) {
            return false;
        }
    }else if(strncmp(filename,"rtmp",4) == 0){
        fmt = av_guess_format("flv", NULL, NULL);
    }else if(strncmp(filename,"rtsp",4) == 0){
        fmt = av_guess_format("rtsp", NULL, NULL);
    }

    /* determine optimal pixel format */
    if (is_color)
    {
        switch (depth)
        {
        case CV_8U: input_pix_fmt = AV_PIX_FMT_BGR24; break;
        default:
            CV_LOG_WARNING(NULL, "Unsupported input depth for color image: " << depth);
            return false;
        }
    }
    else
    {
        switch (depth)
        {
        case CV_8U: input_pix_fmt = AV_PIX_FMT_GRAY8; break;
        case CV_16U: input_pix_fmt = AV_PIX_FMT_GRAY16LE; break;
        default:
            CV_LOG_WARNING(NULL, "Unsupported input depth for grayscale image: " << depth);
            return false;
        }
    }
    CV_LOG_DEBUG(NULL, "Selected pixel format: " << av_get_pix_fmt_name(input_pix_fmt));

    if (fourcc == -1)
    {
        fprintf(stderr,"OpenCV: FFMPEG: format %s / %s\n", fmt->name, fmt->long_name);
        cv_ff_codec_tag_dump(fmt->codec_tag);
        return false;
    }

    /* Lookup codec_id for given fourcc */
    if(!output_flags){
        if( (m_codec_id = av_codec_get_id(fmt->codec_tag, fourcc)) == CV_CODEC(CODEC_ID_NONE) )
        {
            const struct AVCodecTag * fallback_tags[] = {
// APIchanges:
// 2012-01-31 - dd6d3b0 - lavf 54.01.0
//   Add avformat_get_riff_video_tags() and avformat_get_riff_audio_tags().
                avformat_get_riff_video_tags(),
#if LIBAVFORMAT_BUILD >= CALC_FFMPEG_VERSION(55, 25, 100) && defined LIBAVFORMAT_VERSION_MICRO && LIBAVFORMAT_VERSION_MICRO >= 100
// APIchanges: ffmpeg only
// 2014-01-19 - 1a193c4 - lavf 55.25.100 - avformat.h
//   Add avformat_get_mov_video_tags() and avformat_get_mov_audio_tags().
                avformat_get_mov_video_tags(),
#endif
                codec_bmp_tags, // fallback for avformat < 54.1
                NULL };
            if (m_codec_id == CV_CODEC(CODEC_ID_NONE)) {
                m_codec_id = av_codec_get_id(fallback_tags, fourcc);
            }
            if (m_codec_id == CV_CODEC(CODEC_ID_NONE)) {
                char *p = (char *) &fourcc;
                char name[] = {(char)tolower(p[0]), (char)tolower(p[1]), (char)tolower(p[2]), (char)tolower(p[3]), 0};
                const AVCodecDescriptor *desc = avcodec_descriptor_get_by_name(name);
                if (desc)
                    m_codec_id = desc->id;
            }

            if (m_codec_id == CV_CODEC(CODEC_ID_NONE))
            {
                fflush(stdout);
                fprintf(stderr, "OpenCV: FFMPEG: tag 0x%08x/'%c%c%c%c' is not found (format '%s / %s')'\n",
                        fourcc, CV_TAG_TO_PRINTABLE_CHAR4(fourcc),
                        fmt->name, fmt->long_name);
                return false;
            }
        }


        // validate tag
        if (cv_ff_codec_tag_list_match(fmt->codec_tag, m_codec_id, fourcc) == false)
        {
            fflush(stdout);
            fprintf(stderr, "OpenCV: FFMPEG: tag 0x%08x/'%c%c%c%c' is not supported with codec id %d and format '%s / %s'\n",
                    fourcc, CV_TAG_TO_PRINTABLE_CHAR4(fourcc),
                    m_codec_id, fmt->name, fmt->long_name);
            int supported_tag;
            if( (supported_tag = av_codec_get_tag(fmt->codec_tag, m_codec_id)) != 0 )
            {
                fprintf(stderr, "OpenCV: FFMPEG: fallback to use tag 0x%08x/'%c%c%c%c'\n",
                        supported_tag, CV_TAG_TO_PRINTABLE_CHAR4(supported_tag));
                fourcc = supported_tag;
            }
        }
    }else{
        const struct AVCodecTag *tableriff[] = { avformat_get_riff_video_tags(), 0 };
        m_codec_id = av_codec_get_id(tableriff, fourcc);
        if(m_codec_id == AV_CODEC_ID_NONE){
           const struct AVCodecTag *tablemov[] = { avformat_get_mov_video_tags(), 0 };
           m_codec_id = av_codec_get_id(tablemov, fourcc);
        }
        if(m_codec_id == AV_CODEC_ID_NONE){
            fflush(stdout);
            fprintf(stderr, "OpenCV: FFMPEG: tag 0x%08x/'%c%c%c%c' is not found\n",
                    fourcc, CV_TAG_TO_PRINTABLE_CHAR4(fourcc));
            return false;
        }
    }

    // alloc memory for context
    oc = avformat_alloc_context();
    CV_Assert(oc);

    if(!output_flags){
        /* set file name */
        oc->oformat = fmt;
#ifndef CV_FFMPEG_URL
        snprintf(oc->filename, sizeof(oc->filename), "%s", filename);
#else
    }
    size_t name_len = strlen(filename);
    oc->url = (char*)av_malloc(name_len + 1);
    CV_Assert(oc->url);
    memcpy((void*)oc->url, filename, name_len + 1);
    oc->url[name_len] = '\0';
#endif
    /* set some options */
    oc->max_delay = (int)(0.7*AV_TIME_BASE);  /* This reduces buffer underrun warnings with MPEG */

    // set a few optimal pixel formats for lossless codecs of interest..
    switch (m_codec_id) {
    case CV_CODEC(CODEC_ID_JPEGLS):
        // BGR24 or GRAY8 depending on is_color...
        // supported: bgr24 rgb24 gray gray16le
        // as of version 3.4.1
        codec_pix_fmt = input_pix_fmt;
        break;
    case CV_CODEC(CODEC_ID_HUFFYUV):
        // supported: yuv422p rgb24 bgra
        // as of version 3.4.1
        codec_pix_fmt = AV_PIX_FMT_YUV422P;
        break;
    case CV_CODEC(CODEC_ID_MJPEG):
    case CV_CODEC(CODEC_ID_LJPEG):
        codec_pix_fmt = AV_PIX_FMT_YUVJ420P;
        bitrate_scale = 3;
        break;
    case CV_CODEC(CODEC_ID_RAWVIDEO):
        codec_pix_fmt = input_pix_fmt == AV_PIX_FMT_GRAY8 ||
                        input_pix_fmt == AV_PIX_FMT_GRAY16LE ||
                        input_pix_fmt == AV_PIX_FMT_GRAY16BE ? input_pix_fmt : AV_PIX_FMT_YUV420P;
        break;
    default:
        // good for lossy formats, MPEG, etc.
        codec_pix_fmt = AV_PIX_FMT_YUV420P;
        break;
    }

    if((codec_pix_fmt == AV_PIX_FMT_YUV420P) && (is_fmt_nv12 == 0xFF)){
        codec_pix_fmt = AV_PIX_FMT_NV12;
    }

    double bitrate = std::min(bitrate_scale*fps*width*height, (double)INT_MAX/2);

    if (m_codec_id == AV_CODEC_ID_NONE) {
        m_codec_id = av_guess_codec(oc->oformat, NULL, filename, NULL, AVMEDIA_TYPE_VIDEO);
    }

    // Add video stream to output file
    video_st = avformat_new_stream(oc, 0);
    if (!video_st) {
        CV_WARN("Could not allocate stream");
        return false;
    }

    AVDictionary *dict = NULL;
#if !defined(NO_GETENV) && (LIBAVUTIL_VERSION_MAJOR >= 53)
    char* options = getenv("OPENCV_FFMPEG_WRITER_OPTIONS");
    if (options)
    {
        CV_LOG_DEBUG(NULL, "VIDEOIO/FFMPEG: using writer options from environment: " << options);
        av_dict_parse_string(&dict, options, ";", "|", 0);
    }
#endif

    // find and open encoder, try HW acceleration types specified in 'hw_acceleration' list (in order)
    int err = -1;
    const AVCodec* codec = NULL;
    std::string bm_enc_name = "";
#if USE_AV_HW_CODECS
    AVBufferRef* hw_device_ctx = NULL;
    HWAccelIterator accel_iter(va_type, true/*isEncoder*/, dict);
    while (accel_iter.good())
    {
#else
    do {
#endif
#if USE_AV_HW_CODECS
        accel_iter.parse_next();
        AVHWDeviceType hw_type = accel_iter.hw_type();
        codec = NULL;
        AVPixelFormat hw_format = AV_PIX_FMT_NONE;
        if (hw_device_ctx)
            av_buffer_unref(&hw_device_ctx);
        if (hw_type != AV_HWDEVICE_TYPE_NONE)
        {
            codec = hw_find_codec(m_codec_id, hw_type, av_codec_is_encoder, accel_iter.disabled_codecs().c_str(), &hw_format);
            if (!codec)
                continue;

            hw_device_ctx = hw_create_device(hw_type, hw_device, accel_iter.device_subname(), use_opencl != 0);
            if (!hw_device_ctx)
                continue;
        }
        else if (hw_type == AV_HWDEVICE_TYPE_NONE)
#endif
        bm_find_encoder_name(m_codec_id, bm_enc_name);
        if (bm_enc_name.empty())
            codec = avcodec_find_encoder(m_codec_id);
            // codec = avcodec_find_encoder(c->codec_id);
        else
        {
            codec = avcodec_find_encoder_by_name(bm_enc_name.c_str());
            /* if HW encoder not found try SW */
            if (!codec)
            {
                codec = avcodec_find_encoder(m_codec_id);
                // codec = avcodec_find_encoder(c->codec_id);
            }
        }

        if (!codec) {
            CV_LOG_ERROR(NULL, "Could not find encoder for codec_id=" << (int)m_codec_id << ", error: "
                    << icvFFMPEGErrStr(AVERROR_ENCODER_NOT_FOUND));
        }

        if (!codec)
            continue;
#if USE_AV_HW_CODECS
        AVPixelFormat format = (hw_format != AV_PIX_FMT_NONE) ? hw_format : codec_pix_fmt;
#else
        AVPixelFormat format = codec_pix_fmt;
#endif

#ifdef CV_FFMPEG_CODECPAR
        avcodec_free_context(&context);
#endif
        context = icv_configure_video_stream_FFMPEG(oc, video_st, codec,
                                              width, height, (int) (bitrate + 0.5),
                                              fps, format, fourcc,
                                              encode_gop_size, encode_params);
        if (!context)
        {
            continue;
        }

#if 0
#if FF_API_DUMP_FORMAT
        dump_format(oc, 0, filename, 1);
#else
        av_dump_format(oc, 0, filename, 1);
#endif
#endif

#if USE_AV_HW_CODECS
        if (hw_device_ctx) {
            context->hw_device_ctx = av_buffer_ref(hw_device_ctx);
            if (hw_format != AV_PIX_FMT_NONE) {
                context->hw_frames_ctx = hw_create_frames(NULL, hw_device_ctx, width, height, hw_format);
                if (!context->hw_frames_ctx)
                    continue;
            }
        }
#endif

        int64_t lbit_rate = (int64_t) context->bit_rate;
        lbit_rate += (int64_t)(bitrate / 2);
        lbit_rate = std::min(lbit_rate, (int64_t) INT_MAX);
        context->bit_rate_tolerance = (int) lbit_rate;
        context->bit_rate = (int) lbit_rate;

        AVDictionary *dict = NULL;
        if (!bm_enc_name.empty()) {
            /* Use system memory TODO */
            if((is_dma_buffer == 0) || (is_dma_buffer == 1))
            {
                av_dict_set_int(&dict, "is_dma_buffer", is_dma_buffer, 0);
            }else{
                av_dict_set_int(&dict, "is_dma_buffer", 1, 0);
                is_dma_buffer = 1;
            }
            av_dict_set_int(&dict, "roi_enable", roi_enable, 0);
#ifndef USING_SOC // PCIE Mode
            av_dict_set_int(&dict, "sophon_idx", BM_CARD_ID(card_idx), 0);
#endif
        }

        /* open the codec */
        err = avcodec_open2(context, codec, &dict);
        if (err >= 0) {
#if USE_AV_HW_CODECS
            va_type = hw_type_to_va_type(hw_type);
            if (hw_type != AV_HWDEVICE_TYPE_NONE && hw_device < 0)
                hw_device = 0;
#endif
            break;
        } else {
            CV_LOG_ERROR(NULL, "Could not open codec " << codec->name << ", error: " << icvFFMPEGErrStr(err) << " (" << err << ")");
        }
#if USE_AV_HW_CODECS
    }  // while (accel_iter.good())
#else
    } while (0);
#endif

#if USE_AV_HW_CODECS
    if (hw_device_ctx)
        av_buffer_unref(&hw_device_ctx);
#endif

    if (dict != NULL)
        av_dict_free(&dict);

    if (err < 0) {
        CV_LOG_ERROR(NULL, "VIDEOIO/FFMPEG: Failed to initialize VideoWriter");
        return false;
    }

#ifdef CV_FFMPEG_CODECPAR
    // Copy all to codecpar...
    // !!! https://stackoverflow.com/questions/15897849/c-ffmpeg-not-writing-avcc-box-information
    avcodec_parameters_from_context(video_st->codecpar, context);
#endif

    outbuf = NULL;


#if LIBAVFORMAT_BUILD < CALC_FFMPEG_VERSION(57, 0, 0)
    if (!(oc->oformat->flags & AVFMT_RAWPICTURE))
#endif
    {
        /* allocate output buffer */
        /* assume we will never get codec output with more than 4 bytes per pixel... */
        outbuf_size = width*height*4;
        outbuf = (uint8_t *) av_malloc(outbuf_size);
    }

    bool need_color_convert;
    AVPixelFormat sw_pix_fmt = context->pix_fmt;
#if USE_AV_HW_CODECS
    if (context->hw_frames_ctx)
        sw_pix_fmt = ((AVHWFramesContext*)context->hw_frames_ctx->data)->sw_format;
#endif

    need_color_convert = (sw_pix_fmt != input_pix_fmt);

    /* allocate the encoded raw picture */
    picture = icv_alloc_picture_FFMPEG(sw_pix_fmt, context->width, context->height, need_color_convert);
    if (!picture) {
        return false;
    }

    /* if the output format is not our input format, then a temporary
   picture of the input format is needed too. It is then converted
   to the required output format */
    input_picture = NULL;
    if ( need_color_convert ) {
        input_picture = icv_alloc_picture_FFMPEG(input_pix_fmt, context->width, context->height, false);
        if (!input_picture) {
            return false;
        }
    }

    if(!output_flags) {
        /* open the output file, if needed */
        if (!(fmt->flags & AVFMT_NOFILE))
        {
            if (avio_open(&oc->pb, filename, AVIO_FLAG_WRITE) < 0)
            {
                return false;
            }
        }

        /* write the stream header, if any */
        av_dict_set(&dict, "rtsp_transport", "tcp", 0);
        av_dict_set(&dict, "stimeout", "5000000", 0);

        err=avformat_write_header(oc, &dict);
        if (dict != NULL)
            av_dict_free(&dict);

        if(err < 0)
        {
            close();
            remove(filename);
            return false;
        }
    }
    frame_width = width;
    frame_height = height;
    frame_idx = 0;

    bm_handle_t mem_handle;
    int ret = bm_dev_request(&mem_handle, card_idx);
    if (ret != BM_SUCCESS) {
      CV_Error(CV_StsError, "Request Bm_handle Failed\n");
      return false;
    }
    ret = bm_get_gmem_total_heap_num(mem_handle, &total_heap_num);
    if (ret != 0) {
        CV_Error(CV_StsError, "bm_get_gmem_total_heap_num failed!");
        return false;
    }
    bm_dev_free(mem_handle);

    ok = true;

    return true;
}



static
CvCapture_FFMPEG* cvCreateFileCaptureWithParams_FFMPEG(const char* filename, const VideoCaptureParameters& params, int id)
{
    // FIXIT: remove unsafe malloc() approach
    CvCapture_FFMPEG* capture = (CvCapture_FFMPEG*)malloc(sizeof(*capture));
    if (!capture) {
        return 0;
    }
    capture->init();
    if (capture->open(filename, params, id)) {
        return capture;
    }

    capture->close();
    free(capture);
    return 0;
}

void cvReleaseCapture_FFMPEG(CvCapture_FFMPEG** capture)
{
    if( capture && *capture )
    {
        (*capture)->close();
        free(*capture);
        *capture = 0;
    }
}

int cvSetCaptureProperty_FFMPEG(CvCapture_FFMPEG* capture, int prop_id, double value)
{
    return capture->setProperty(prop_id, value);
}

double cvGetCaptureProperty_FFMPEG(CvCapture_FFMPEG* capture, int prop_id)
{
    return capture->getProperty(prop_id);
}

int cvGrabFrame_FFMPEG(CvCapture_FFMPEG* capture, char *buf, unsigned int len_in, unsigned int *len_out)
{
    return capture->grabFrame(buf, len_in, len_out);
}

int cvRetrieveFrame_FFMPEG(CvCapture_FFMPEG* capture, cv::OutputArray frame)
{
    return capture->retrieveFrame(0, frame);
}

int cvSetResampler_FFMPEG(CvCapture_FFMPEG* capture, int den, int num )
{
    if (den <= 0  || num <= 0)
        return false;

    return capture->set_resampler(den, num);
}

int cvGetResampler_FFMPEG(CvCapture_FFMPEG* capture, int *den, int *num )
{
    if (den == NULL || num == NULL)
        return false;

    return capture->get_resampler(den, num);
}

int cvRetrieveFrame2_FFMPEG(CvCapture_FFMPEG* capture, cv::OutputArray frame)
// int cvRetrieveFrame2_FFMPEG(CvCapture_FFMPEG* capture, unsigned char** data, int* step, int* width, int* height, int* cn, int* depth)
{
    return capture->retrieveFrame(0, frame);
}

static CvVideoWriter_FFMPEG* cvCreateVideoWriterWithParams_FFMPEG( const char* filename, int fourcc, double fps,
                                                  int width, int height, const VideoWriterParameters& params,
                                                  int id, const char* encodeParams )
{
    CvVideoWriter_FFMPEG* writer = (CvVideoWriter_FFMPEG*)malloc(sizeof(*writer));
    if (!writer)
        return 0;
    std::vector<int> m_intparams = params.getIntVector();
    writer->m_writeparams = new VideoWriterParameters(m_intparams);
    writer->init(true);
    if( writer->open( filename, fourcc, fps, width, height, params, id, encodeParams ))
        return writer;
    writer->close();
    free(writer);
    return 0;
}

CvVideoWriter_FFMPEG* cvCreateVideoWriter_FFMPEG( const char* filename, int fourcc, double fps,
                                                  int width, int height, int isColor,
                                                  int id, const char* encodeParams )
{
    VideoWriterParameters params;
    params.add(VIDEOWRITER_PROP_IS_COLOR, isColor);
    return cvCreateVideoWriterWithParams_FFMPEG(filename, fourcc, fps, width, height, params, 0, "");
}

void cvReleaseVideoWriter_FFMPEG( CvVideoWriter_FFMPEG** writer )
{
    if( writer && *writer )
    {
        if((*writer)->m_writeparams)
            delete (*writer)->m_writeparams;
        (*writer)->close();
        free(*writer);
        *writer = 0;
    }
}


int cvWriteFrame_FFMPEG( CvVideoWriter_FFMPEG* writer,
                         const unsigned char* data, int step,
                         int width, int height, int cn, int origin)
{
    return writer->writeFrame(data, step, width, height, cn, origin, NULL, NULL, NULL);
}
int cvWriteFrameByHd_FFMPEG( CvVideoWriter_FFMPEG* writer, cv::InputArray image )
{
    return writer->writeFrame(image,NULL,NULL);
}

int cvWriteFrameByHdOutbuf_FFMPEG( CvVideoWriter_FFMPEG* writer, cv::InputArray image,char *data, int *len )
{
    return writer->writeFrame(image,data,len);
}

int cvWriteFrameByHdOutbufRoi_FFMPEG( CvVideoWriter_FFMPEG* writer, cv::InputArray image,char *data, int *len, void *roiinfo)
{
    return writer->writeFrame(image, data, len, roiinfo);
}

