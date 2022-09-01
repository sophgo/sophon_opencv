/*
 * Copyright (c) 2019 Solan Shang
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

/**
 * Show how to use the libavformat/libavcodec API to demux and decode to process jpeg data,
 * Show how to use the libyuv API to convert yuv data to bgr data.
 */

#define __STDC_CONSTANT_MACROS
extern "C" {
#include <libavformat/avformat.h>
#include <libavutil/imgutils.h>
}
#include <opencv2/core.hpp>
#include <iostream>
#ifdef HAVE_LIBYUV
#include <libyuv.h>
#endif
#include <sys/time.h>

using namespace std;

//#define PROFILING_COLOR_CVT


/* Enable or disable frame reference counting. You are not supposed to support
 * both paths in your application but pick the one most appropriate to your
 * needs. Look for the use of refcount in this example to see what are the
 * differences of API usage between them. */
static int refcount = 0;

static int process_frame(cv::Mat* out, AVFrame* frame)
{
    if (!frame || !out)
        return -1;


    uint8_t* src_y = frame->data[0];
    uint8_t* src_u = frame->data[1];
    uint8_t* src_v = frame->data[2];
    int src_stride_y = frame->linesize[0];
    int src_stride_u = frame->linesize[1];
    int src_stride_v = frame->linesize[2];
    int width  = frame->width;
    int height = frame->height;

    uint8_t* dst_rgb24   = out->data;
    int dst_stride_rgb24 = out->step[0];
#if defined(PROFILING_COLOR_CVT)
    struct timeval start, end;
#endif

#if defined(PROFILING_COLOR_CVT)
    gettimeofday(&start, NULL);
#endif

    /* YUV to BGR */
    /* For YUV to RGB, try libyuv::xxxxToRAW */
    switch (frame->format)
    {
#ifdef HAVE_LIBYUV
    case AV_PIX_FMT_YUVJ420P:
        libyuv::J420ToRGB24(src_y, src_stride_y,
                            src_u, src_stride_u,
                            src_v, src_stride_v,
                            dst_rgb24, dst_stride_rgb24,
                            width, height);
        break;
    case AV_PIX_FMT_YUVJ422P:
        libyuv::J422ToRGB24(src_y, src_stride_y,
                            src_u, src_stride_u,
                            src_v, src_stride_v,
                            dst_rgb24, dst_stride_rgb24,
                            width, height);
        break;
    case AV_PIX_FMT_YUVJ444P:
        libyuv::J444ToRGB24(src_y, src_stride_y,
                            src_u, src_stride_u,
                            src_v, src_stride_v,
                            dst_rgb24, dst_stride_rgb24,
                            width, height);
        break;
    case AV_PIX_FMT_YUV420P:
        libyuv::I420ToRGB24(src_y, src_stride_y,
                            src_u, src_stride_u,
                            src_v, src_stride_v,
                            dst_rgb24, dst_stride_rgb24,
                            width, height);
        break;
    case AV_PIX_FMT_NV12:
        if (frame->color_range == AVCOL_RANGE_JPEG)
        {
            fprintf(stderr, "Unsupported nv12 with full range\n");
            return -1;
        }
        else
        {
            libyuv::NV12ToRGB24(src_y, src_stride_y,
                                src_u, src_stride_u,
                                dst_rgb24, dst_stride_rgb24,
                                width, height);
        }
        break;
    case AV_PIX_FMT_GRAY8:
        if (frame->color_range == AVCOL_RANGE_JPEG)
        {
            libyuv::J400ToRGB24(src_y, src_stride_y,
                                dst_rgb24, dst_stride_rgb24,
                                width, height);
        }
        else
        {
            libyuv::I400ToRGB24(src_y, src_stride_y,
                                dst_rgb24, dst_stride_rgb24,
                                width, height);
        }
        break;
#endif
    default:
        fprintf(stderr, "Unsupported pixel format\n");
        return -1;
        break;
    }

#if defined(PROFILING_COLOR_CVT)
    gettimeofday(&end, NULL);
    double s_ms = start.tv_sec*1000.0 + start.tv_usec/1000.0;
    double t_ms = end.  tv_sec*1000.0 + end.  tv_usec/1000.0;
    printf("Conversion time: %.1fms\n", t_ms - s_ms);
#endif

    return 0;
}

static int decode_packet(AVCodecContext *dec_ctx, int *got_frame, AVFrame *frame, FILE* bgrf, AVPacket *pkt)
{
    int decoded = pkt->size;
    int ret = 0;

    *got_frame = 0;

    /* decode video frame */
    ret = avcodec_decode_video2(dec_ctx, frame, got_frame, pkt);
    if (ret < 0) {
        fprintf(stderr, "Error decoding video frame (%d)\n", ret);
        return ret;
    }

    if (*got_frame) {
        if (frame->width  != dec_ctx->width || 
            frame->height != dec_ctx->height ||
            frame->format != dec_ctx->pix_fmt) {
            /* To handle this change, one could call av_image_alloc again and
             * decode the following frames into another rawvideo file. */
            fprintf(stderr, "Error: Width, height and pixel format have to be "
                    "constant in a rawvideo file, but the width, height or "
                    "pixel format of the input video changed:\n"
                    "old: width = %d, height = %d, format = %s\n"
                    "new: width = %d, height = %d, format = %s\n",
                    dec_ctx->width, dec_ctx->height,
                    av_get_pix_fmt_name(dec_ctx->pix_fmt),
                    frame->width, frame->height,
                    av_get_pix_fmt_name((AVPixelFormat)frame->format));
            return -1;
        }

#if 0
        printf("video_frame display: %d coded: %d\n",
               frame->display_picture_number, frame->coded_picture_number);
#endif

        cv::Mat img(frame->height, frame->width, CV_8UC3);
        ret = process_frame(&img, frame);
        if(ret == 0) {
            uint8_t* p = img.data;
            int stride = img.step[0];
            for (int i=0; i<img.rows; i++) {
                fwrite(p, sizeof(uint8_t), img.cols*3, bgrf);
                p += stride;
            }
#if 0
            static int picture_number = 0;
            imwrite("out" + std::to_string(picture_number) + ".png", img);
            picture_number++;
#endif
        }
        if (!img.empty())
            img.release();
    }

    /* If we use frame reference counting, we own the data and need
     * to de-reference it when we don't use it anymore */
    if (*got_frame && refcount)
        av_frame_unref(frame);

    return decoded;
}

static int open_codec_context(AVCodecContext **dec_ctx,
                              AVFormatContext *fmt_ctx,
                              enum AVMediaType type)
{
    int ret, stream_index;
    AVStream *st = NULL;
    AVCodec *dec = NULL;
    AVDictionary *opts = NULL;

    ret = av_find_best_stream(fmt_ctx, type, -1, -1, NULL, 0);
    if (ret < 0) {
        fprintf(stderr, "Could not find %s stream\n",
                av_get_media_type_string(type));
        return ret;
    }

    stream_index = ret;
    st = fmt_ctx->streams[stream_index];

    /* find decoder for the stream */
    dec = avcodec_find_decoder(st->codecpar->codec_id);
    if (!dec) {
        fprintf(stderr, "Failed to find %s codec\n",
                av_get_media_type_string(type));
        return AVERROR(EINVAL);
    }

    /* Allocate a codec context for the decoder */
    *dec_ctx = avcodec_alloc_context3(dec);
    if (!*dec_ctx) {
        fprintf(stderr, "Failed to allocate the %s codec context\n",
                av_get_media_type_string(type));
        return AVERROR(ENOMEM);
    }

    /* Copy codec parameters from input stream to output codec context */
    ret = avcodec_parameters_to_context(*dec_ctx, st->codecpar);
    if (ret < 0) {
        fprintf(stderr, "Failed to copy %s codec parameters to decoder context\n",
                av_get_media_type_string(type));
        return ret;
    }

    /* Init the decoders, with or without reference counting */
    av_dict_set(&opts, "refcounted_frames", refcount ? "1" : "0", 0);
    ret = avcodec_open2(*dec_ctx, dec, &opts);
    if (ret < 0) {
        fprintf(stderr, "Failed to open %s codec\n",
                av_get_media_type_string(type));
        return ret;
    }

    return stream_index;
}

int main (int argc, char **argv)
{
    AVFormatContext *fmt_ctx = NULL;
    AVStream *video_st = NULL;
    int video_st_idx = -1;
    AVCodecContext *dec_ctx = NULL;
    AVFrame *frame = NULL;
    AVPacket pkt;
    const char *src_filename = NULL;
    const char *bgr_filename = NULL;
    FILE *bgr_file = NULL;
    int width, height;
    int ret = 0, got_frame;

    if (argc != 3 && argc != 4) {
        fprintf(stderr, "usage:\n\t%s [-refcount] input_file video_output_file\n"
                "Read frames from a file, decodes and writes decoded video frames to a rawvideo file\n"
                "If the -refcount option is specified, the program use the\n"
                "reference counting frame system which allows keeping a copy of\n"
                "the data for longer than one decode call.\n"
                "\n", argv[0]);
        exit(1);
    }
    if (argc == 4 && !strcmp(argv[1], "-refcount")) {
        refcount = 1;
        argv++;
    }
    src_filename = argv[1];
    bgr_filename = argv[2];

    /* open input file, and allocate format context */
    ret = avformat_open_input(&fmt_ctx, src_filename, NULL, NULL);
    if (ret < 0) {
        fprintf(stderr, "Could not open source file %s\n", src_filename);
        goto end;
    }

    /* retrieve stream information */
    ret = avformat_find_stream_info(fmt_ctx, NULL);
    if (ret < 0) {
        fprintf(stderr, "Could not find stream information\n");
        goto end;
    }

    ret = open_codec_context(&dec_ctx, fmt_ctx, AVMEDIA_TYPE_VIDEO);
    if (ret < 0) {
        goto end;
    }

    video_st_idx = ret;
    video_st = fmt_ctx->streams[video_st_idx];
    if (!video_st) {
        fprintf(stderr, "Could not find video stream in the input, aborting\n");
        ret = 1;
        goto end;
    }

    bgr_file = fopen(bgr_filename, "wb");
    if (!bgr_file) {
        fprintf(stderr, "Could not open destination file %s\n", bgr_filename);
        ret = 1;
        goto end;
    }

    width   = dec_ctx->width;
    height  = dec_ctx->height;

    printf("width = %d, height = %d, format = %s\n",
           dec_ctx->width, dec_ctx->height,
           av_get_pix_fmt_name(dec_ctx->pix_fmt));

    /* dump input information to stderr */
    av_dump_format(fmt_ctx, 0, src_filename, 0);

    /* allocate image where the decoded image will be put */
    frame = av_frame_alloc();
    if (!frame) {
        fprintf(stderr, "Could not allocate frame\n");
        ret = AVERROR(ENOMEM);
        goto end;
    }

    /* initialize packet, set data to NULL, let the demuxer fill it */
    av_init_packet(&pkt);
    pkt.data = NULL;
    pkt.size = 0;

    printf("Demuxing video from file '%s' into '%s'\n", src_filename, bgr_filename);

    /* read frames from the file */
    while (av_read_frame(fmt_ctx, &pkt) >= 0) {
        AVPacket orig_pkt = pkt;
        if (pkt.stream_index != video_st_idx) {
            av_packet_unref(&pkt);
            continue;
        }

        /* Only decoding video packet */
        do {
            ret = decode_packet(dec_ctx, &got_frame, frame, bgr_file, &pkt);
            if (ret < 0)
                break;
            pkt.data += ret;
            pkt.size -= ret;
        } while (pkt.size > 0);
        av_packet_unref(&orig_pkt);
    }

    /* flush cached frames */
    pkt.data = NULL;
    pkt.size = 0;
    if (pkt.stream_index == video_st_idx) { // TODO
        do {
            decode_packet(dec_ctx, &got_frame, frame, bgr_file, &pkt);
        } while (got_frame);
    }

    printf("Demuxing succeeded.\n");

    if (video_st) {
        printf("Play the output video file with the command:\n"
               "ffplay -f rawvideo -pix_fmt bgr24 -video_size %dx%d %s\n",
               width, height, bgr_filename);
    }

end:
    if (dec_ctx)
        avcodec_free_context(&dec_ctx);
    if (fmt_ctx)
        avformat_close_input(&fmt_ctx);
    if (bgr_file)
        fclose(bgr_file);
    if (frame)
        av_frame_free(&frame);

    return ret;
}
