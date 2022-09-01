/*
 * Copyright (c) 2010 BitMan

/**
 * @file
 *  example for video encode from BGR data,
     ps: i implement a C++ encoder it will be more easy to understand ffmpeg
 */
#ifdef __cplusplus
     extern "C" {
#endif

#include "libavcodec/avcodec.h"
#include <libswscale/swscale.h>
#include <libavutil/imgutils.h>
#include <libavformat/avformat.h>
#include <libavfilter/buffersink.h>
#include <libavfilter/buffersrc.h>
#include <libavutil/opt.h>
#include <libavutil/pixdesc.h>
#ifdef __cplusplus
}
#endif
#include <iostream>

#define SIMD_SIZE 32

class VideoEnc_FFMPEG
{
public:
    VideoEnc_FFMPEG();
    ~VideoEnc_FFMPEG();

    int openEnc( const char* filename, int codecId, int framerate, int width, int height,int inputformat,int bitrate);
    int flush_encoder();
    void closeEnc();
    int writeFrame( const unsigned char* data, int step, int width, int height);
    static void bm_find_encoder_name(int enc_id, std::string &enc_name)
    {
        switch (enc_id)
        {
            case AV_CODEC_ID_H264:       enc_name = "h264_bm";    break;
            case AV_CODEC_ID_H265:       enc_name = "h265_bm";    break;
            default:                     enc_name = "";           break;
        }
    }
private:
    AVFormatContext * ofmt_ctx;
    AVCodecContext  * enc_ctx;
    AVFrame         * picture;
    AVFrame         * input_picture;
    AVStream        * out_stream;
    int               input_pix_fmt;
    unsigned char   * aligned_input;
    int               frame_width, frame_height;
    int               frame_idx;
    struct SwsContext *img_convert_ctx;
};

VideoEnc_FFMPEG::VideoEnc_FFMPEG()
{
    ofmt_ctx = NULL;
    picture= NULL;
    input_picture = NULL;
    out_stream = NULL;
    input_pix_fmt = NULL;
    aligned_input  = NULL;
    frame_width = 0;
    frame_height = 0;
    frame_idx = 0;
}
VideoEnc_FFMPEG::~VideoEnc_FFMPEG()
{
    printf("#######VideoEnc_FFMPEG exit \n");
}

int VideoEnc_FFMPEG::openEnc( const char* filename, int codecId, int framerate, int width, int height, int inputformat, int bitrate)
{

    int ret = 0;
    AVCodec *encoder;
    AVDictionary *dict = NULL;
    frame_idx = 0;
    //if (isColor) {
    //    input_pix_fmt = AV_PIX_FMT_BGR24;
    //}
    //else {
    //    input_pix_fmt = AV_PIX_FMT_GRAY8;
    //}
    input_pix_fmt = inputformat;
    frame_width = width;
    frame_height = height;
    avformat_alloc_output_context2(&ofmt_ctx, NULL, NULL, filename);
    if (!ofmt_ctx) {
        av_log(NULL, AV_LOG_ERROR, "Could not create output context\n");
        return AVERROR_UNKNOWN;
    }
    /* find the video encoder */
    std::string bm_enc_name =  "";
    bm_find_encoder_name((AVCodecID)codecId,bm_enc_name);
    if (bm_enc_name.empty())
        encoder = avcodec_find_encoder((AVCodecID)codecId);
    else
    {
        encoder = avcodec_find_encoder_by_name(bm_enc_name.c_str());
        /* if HW encoder not found try SW */
        if (!encoder)
        {
            encoder = avcodec_find_encoder((AVCodecID)codecId);
        }
    }
    if (!encoder) {
        av_log(NULL, AV_LOG_FATAL, "Necessary encoder not found\n");
        return AVERROR_INVALIDDATA;
    }
    enc_ctx = avcodec_alloc_context3(encoder);
    if (!enc_ctx) {
        av_log(NULL, AV_LOG_FATAL, "Failed to allocate the encoder context\n");
        return AVERROR(ENOMEM);
    }
    enc_ctx->codec_id = (AVCodecID)codecId;
    enc_ctx->height = height;
    enc_ctx->width = width;
    enc_ctx->pix_fmt = AV_PIX_FMT_YUV420P;
    enc_ctx->bit_rate_tolerance = bitrate;
    enc_ctx->bit_rate = (int)bitrate;
    enc_ctx->gop_size = 32;
    printf("enc_ctx->bit_rate = %d \n",enc_ctx->bit_rate);
    /* video time_base can be set to whatever is handy and supported by encoder */
    enc_ctx->time_base = (AVRational){1, framerate};   // only for network stream frame rate
    enc_ctx->framerate = (AVRational){framerate,1};
    out_stream = avformat_new_stream(ofmt_ctx, encoder);
    out_stream->time_base = enc_ctx->time_base;
    out_stream->avg_frame_rate = enc_ctx->framerate;
    out_stream->r_frame_rate = out_stream->avg_frame_rate;
    av_dict_set_int(&dict, "gop_preset", 8, 0);
    /* when use system memory, is_dma_buffer must be set 0 */
    av_dict_set_int(&dict, "is_dma_buffer", 0, 0);
    /* Third parameter can be used to pass settings to encoder */
    ret = avcodec_open2(enc_ctx, encoder, &dict);
    if (ret < 0) {
        av_log(NULL, AV_LOG_ERROR, "Cannot open video encoder ");
        return ret;
    }
    ret = avcodec_parameters_from_context(out_stream->codecpar, enc_ctx);
    if (ret < 0) {
        av_log(NULL, AV_LOG_ERROR, "Failed to copy encoder parameters to output stream ");
        return ret;
    }
    if (!(ofmt_ctx->oformat->flags & AVFMT_NOFILE)) {
        ret = avio_open(&ofmt_ctx->pb, filename, AVIO_FLAG_WRITE);
        if (ret < 0) {
            av_log(NULL, AV_LOG_ERROR, "Could not open output file '%s'", filename);
            return ret;
        }
    }
    /* init muxer, write output file header */
    ret = avformat_write_header(ofmt_ctx, NULL);
    if (ret < 0) {
        av_log(NULL, AV_LOG_ERROR, "Error occurred when opening output file\n");
        return ret;
    }
   //av_dump_format(ofmt_ctx, 0, filename, 1);   // only for debug

   if ( enc_ctx->pix_fmt != input_pix_fmt )
    {
        input_picture = av_frame_alloc();
        input_picture->format = input_pix_fmt;
        input_picture->width = width;
        input_picture->height = height;
    }
    picture = av_frame_alloc();
    picture->format = enc_ctx->pix_fmt;
    picture->width = width;
    picture->height = height;

    if ( enc_ctx->pix_fmt != input_pix_fmt )
    {
        uint8_t * picture_buf = 0;
        picture_buf = (uint8_t *) malloc(av_image_get_buffer_size( (AVPixelFormat) enc_ctx->pix_fmt, width, height,1));
        if (!picture_buf)
        {
            av_free(picture);
            return -1;
        }
        av_image_fill_arrays(picture->data, picture->linesize, picture_buf, enc_ctx->pix_fmt, width, height, 1);
    }
    return 0;
}

//data is allign with 32
int VideoEnc_FFMPEG::writeFrame( const unsigned char* data, int step, int width, int height)
{
    int ret = 0 ;
    int got_output = 0;
    if( step % SIMD_SIZE != 0)
    {
        av_log(NULL, AV_LOG_ERROR, "input step must align with SIMD_SIZE\n");
        return -1;
    }
    if ( enc_ctx->pix_fmt != input_pix_fmt ) {
        // let input_picture point to the raw data buffer of 'image'
        //input_picture->linesize[0] = step;
        av_image_fill_arrays(input_picture->data, input_picture->linesize, (uint8_t *) data, (AVPixelFormat)input_pix_fmt, width, height, 1);
        input_picture->linesize[0] = step;

        if( !img_convert_ctx )
        {
            img_convert_ctx = sws_getContext(width,
                                             height,
                                             (AVPixelFormat)input_pix_fmt,
                                             enc_ctx->width,
                                             enc_ctx->height,
                                             enc_ctx->pix_fmt,
                                             SWS_BICUBIC,
                                             NULL, NULL, NULL);
            if( !img_convert_ctx )
                return -1;
        }

        if ( sws_scale(img_convert_ctx, input_picture->data,
                       input_picture->linesize, 0,
                       height,
                       picture->data, picture->linesize) < 0 )
            return -1;
    }
    else{
        av_image_fill_arrays(picture->data, picture->linesize, (uint8_t *) data, enc_ctx->pix_fmt, width, height, 1);
        picture->linesize[0] = step;
    }

    picture->pts = frame_idx;
    frame_idx++;
    AVPacket enc_pkt;
    av_log(NULL, AV_LOG_INFO, "Encoding frame\n");
    /* encode filtered frame */
    enc_pkt.data = NULL;
    enc_pkt.size = 0;
    av_init_packet(&enc_pkt);
    ret = avcodec_encode_video2(enc_ctx, &enc_pkt, picture, &got_output);
    if (ret < 0)
        return ret;
    if (got_output == 0)
    {
       av_log(NULL, AV_LOG_ERROR, "can't get output from encoder\n");
       return -1;
    }
    /* prepare packet for muxing */
    printf("xinging encode_write_frame enc_pkt.pts=%lld,enc_pkt.dts=%lld \n ",enc_pkt.pts,enc_pkt.dts);
    av_packet_rescale_ts(&enc_pkt, enc_ctx->time_base,out_stream->time_base);
    printf("real encode_write_frame enc_pkt.pts=%lld,enc_pkt.dts=%lld \n ",enc_pkt.pts,enc_pkt.dts);
    av_log(NULL, AV_LOG_DEBUG, "Muxing frame\n");
    /* mux encoded frame */
    ret = av_interleaved_write_frame(ofmt_ctx, &enc_pkt);
    return ret;

}
int  VideoEnc_FFMPEG::flush_encoder()
{
    int ret;
    int got_frame = 0;

    if (!(enc_ctx->codec->capabilities & AV_CODEC_CAP_DELAY))
        return 0;

    while (1) {
        av_log(NULL, AV_LOG_INFO, "Flushing video encoder\n");
        AVPacket enc_pkt;
        enc_pkt.data = NULL;
        enc_pkt.size = 0;
        av_init_packet(&enc_pkt);

        ret = avcodec_encode_video2(enc_ctx, &enc_pkt, NULL, &got_frame);
        if (ret < 0)
            return ret;

        if (!got_frame)
            break;

        /* prepare packet for muxing */
        printf("xinging encode_write_frame enc_pkt.pts=%lld,enc_pkt.dts=%lld \n ",enc_pkt.pts,enc_pkt.dts);
        av_packet_rescale_ts(&enc_pkt, enc_ctx->time_base,out_stream->time_base);
        printf("real encode_write_frame enc_pkt.pts=%lld,enc_pkt.dts=%lld \n ",enc_pkt.pts,enc_pkt.dts);
        av_log(NULL, AV_LOG_DEBUG, "Muxing frame\n");
        /* mux encoded frame */
        ret = av_interleaved_write_frame(ofmt_ctx, &enc_pkt);
        if (ret < 0)
            break;

    }
    return ret;
}

void VideoEnc_FFMPEG::closeEnc()
{
    flush_encoder();
    av_write_trailer(ofmt_ctx);

    if( img_convert_ctx )
    {
        sws_freeContext(img_convert_ctx);
        img_convert_ctx = 0;
    }

    if( enc_ctx->pix_fmt != input_pix_fmt)
    {
        if(picture->data[0])
            free(picture->data[0]);
        picture->data[0] = 0;
    }

    av_free(picture);

    if (input_picture)
        av_free(input_picture);

    avcodec_free_context(&enc_ctx);

    if (ofmt_ctx && !(ofmt_ctx->oformat->flags & AVFMT_NOFILE))
        avio_closep(&ofmt_ctx->pb);
    avformat_free_context(ofmt_ctx);

}




int main(int argc, char **argv)
{
    int ret;
    int enccodec_id = AV_CODEC_ID_H264;
    int inputformat = AV_PIX_FMT_BGR24;
    bool isYUV = true;
    int framerate = 30;
    if (argc < 6) {
        av_log(NULL, AV_LOG_ERROR, "Usage: %s <input file> <output file>  H264 width height dataformat  bitrate(kbps)  framerate\n", argv[0]);
        av_log(NULL, AV_LOG_ERROR, "Usage: %s <input file> <output file>  H264 width height RGB/YUV 3000 30\n", argv[0]);
        av_log(NULL, AV_LOG_ERROR, "Usage: %s <input file> <output file>  H265 width height RGB/YUV\n", argv[0]);
        return 1;
    }
    if(argc > 4)
    {
        if(strstr(argv[3],"H265") != NULL)
        {
           enccodec_id = AV_CODEC_ID_H265;
        }
    }
    int width = atoi(argv[4]);
    int height = atoi(argv[5]);
    if(argc >= 7)
    {
        if(strstr(argv[6],"YUV") != NULL)
        {
           isYUV = true;
        }
    }
    if (argc >= 9)
    {
        int temp = atoi(argv[8]);
        if(temp >10 && temp <= 60)
        {
             framerate = temp;
        }
    }
    int bitrate = framerate*width*height/8;
    if(enccodec_id == AV_CODEC_ID_H265)
    {
       bitrate = bitrate/2;
    }
    if (argc > 7 )
    {
        int temp = atoi(argv[7]);
        if(temp >10 && temp < 10000)
        {
             bitrate = temp*1000;
        }
    }

    const int CV_STEP_ALIGNMENT = SIMD_SIZE;
    const size_t CV_SIMD_SIZE = SIMD_SIZE;
    int aligned_step = 0;
    size_t aligned_input_size = 0;
    if(isYUV)
    {
        inputformat = AV_PIX_FMT_YUV420P;
        aligned_step = (width + CV_STEP_ALIGNMENT - 1) & ~(CV_STEP_ALIGNMENT - 1);
        aligned_input_size = (aligned_step * height*3/2 + CV_SIMD_SIZE);
    }
    else
    {
       aligned_step = (width*3 + CV_STEP_ALIGNMENT - 1) & ~(CV_STEP_ALIGNMENT - 1);
       aligned_input_size = (aligned_step * height + CV_SIMD_SIZE);
    }

    unsigned char   *aligned_input = (unsigned char*)av_mallocz(aligned_input_size);
    FILE *in_file = fopen(argv[1], "rb");   //Input raw RGB data
    bool isFileEnd = false;
    VideoEnc_FFMPEG writer;
    if (writer.openEnc(argv[2],enccodec_id, framerate , width, height, inputformat,bitrate) !=0)
    {
        av_log(NULL, AV_LOG_ERROR,"writer.openEnc failed \n ");
        return -1;
    }
    int y = 0;
    while(1)
    {
        if(isYUV)
        {
            for(y = 0; y < height*3/2; y++ )
            {
                if (fread(aligned_input + y*aligned_step, 1, width, in_file) <= 0){
                   av_log(NULL, AV_LOG_ERROR,"Failed to read raw data! \n");
                   isFileEnd = true;
                   break;
                }else if(feof(in_file)){
                    isFileEnd = true;
                    break;
                }
            }
            if (y >= height*3/2-1)
            {
               writer.writeFrame(aligned_input,aligned_step, width, height);
            }
            else
            {
                break;
            }
        }
        else
        {
            for(y = 0; y < height; y++ )
            {
                if (fread(aligned_input + y*aligned_step, 1, width*3, in_file) <= 0){
                   av_log(NULL, AV_LOG_ERROR,"Failed to read raw data! \n");
                   isFileEnd = true;
                   break;
                }else if(feof(in_file)){
                    isFileEnd = true;
                    break;
                }
            }
            if (y >= height-1)
            {
               writer.writeFrame(aligned_input,aligned_step, width, height);
            }
            else
            {
                break;
            }
        }
    }
    writer.closeEnc();
    fclose(in_file);
    av_free(aligned_input);
    av_log(NULL, AV_LOG_ERROR,"encode finish! \n");
    return 0;
}
