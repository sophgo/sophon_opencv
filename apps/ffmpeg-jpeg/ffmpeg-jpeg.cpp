/*=============================================================================
# FileName: ffmpeg_jpeg.c
# Desc: an example of ffmpeg read from memory
# Author: wei.cheng
=============================================================================*/

#define __STDC_CONSTANT_MACROS

#ifdef __cplusplus
extern "C" {
#endif

#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavformat/avio.h>
#include <libavutil/file.h>
#include <libavutil/pixfmt.h>

#ifdef __cplusplus
}
#endif

#include <iostream>

using namespace std;

typedef struct {
    uint8_t* start;
    int      size;
    int      pos;
} bs_buffer_t;


static int read_buffer(void *opaque, uint8_t *buf, int buf_size);
static int writeJPEG(AVFrame* pFrame, int iIndex, string filename);
static int saveYUV(AVFrame* pFrame, int iIndex, string filename);

static int read_buffer(void *opaque, uint8_t *buf, int buf_size)
{
    bs_buffer_t* bs = (bs_buffer_t*)opaque;

    int r = bs->size - bs->pos;
    if (r <= 0) {
        cout << "EOF of AVIO." << endl;
        return AVERROR_EOF;
    }

    uint8_t* p = bs->start + bs->pos;
    int len = (r >= buf_size) ? buf_size : r;
    memcpy(buf, p, len);

    //cout << "read " << len << endl;

    bs->pos += len;

    return len;
}

static int writeJPEG(AVFrame* pFrame, int iIndex, string filename)
{
    AVCodec         *pCodec  = nullptr;
    AVCodecContext  *enc_ctx = nullptr;
    AVDictionary    *dict    = nullptr;
    int ret = 0;

    string out_str = "new-" + filename + "-" + to_string(iIndex) + ".jpg";
    FILE *outfile = fopen(out_str.c_str(), "wb");

    /* Find HW JPEG encoder: jpeg_bm */
    pCodec = avcodec_find_encoder_by_name("jpeg_bm");
    if( !pCodec ) {
        cerr << "Codec jpeg_bm not found." << endl;
        return -1;
    }

    enc_ctx = avcodec_alloc_context3(pCodec);
    if (enc_ctx == NULL) {
        cerr << "Could not allocate video codec context!" << endl;
        return AVERROR(ENOMEM);
    }

    enc_ctx->pix_fmt = AVPixelFormat(pFrame->format);
    enc_ctx->width   = pFrame->width;
    enc_ctx->height  = pFrame->height;
    enc_ctx->time_base = (AVRational){1, 25};
    enc_ctx->framerate = (AVRational){25, 1};

    /* 0: the data stored in virtual memory(pFrame->data[0-2]) */
    /* 1: the data stored in continuous physical memory(pFrame->data[3-5]) */
    int64_t value = 1;
    av_dict_set_int(&dict, "is_dma_buffer", value, 0);
    /* Open jpeg_bm encoder */
    ret = avcodec_open2(enc_ctx, pCodec, &dict);
    if (ret < 0) {
        cerr << "Could not open codec." << endl;
        return ret;
    }

    AVPacket *pkt = av_packet_alloc();
    if (!pkt) {
        cerr << "av_packet_alloc failed" << endl;
        return AVERROR(ENOMEM);
    }

    ret = avcodec_send_frame(enc_ctx, pFrame);
    if (ret < 0) {
        cerr << "Error sending a frame for encoding" << endl;
        return ret;
    }

    while (ret >= 0) {
        ret = avcodec_receive_packet(enc_ctx, pkt);
        if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF)
            return ret;
        else if (ret < 0) {
            cerr << "Error during encoding" << endl;
            return ret;
        }

        cout << "packet size=" << pkt->size << endl;
        fwrite(pkt->data, 1, pkt->size, outfile);
        av_packet_unref(pkt);
    }

    av_packet_free(&pkt);

    fclose(outfile);

    av_dict_free(&dict);
    avcodec_free_context(&enc_ctx);

    cout << "Encode Successful." << endl;

    return 0;
}


static int saveYUV(AVFrame* pFrame, int iIndex, string filename)
{
    string yuv_filename = "new-" + filename + "-" + to_string(iIndex) + ".yuv";
    FILE *fp = fopen(yuv_filename.c_str(), "wb+");
    if (fp == nullptr) {
        cerr << "create yuv file failed:" <<  yuv_filename << endl;
        return -1;
    }

    uint8_t * pY = pFrame->data[0];
    uint8_t * pU = pFrame->data[1];
    uint8_t * pV = pFrame->data[2];
    int frameWidth  = pFrame->width;
    int frameHeight = pFrame->height;
    int y_stride = pFrame->linesize[0];//may be larger than width because of alignment
    int u_stride = pFrame->linesize[1];
    int v_stride = pFrame->linesize[2];

    cout << "frame width:" << frameWidth << " frame height:" << frameHeight << endl;
    cout << "Y num:" << y_stride << " U num:" << u_stride << " V num:" << v_stride<< endl;

    if (AV_PIX_FMT_YUVJ420P == pFrame->format) {
        cout << "Pixel format is YUV420P" << endl;
        for (int j=0; j < frameHeight; j++) {
            fwrite(pY + j*y_stride, 1, frameWidth, fp);
        }

        for (int j=0; j < frameHeight / 2; j++) {
            fwrite(pU + j*u_stride, 1, frameWidth/2, fp);
        }

        for (int j=0; j < frameHeight / 2; j++) {
            fwrite(pV + j*v_stride, 1, frameWidth/2, fp);
        }
    } else if ( AV_PIX_FMT_YUVJ422P == pFrame->format) {
        cout << "Pixel format is YUV422P" << endl;
        for (int j=0; j < frameHeight; j++) {
            fwrite(pY + j*y_stride, 1, frameWidth, fp);
        }

        for (int j=0; j < frameHeight; j++) {
            fwrite(pU + j*u_stride, 1, frameWidth/2, fp);
        }

        for (int j=0; j < frameHeight; j++) {
            fwrite(pV + j*v_stride, 1, frameWidth/2, fp);
        }
    } else if (AV_PIX_FMT_YUVJ444P == pFrame->format) {
        cout << "Pixel format is YUVJ444P" << endl;
        for (int j=0; j < frameHeight; j++) {
            fwrite(pY + j*y_stride, 1, frameWidth, fp);
        }

        for (int j=0; j < frameHeight; j++) {
            fwrite(pU + j*u_stride, 1, frameWidth, fp);
        }

        for (int j=0; j < frameHeight; j++) {
            fwrite(pV + j*v_stride, 1, frameWidth, fp);
        }
    } else {
        cerr << "Not not support format:" << pFrame->format << endl;
        return -1;
    }

    fclose(fp);
    return 0;
}


int main(int argc, char* argv[])
{
    AVInputFormat   *iformat = nullptr;
    AVFormatContext *pFormatCtx = nullptr;
    AVCodecContext  *dec_ctx = nullptr;
    AVCodec         *pCodec = nullptr;
    AVDictionary    *dict = nullptr;
    AVIOContext     *avio_ctx = nullptr;
    AVFrame         *pFrame = nullptr;
    AVPacket         pkt;
    int              got_picture;
    FILE    *infile;
    int      numBytes;
    uint8_t *aviobuffer = nullptr;
    int      aviobuf_size = 32*1024; // 32K
    uint8_t *bs_buffer = nullptr;
    int      bs_size;
    bs_buffer_t bs_obj = {0, 0, 0};
    int ret = 0;
    int loop = 0;


    if (argc != 2) {
        cerr << "Usage:"<< argv[0] << " xxx.jpg" << endl;
        return 0;
    }

    string input_name = argv[1];

    auto pos1 = input_name.find(".jpg");
    auto pos2 = input_name.find(".jpeg");
    if (pos1 == string::npos && pos2 == string::npos) {
        cerr << "The input file is invalid jpeg file:" << input_name << endl;
        return -1;
    }

    infile = fopen(input_name.c_str(), "rb+");
    if (infile == nullptr) {
        cerr << "open file1 failed"  << endl;
        goto Func_Exit;
    }

    fseek(infile, 0, SEEK_END);
    numBytes = ftell(infile);
    cout << "infile size: " << numBytes << endl;
    fseek(infile, 0, SEEK_SET);

    bs_buffer = (uint8_t *)av_malloc(numBytes);
    if (bs_buffer == nullptr) {
        cerr << "av malloc for bs buffer failed" << endl;
        goto Func_Exit;
    }

    fread(bs_buffer, sizeof(uint8_t), numBytes, infile);
    fclose(infile);
    infile = nullptr;


    av_register_all();

    aviobuffer = (uint8_t *)av_malloc(aviobuf_size); //32k
    if (aviobuffer == nullptr) {
        cerr << "av malloc for avio failed" << endl;
        goto Func_Exit;
    }

    bs_obj.start = bs_buffer;
    bs_obj.size  = numBytes;
    bs_obj.pos   = 0;
    avio_ctx = avio_alloc_context(aviobuffer, aviobuf_size, 0,
                                  (void*)(&bs_obj), read_buffer, NULL, NULL);
    if (avio_ctx == NULL)
    {
        cerr << "avio_alloc_context failed" << endl;
        ret = AVERROR(ENOMEM);
        goto Func_Exit;
    }

    pFormatCtx = avformat_alloc_context();
    pFormatCtx->pb = avio_ctx;

    /* mjpeg demuxer */
    iformat = av_find_input_format("mjpeg");
    if (iformat == NULL) {
        cerr << "av_find_input_format failed." << endl;
        ret = AVERROR_DEMUXER_NOT_FOUND;
        goto Func_Exit;
    }

    /* Open an input stream */
    ret = avformat_open_input(&pFormatCtx, NULL, iformat, NULL);
    if (ret != 0) {
        cerr << "Couldn't open input stream.\n" << endl;
        goto Func_Exit;
    }

    //av_dump_format(pFormatCtx, 0, 0, 0);

    /* HW JPEG decoder: jpeg_bm */
    pCodec = avcodec_find_decoder_by_name("jpeg_bm");
    if (pCodec == NULL) {
        cerr << "Codec not found." << endl;
        ret = AVERROR_DECODER_NOT_FOUND;
        goto Func_Exit;
    }

    dec_ctx = avcodec_alloc_context3(pCodec);
    if (dec_ctx == NULL) {
        cerr << "Could not allocate video codec context!" << endl;
        ret = AVERROR(ENOMEM);
        goto Func_Exit;
    }


    /* Set parameters for jpeg_bm decoder */

    /* The output of bm jpeg decoder is chroma-separated,for example, YUVJ420P */
    av_dict_set_int(&dict, "chroma_interleave", 0, 0);
    /* The bitstream buffer size (KB) */
#define BS_MASK (1024*16-1)
    bs_size = (numBytes+BS_MASK)&(~BS_MASK); /* in unit of 16k */
#undef  BS_MASK
#define JPU_PAGE_UNIT_SIZE 256 /* each page unit of jpu is 256 byte */
    /* Avoid the false alarm that bs buffer is empty (SA3SW-252) */
    if (bs_size - numBytes < JPU_PAGE_UNIT_SIZE)
        bs_size += 16*1024;  /* in unit of 16k */
#undef JPU_PAGE_UNIT_SIZE
    bs_size /= 1024;
    cout << "bs buffer size: " << bs_size << "K" << endl;
    av_dict_set_int(&dict, "bs_buffer_size", bs_size, 0);
    /* Extra frame buffers: "0" for still jpeg, at least "2" for mjpeg */
    av_dict_set_int(&dict, "num_extra_framebuffers", 0, 0);

    ret = avcodec_open2(dec_ctx, pCodec, &dict);
    if (ret < 0) {
        cerr << "Could not open codec." << endl;
        ret = AVERROR_UNKNOWN;
        goto Func_Exit;
    }

    pFrame = av_frame_alloc();
    if (pFrame == nullptr) {
        cerr << "av frame malloc failed" << endl;
        goto Func_Exit;
    }

    while (av_read_frame(pFormatCtx, &pkt) >= 0) {
        ret = avcodec_decode_video2(dec_ctx, pFrame, &got_picture, &pkt);
        if (ret < 0) {
            cerr << "Decode Error." << endl;
            goto Func_Exit;
        }

        if (got_picture) {
            //save YUV data
            ret = saveYUV(pFrame, loop, "rencode-output");
            if (ret < 0) {
                break;
            }

            //convert AVFrame to JPEG
            cout << "pixel format: " << pFrame->format << endl;
            cout << "frame width : " << pFrame->width << endl;
            cout << "frame height: " << pFrame->height << endl;
            ret = writeJPEG(pFrame, loop, "rencode-output");
            if (ret < 0) {
                break;
            }

            loop++;
        }
        av_packet_unref(&pkt);//free mem because av_read_frame had applied for mem
        av_init_packet(&pkt);
    }

Func_Exit:
    av_packet_unref(&pkt);

    if (pFrame) {
        av_frame_free(&pFrame);
    }

    avformat_close_input(&pFormatCtx);

    if (avio_ctx) {
        av_freep(&avio_ctx->buffer);
        av_freep(&avio_ctx);
    }

    if (infile) {
        fclose(infile);
    }

    if (dict) {
        av_dict_free(&dict);
    }

    if (dec_ctx) {
        avcodec_close(dec_ctx);
    }
    if (bs_buffer) {
        av_free(bs_buffer);
    }
    return 0; // TODO
}

