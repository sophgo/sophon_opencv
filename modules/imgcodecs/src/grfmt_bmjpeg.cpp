/*M///////////////////////////////////////////////////////////////////////////////////////
//
//  IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
//
//  By downloading, copying, installing or using the software you agree to this license.
//  If you do not agree to this license, do not download, install,
//  copy or use the software.
//
//
//                        Bitmain License Agreement
//
// Copyright (C) 2018, Bitmain Corporation, all rights reserved.
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
//   * The name of Intel Corporation may not be used to endorse or promote products
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
#include "precomp.hpp"
#include "grfmt_bmjpeg.hpp"
#include "libyuv.h"
#include "bmlib_runtime.h"


#ifdef BM1684_CHIP

#ifdef _MSC_VER
//interaction between '_setjmp' and C++ object destruction is non-portable
#pragma warning(disable: 4611)
#endif

#include <stdio.h>
#include <stdarg.h>
#include <setjmp.h>
#ifndef _WIN32
#include <sys/time.h>
#endif

// the following defines are a hack to avoid multiple problems with frame ponter handling and setjmp
// see http://gcc.gnu.org/ml/gcc/2011-10/msg00324.html for some details
#define mingw_getsp(...) 0
#define __builtin_frame_address(...) 0

#ifdef _WIN32

#define XMD_H // prevent redefinition of INT32
#undef FAR  // prevent FAR redefinition

#endif

#if defined _WIN32 && defined __GNUC__
typedef unsigned char boolean;
#endif

#undef FALSE
#undef TRUE

extern "C" {
#include "jpeglib.h"
}

#define BS_MASK (1024*16-1)
#define CHIP_ID_1684 0x1684

#ifndef _WIN32
#define FAST_MEMCPY
#endif
#define HEAP_1_2    (0x6)
//#define PROFILING_DEC
//#define PROFILING_ENC
static int frame_dump_enc_num = 0;
static int frame_dump_dec_num = 0;
#ifndef _WIN32 //FIXME
extern int  g_vpp_fd[64];
#endif
namespace cv
{

struct JpegErrorMgr
{
    struct jpeg_error_mgr pub;
    jmp_buf setjmp_buffer;
};

struct JpegSource
{
    struct jpeg_source_mgr pub;
    int skip;
};

struct JpegState
{
    jpeg_decompress_struct cinfo; // IJG JPEG codec structure
    JpegErrorMgr jerr; // error processing manager state
    JpegSource source; // memory buffer source
};

enum dump_dir
{
    DUMP_IN  = 0,
    DUMP_OUT = 1
};

enum enSampleFactor
{
    FORMAT_RGB24 = 0,
    FORMAT_444   = 5,
    FORMAT_224   = 6,
    FORMAT_422   = 9,
    FORMAT_420   = 10,
    FORMAT_400   = 1
};

enum enDecodeStatus
{
    DECODE_INIT    = 0,
    DECODE_SUCCESS = 1,
    DECODE_FAILED  = 2
};

/////////////////////// Error processing /////////////////////
METHODDEF(void)
error_exit( j_common_ptr cinfo )
{
    JpegErrorMgr* err_mgr = (JpegErrorMgr*)(cinfo->err);

    /* Return control to the setjmp point */
    longjmp( err_mgr->setjmp_buffer, 1 );
}


/* Check for a non-baseline specification. */
static int check_jpeg_baseline(jpeg_decompress_struct* cinfo)
{
    if (cinfo->arith_code || cinfo->progressive_mode || cinfo->data_precision != 8)
        return 0;

    jpeg_component_info *p = cinfo->comp_info;
    for (int ci=0; ci < cinfo->num_components; ci++)
    {
        if (p->dc_tbl_no > 1 || p->ac_tbl_no > 1)
            return 0;

        JQUANT_TBL *qtbl = cinfo->quant_tbl_ptrs[p->quant_tbl_no];
        assert (qtbl != NULL);
        for (int i = 0; i < DCTSIZE2; i++)
        {
            if (qtbl->quantval[i] > 255)
                return 0;
        }

        p++;
    }

    return 1;
}

static int determine_hw_decoding(jpeg_decompress_struct* cinfo)
{
    /* A. Baseline ISO/IEC 10918-1 JPEG compliance */
    int is_baseline = check_jpeg_baseline(cinfo);
    if (!is_baseline)
        return 0;

#if 0
    printf("cinfo->num_components=%d\n", cinfo->num_components);
    printf("cinfo->comps_in_scan=%d\n", cinfo->comps_in_scan);
#endif

    /* B. Support 1 or 3 color components */
    if (cinfo->num_components != 1 && cinfo->num_components != 3)
        return 0;

    /* C. 3 component in a scan (interleaved only) */
    if (cinfo->num_components == 3 && cinfo->comps_in_scan != 3)
        return 0;

    /* The following cases are NOT supported by JPU */
    if (cinfo->comp_info[0].h_samp_factor>2 ||
        cinfo->comp_info[0].v_samp_factor>2)
        return 0;

    if (cinfo->num_components == 3 &&
        (cinfo->comp_info[1].h_samp_factor!=1 ||
         cinfo->comp_info[2].h_samp_factor!=1 ||
         cinfo->comp_info[1].v_samp_factor!=1 ||
         cinfo->comp_info[2].v_samp_factor!=1))
        return 0;

    int sampleFactor;
    if (cinfo->num_components == 1)
        sampleFactor = 0x1;
    else
        sampleFactor = (((cinfo->comp_info[0].h_samp_factor&3)<<2) |
                        ( cinfo->comp_info[0].v_samp_factor&3));

    /* For now, yuv420/yuv422/yuv444/yuv400 pictures ared decoded by JPU. */
    if ((0xA != sampleFactor) &&
        (0x9 != sampleFactor) &&
        (0x5 != sampleFactor) &&
        (0x1 != sampleFactor)
        )
        return 0;
    if (cinfo->jpeg_color_space != JCS_YCbCr &&
        cinfo->jpeg_color_space != JCS_GRAYSCALE &&
        cinfo->jpeg_color_space != JCS_RGB)
        return 0;

    if (cinfo->image_width < 16 || cinfo->image_height < 16)
        return 0;

    return 1;
}
static void get_csc_info(int sampleFactor, int width, int height, int& cbcr_width, int& cbcr_height )
{
    if(FORMAT_420 == sampleFactor)
    {
        cbcr_width = (width+1)/2;
        cbcr_height = (height+1)/2;
    }
    else if(FORMAT_422 == sampleFactor)
    {
        cbcr_width = (width+1)/2;
        cbcr_height = height;
    }
    else if(FORMAT_444 == sampleFactor)
    {
        cbcr_width = width;
        cbcr_height = height;
    }
}

static void write_plane(uint8_t* src, int width, int height, int stride, FILE* file)
{
    if(NULL == file)
        return;
    uint8_t* addr = src;
    for(int i = 0; i < height; i++)
    {
        fwrite(addr, sizeof(uint8_t), width, file);
        addr += (size_t)stride;
    }
}

#ifndef _WIN32
static void dump_jpu_framebuf(char * strFunc, uint8_t* virt_addr, BmJpuFramebuffer& framebuffer, int width, int height, int dir, int sampleFactor, int dump_num)
{
    FILE *fout = NULL;
    char filename[255];
    char* strdir;
    strdir = (dir == DUMP_IN) ? (char*)"csc_in": (char*)"csc_out";

    uint8_t* addr_y  = virt_addr + framebuffer. y_offset;
    uint8_t* addr_cb = virt_addr + framebuffer.cb_offset;
    uint8_t* addr_cr = virt_addr + framebuffer.cr_offset;

    pthread_t tid = pthread_self();
    sprintf(filename, "/data/pic%d_%s_%dx%d_step%d_sf%d_%u%s.bin", dump_num, strFunc, width, height, framebuffer.y_stride, sampleFactor, (unsigned int)tid,strdir);
    fout = fopen(filename, "wb");
    if(NULL!=fout)
    {
        int cbcr_width = 0;
        int cbcr_height = 0;

        get_csc_info(sampleFactor, width, height, cbcr_width, cbcr_height);

        write_plane(addr_y, (width+1)&~0x1, (height+1)&~0x1, framebuffer.y_stride, fout);            //write y
        write_plane(addr_cb, cbcr_width, cbcr_height, framebuffer.cbcr_stride, fout); //write u
        write_plane(addr_cr, cbcr_width, cbcr_height, framebuffer.cbcr_stride, fout); //write v
        if(NULL!=fout)
            fclose(fout);
    }

}
static void dump_jpu_mat(char * strFunc, const Mat&  img,  int dir, int sampleFactor, int dump_num)
{
    FILE *fout = NULL;
    char filename[255];
    char* strdir;

    strdir = (dir == DUMP_IN) ? (char*)"csc_in": (char*)"csc_out";
    pthread_t tid = pthread_self();
    sprintf(filename, "/tmp/pic%d_%s_%dx%d_step%d_sf%d_%u%s.bin",
            dump_num, strFunc, img.cols, img.rows, (int)img.step[0], sampleFactor, (unsigned int)tid, strdir);
    fout = fopen(filename, "wb");
    if(NULL==fout)
        return;

    uint8_t* bgr = img.data;
    int stride_rgb = img.step[0];
    int channels = img.channels();
    if( channels == 1 || channels == 3)
        write_plane(bgr, img.cols*channels, img.rows, stride_rgb, fout);   //write RGB
    else
        printf("warnning:dump_jpu_mat channels=%d\n", channels);

    fclose(fout);
}

static void dump_jpu_virt_addr(char * strFunc, uint8_t* virt_addr, BmJpuJPEGDecInfo&  info,  int dir, int sampleFactor, int dump_num)
{
    FILE *fout = NULL;
    char filename[255];
    char* strdir;

    uint8_t* addr_y = virt_addr;
    uint8_t* addr_cb = virt_addr + info.cb_offset;
    uint8_t* addr_cr = virt_addr + info.cr_offset;

    int width  = info.actual_frame_width;
    int height = info.actual_frame_height;
    strdir = (dir == DUMP_IN) ? (char*)"csc_in": (char*)"csc_out";
    pthread_t tid = pthread_self();
    sprintf(filename, "/data/pic%d_%s_%dx%d_stride%d_sf%d_%u%s.bin", dump_num, strFunc, width, height, info.y_stride, sampleFactor, (unsigned int)tid, strdir);
    fout = fopen(filename, "wb");
    if(fout != NULL)
    {
        int cbcr_width = 0;
        int cbcr_height = 0;

        get_csc_info(sampleFactor, width, height, cbcr_width, cbcr_height);

        write_plane(addr_y, (width+1)&~0x1, (height+1)&~0x1, info.y_stride, fout);            //write y
        write_plane(addr_cb, cbcr_width, cbcr_height, info.cbcr_stride, fout); //write u
        write_plane(addr_cr, cbcr_width, cbcr_height, info.cbcr_stride, fout); //write v

        if(NULL!=fout)
            fclose(fout);
    }
}
#endif
static void logging_func(BmJpuLogLevel level, char const *file,
                         int const line, char const *fn, const char *format, ...)
{
    va_list args;

    char const *lvlstr = "";
    switch (level)
    {
    case BM_JPU_LOG_LEVEL_ERROR:   lvlstr = "ERROR";   break;
    case BM_JPU_LOG_LEVEL_WARNING: lvlstr = "WARNING"; break;
    case BM_JPU_LOG_LEVEL_INFO:    lvlstr = "INFO";    break;
    case BM_JPU_LOG_LEVEL_DEBUG:   lvlstr = "DEBUG";   break;
    case BM_JPU_LOG_LEVEL_TRACE:   lvlstr = "TRACE";   break;
    case BM_JPU_LOG_LEVEL_LOG:     lvlstr = "LOG";     break;
    default: break;
    }

    fprintf(stderr, "%s:%d (%s)   %s: ", file, line, fn, lvlstr);

    va_start(args, format);
    vfprintf(stderr, format, args);
    va_end(args);

    fprintf(stderr, "\n");
}
static void bmjpu_setup_logging(void)
{
    BmJpuLogLevel level = BM_JPU_LOG_LEVEL_ERROR;
    //level = BM_JPU_LOG_LEVEL_WARNING;
    //level = BM_JPU_LOG_LEVEL_INFO;
    //level = BM_JPU_LOG_LEVEL_DEBUG;
    //level = BM_JPU_LOG_LEVEL_TRACE;

    //printf("bmjpuapi logging threshold: %d\n", level);

    bm_jpu_set_logging_threshold(level);
    bm_jpu_set_logging_function(logging_func);
}



/////////////////////// BMJpegDecoder ///////////////////
BMJpegDecoder::BMJpegDecoder()
:jpeg_decoder(NULL),m_vpp_fd(0),m_device_id(0)
{
    m_signature = "\xFF\xD8\xFF";
    m_state = NULL;
    m_f = NULL;
    m_buf_supported = true;
    m_src_data = NULL;
    m_data_len = 0;
    m_src_type = 0;
    char *strToSoft = getenv("OPENCV_RETRY_SOFTDEC");
    if(strToSoft){
        int toSof = atoi(strToSoft);
        if(toSof)
            m_retrySoftDec = true;
    }
}

BMJpegDecoder::~BMJpegDecoder()
{
    close(DECODE_INIT);
    bm_jpu_dec_unload(BM_CARD_ID(m_device_id));
}

void  BMJpegDecoder::close(int decode_status)
{

#ifdef USING_SOC
    if (jpeg_decoder &&  (!m_yuv_output || (decode_status == DECODE_FAILED)))
    {
        bm_jpu_jpeg_dec_close(jpeg_decoder);
        jpeg_decoder = NULL;
    }
#endif
    // for Other
    if( m_state )
    {
        JpegState * state = (JpegState*)m_state;
        jpeg_destroy_decompress( &state->cinfo );
        delete state;
        m_state = NULL;
    }

    if( m_f )
    {
        fclose( m_f );
        m_f = 0;
    }

    m_width = m_height = 0;
    m_type = -1;
}

ImageDecoder BMJpegDecoder::newDecoder() const
{
    // for Other
    return makePtr<BMJpegDecoder>();
}

bool BMJpegDecoder::readHeader()
{
    close(DECODE_INIT);

    JpegState* state = new JpegState;
    m_state = state;
    state->cinfo.err = jpeg_std_error(&state->jerr.pub);
    state->jerr.pub.error_exit = error_exit;
    m_src_type = 0;

    int ret = setjmp(state->jerr.setjmp_buffer);
    if (ret != 0)
    {
        close(DECODE_INIT);
        return false;
    }

    if( !m_buf.empty() )
    {
        m_src_data = (uint8_t *)m_buf.ptr();
        m_data_len = m_buf.cols*m_buf.rows*m_buf.elemSize();
        m_src_type = 0;

#if 0
        static int cnt = 0;
        char name[256];
        sprintf(name, "readHead%d.bin", cnt);
        FILE * tst = fopen(name, "wb");
        int ret = fwrite(m_src_data, sizeof(uint8_t), m_data_len, tst);
        cnt ++;
        fclose(tst);
        printf("%s(), line %d: m_data_len = %d  ,ret=%d\n",
               __FUNCTION__, __LINE__, m_data_len, ret);
#endif
    }
    else
    {
        m_f = fopen( m_filename.c_str(), "rb" );
        if (!m_f)
        {
            printf("Error! fopen failed.\n");
            close(DECODE_INIT);
            return false;
        }

        fseek(m_f, 0, SEEK_END);
        int fileSize = ftell( m_f );
        fseek(m_f, 0, SEEK_SET);

        m_src_data = (uint8_t *)malloc(fileSize);
        if (NULL == m_src_data)
        {
            printf("Error! malloc failed.\n");
            close(DECODE_INIT);
            return false;
        }

        ret = fread(m_src_data, sizeof(uint8_t), fileSize, m_f);
        if (ret != fileSize)
        {
            printf("Error! fread failed.\n");
            close(DECODE_INIT);
            return false;
        }
        m_data_len = ret;

        fclose(m_f);
        m_f = NULL;

        m_src_type = 1;
    }

    jpeg_create_decompress( &state->cinfo );

    jpeg_mem_src(&state->cinfo, m_src_data, m_data_len);
    if (state->cinfo.src == NULL)
    {
        printf("Error! state->cinfo.src is NULL.\n");
        close(DECODE_INIT);
        return false;
    }

    jpeg_read_header(&state->cinfo, TRUE);

    state->cinfo.scale_num=1;
    state->cinfo.scale_denom = m_scale_denom;
    m_scale_denom=1; // trick! to know which decoder used scale_denom see imread_

    jpeg_calc_output_dimensions(&state->cinfo);
    m_width  = state->cinfo.output_width;
    m_height = state->cinfo.output_height;
    m_type   = state->cinfo.num_components > 1 ? CV_8UC3 : CV_8UC1;

    return true;
}

/***************************************************************************
 * end of code for supportting MJPEG image files
 * based on a message of Laurent Pinchart on the video4linux mailing list
 ***************************************************************************/
/***************************************************************************
 * following code is for supporting MJPEG image files
 * based on a message of Laurent Pinchart on the video4linux mailing list
 ***************************************************************************/

/* JPEG DHT Segment for YCrCb omitted from MJPEG data */
static uint8_t my_jpeg_odml_dht[0x1a4] = {
    0xff, 0xc4, 0x01, 0xa2,

    0x00, 0x00, 0x01, 0x05, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b,

    0x01, 0x00, 0x03, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
    0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b,

    0x10, 0x00, 0x02, 0x01, 0x03, 0x03, 0x02, 0x04, 0x03, 0x05, 0x05, 0x04,
    0x04, 0x00, 0x00, 0x01, 0x7d,
    0x01, 0x02, 0x03, 0x00, 0x04, 0x11, 0x05, 0x12, 0x21, 0x31, 0x41, 0x06,
    0x13, 0x51, 0x61, 0x07,
    0x22, 0x71, 0x14, 0x32, 0x81, 0x91, 0xa1, 0x08, 0x23, 0x42, 0xb1, 0xc1,
    0x15, 0x52, 0xd1, 0xf0,
    0x24, 0x33, 0x62, 0x72, 0x82, 0x09, 0x0a, 0x16, 0x17, 0x18, 0x19, 0x1a,
    0x25, 0x26, 0x27, 0x28,
    0x29, 0x2a, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3a, 0x43, 0x44, 0x45,
    0x46, 0x47, 0x48, 0x49,
    0x4a, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5a, 0x63, 0x64, 0x65,
    0x66, 0x67, 0x68, 0x69,
    0x6a, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79, 0x7a, 0x83, 0x84, 0x85,
    0x86, 0x87, 0x88, 0x89,
    0x8a, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98, 0x99, 0x9a, 0xa2, 0xa3,
    0xa4, 0xa5, 0xa6, 0xa7,
    0xa8, 0xa9, 0xaa, 0xb2, 0xb3, 0xb4, 0xb5, 0xb6, 0xb7, 0xb8, 0xb9, 0xba,
    0xc2, 0xc3, 0xc4, 0xc5,
    0xc6, 0xc7, 0xc8, 0xc9, 0xca, 0xd2, 0xd3, 0xd4, 0xd5, 0xd6, 0xd7, 0xd8,
    0xd9, 0xda, 0xe1, 0xe2,
    0xe3, 0xe4, 0xe5, 0xe6, 0xe7, 0xe8, 0xe9, 0xea, 0xf1, 0xf2, 0xf3, 0xf4,
    0xf5, 0xf6, 0xf7, 0xf8,
    0xf9, 0xfa,

    0x11, 0x00, 0x02, 0x01, 0x02, 0x04, 0x04, 0x03, 0x04, 0x07, 0x05, 0x04,
    0x04, 0x00, 0x01, 0x02, 0x77,
    0x00, 0x01, 0x02, 0x03, 0x11, 0x04, 0x05, 0x21, 0x31, 0x06, 0x12, 0x41,
    0x51, 0x07, 0x61, 0x71,
    0x13, 0x22, 0x32, 0x81, 0x08, 0x14, 0x42, 0x91, 0xa1, 0xb1, 0xc1, 0x09,
    0x23, 0x33, 0x52, 0xf0,
    0x15, 0x62, 0x72, 0xd1, 0x0a, 0x16, 0x24, 0x34, 0xe1, 0x25, 0xf1, 0x17,
    0x18, 0x19, 0x1a, 0x26,
    0x27, 0x28, 0x29, 0x2a, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3a, 0x43, 0x44,
    0x45, 0x46, 0x47, 0x48,
    0x49, 0x4a, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5a, 0x63, 0x64,
    0x65, 0x66, 0x67, 0x68,
    0x69, 0x6a, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79, 0x7a, 0x82, 0x83,
    0x84, 0x85, 0x86, 0x87,
    0x88, 0x89, 0x8a, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98, 0x99, 0x9a,
    0xa2, 0xa3, 0xa4, 0xa5,
    0xa6, 0xa7, 0xa8, 0xa9, 0xaa, 0xb2, 0xb3, 0xb4, 0xb5, 0xb6, 0xb7, 0xb8,
    0xb9, 0xba, 0xc2, 0xc3,
    0xc4, 0xc5, 0xc6, 0xc7, 0xc8, 0xc9, 0xca, 0xd2, 0xd3, 0xd4, 0xd5, 0xd6,
    0xd7, 0xd8, 0xd9, 0xda,
    0xe2, 0xe3, 0xe4, 0xe5, 0xe6, 0xe7, 0xe8, 0xe9, 0xea, 0xf2, 0xf3, 0xf4,
    0xf5, 0xf6, 0xf7, 0xf8,
    0xf9, 0xfa
};

/*
 * Parse the DHT table.
 * This code comes from jpeg6b (jdmarker.c).
 */
static int my_jpeg_load_dht(struct jpeg_decompress_struct *info, uint8_t *dht,
                            JHUFF_TBL *ac_tables[], JHUFF_TBL *dc_tables[])
{
    unsigned int length = (dht[2] << 8) + dht[3] - 2;
    unsigned int pos = 4;
    unsigned int count, i;
    int index;

    JHUFF_TBL **hufftbl;
    uint8_t bits[17];
    uint8_t huffval[256] = {0};

    while (length > 16)
    {
        bits[0] = 0;
        index = dht[pos++];
        count = 0;
        for (i = 1; i <= 16; ++i)
        {
            bits[i] = dht[pos++];
            count += bits[i];
        }
        length -= 17;

        if (count > 256 || count > length)
            return -1;

        for (i = 0; i < count; ++i)
            huffval[i] = dht[pos++];
        length -= count;

        if (index & 0x10)
        {
            index &= ~0x10;
            hufftbl = &ac_tables[index];
        }
        else
            hufftbl = &dc_tables[index];

        if (index < 0 || index >= NUM_HUFF_TBLS)
            return -1;

        if (*hufftbl == NULL)
            *hufftbl = jpeg_alloc_huff_table ((j_common_ptr)info);
        if (*hufftbl == NULL)
            return -1;

        memcpy ((*hufftbl)->bits, bits, sizeof (*hufftbl)->bits);
        memcpy ((*hufftbl)->huffval, huffval, sizeof (*hufftbl)->huffval);
    }

    if (length != 0)
        return -1;
    return 0;
}

static void av_buffer_release(void *opaque, uint8_t *data)
{
    BmJpuJPEGDecoder *jpeg_decoder = (BmJpuJPEGDecoder *)opaque;
    UMatOpaque *jpeg_opaque;

    if (!jpeg_decoder)
        return;

    jpeg_opaque = (UMatOpaque *)jpeg_decoder->opaque;
    if (!jpeg_opaque || jpeg_opaque->magic_number != MAGIC_MAT)
        return;

    UMatData *u = (UMatData *)jpeg_opaque->data;

#ifdef USING_SOC
    bm_handle_t handle = bm_jpu_dec_get_bm_handle(jpeg_decoder->device_index);
    bm_status_t ret = bm_mem_unmap_device_mem(handle, u->origdata
                                             ,jpeg_decoder->raw_frame.framebuffer->dma_buffer->size);
    if (ret != BM_SUCCESS) {
        CV_Error(CV_HalMemErr, "unmap failed");
        return;
    }

#else
    if (u->origdata){ fastFree(u->origdata); u->origdata = 0;}
#endif

    /* Decoded frame is no longer needed,
     * so inform the decoder that it can reclaim it */
    bm_jpu_jpeg_dec_frame_finished(jpeg_decoder, jpeg_decoder->raw_frame.framebuffer);
    bm_jpu_jpeg_dec_close(jpeg_decoder);

    if (jpeg_opaque) delete jpeg_opaque;
    if (u) delete u;
}

AVFrame *BMJpegDecoder::fillAVFrame(BmJpuJPEGDecInfo& info)
{
    MatAllocator *a = av::getAllocator();
    UMatData *u = new UMatData(a);

    AVFrame *f = av_frame_alloc();
    f->height = info.actual_frame_height;
    f->width = info.actual_frame_width;
    f->colorspace = AVCOL_SPC_BT470BG;
    f->color_range = AVCOL_RANGE_JPEG;

    BmJpuFramebuffer *fb = info.framebuffer;

    uint8_t *data = (uint8_t *)bm_mem_get_device_addr(*(fb->dma_buffer));
    int size = fb->dma_buffer->size;

#ifdef USING_SOC
    unsigned long long vaddr = 0;
    bm_handle_t handle = bm_jpu_dec_get_bm_handle(m_device_id);
    bm_status_t ret =  bm_mem_mmap_device_mem(handle, fb->dma_buffer, &vaddr);
    if (ret != BM_SUCCESS) {
        CV_Error(CV_HalMemErr, "unmap failed");
        return NULL;
    }
    uint8_t *vddr = (uint8_t*)vaddr;
#else
    uint8_t *vddr = (uint8_t *)fastMalloc(size);
    if (!vddr) {
      CV_Error(CV_HalMemErr, "pci fastMalloc failed");
      av_frame_free(&f);
      delete u;
      return NULL;
    }
#endif

    UMatOpaque *jpeg_opaque = new UMatOpaque;
    jpeg_opaque->magic_number = MAGIC_MAT;
    jpeg_opaque->data = u;

    jpeg_decoder->opaque = jpeg_opaque;
    f->opaque = jpeg_opaque;

    u->data = u->origdata = vddr;
    u->size = size;
    u->addr = (bm_uint64)data;
#ifdef HAVE_BMCV
    u->hid = cv::bmcv::getCard(m_device_id);
    u->mem = cv::bmcv::getDeviceMem(u->addr, u->size);
#endif
    f->buf[4] = av_buffer_create(data, size, av_buffer_release, jpeg_decoder, AV_BUFFER_FLAG_READONLY);
    f->data[4] = data + fb->y_offset;
    f->data[0] = vddr + fb->y_offset;
    f->linesize[4] = fb->y_stride;
    f->linesize[0] = fb->y_stride;

    if (info.image_format == BM_JPU_IMAGE_FORMAT_YUV420P ||
        info.image_format == BM_JPU_IMAGE_FORMAT_NV12 ||
        info.image_format == BM_JPU_IMAGE_FORMAT_NV21)
    {
        f->data[5] = data + fb->cb_offset;
        f->data[1] = vddr + fb->cb_offset;
        f->linesize[5] = fb->cbcr_stride;
        f->linesize[1] = fb->cbcr_stride;

        // TODO: support NV21
        if (info.image_format != BM_JPU_IMAGE_FORMAT_YUV420P) {
            f->format = AV_PIX_FMT_NV12;
        } else {
            f->data[6] = data + fb->cr_offset;
            f->data[2] = vddr + fb->cr_offset;
            f->linesize[6] = fb->cbcr_stride;
            f->linesize[2] = fb->cbcr_stride;

            f->format = AV_PIX_FMT_YUV420P;
        }
    }
    else if (info.image_format == BM_JPU_IMAGE_FORMAT_YUV422P ||
        info.image_format == BM_JPU_IMAGE_FORMAT_NV16 ||
        info.image_format == BM_JPU_IMAGE_FORMAT_NV61)
    {
        f->data[5] = data + fb->cb_offset;
        f->data[1] = vddr + fb->cb_offset;
        f->linesize[5] = fb->cbcr_stride;
        f->linesize[1] = fb->cbcr_stride;

        // TODO: support NV61
        if (info.image_format != BM_JPU_IMAGE_FORMAT_YUV422P) {
            f->format = AV_PIX_FMT_NV16;
        } else {
            f->data[6] = data + fb->cr_offset;
            f->data[2] = vddr + fb->cr_offset;
            f->linesize[6] = fb->cbcr_stride;
            f->linesize[2] = fb->cbcr_stride;
            f->format = AV_PIX_FMT_YUV422P;
        }
    }
    else if (info.image_format == BM_JPU_IMAGE_FORMAT_YUV444P ||
             info.image_format == BM_JPU_IMAGE_FORMAT_RGB)
    {
        f->data[5] = data + fb->cb_offset;
        f->data[1] = vddr + fb->cb_offset;
        f->linesize[5] = fb->cbcr_stride;
        f->linesize[1] = fb->cbcr_stride;

        f->data[6] = data + fb->cr_offset;
        f->data[2] = vddr + fb->cr_offset;
        f->linesize[6] = fb->cbcr_stride;
        f->linesize[2] = fb->cbcr_stride;

        if (info.image_format == BM_JPU_IMAGE_FORMAT_YUV444P)
            f->format = AV_PIX_FMT_YUV444P;
        else
            f->format = AV_PIX_FMT_GBRP;   // reuse this GBRP as RGBP
    }
    else if (info.image_format == BM_JPU_IMAGE_FORMAT_GRAY)
    {
        f->format = AV_PIX_FMT_GRAY8;
    }

    return f;
}

bm_status_t dec_convert(Mat & img, int height, int width, unsigned long long p_phys_addr, unsigned int cb_offset, unsigned int cr_offset,
                      int* src_stride, int* dst_stride, bm_image_format_ext src_fmt,  csc_type_t csc_type)
{
  bm_status_t ret = BM_SUCCESS;
#ifdef HAVE_BMCV
  bm_handle_t handle = img.u->hid ? img.u->hid : cv::bmcv::getCard();

  bm_image       src, dst;
  bm_device_mem_t src_mem[4];
  bm_device_mem_t dst_mem[4];
  int src_size[4] = {0};
  int dst_size[4] = {0};

  ret = bm_image_create(handle, height, width, src_fmt, DATA_TYPE_EXT_1N_BYTE, &src,src_stride);
  if (ret != BM_SUCCESS) {
    printf("%s:%d[%s] Error:failed to create input bm_image.\n",__FILE__, __LINE__, __FUNCTION__);
    return ret;
  }

  ret = bm_image_create(handle, height, width, FORMAT_BGR_PACKED, DATA_TYPE_EXT_1N_BYTE, &dst,dst_stride);
  if (ret != BM_SUCCESS) {
    printf("%s:%d[%s] Error:failed to create output bm_image.\n",__FILE__, __LINE__, __FUNCTION__);
    bm_image_destroy(src);
    return ret;
  }

  ret = bm_image_get_byte_size(src, src_size);
  if(ret != BM_SUCCESS){
    printf("%s:%d[%s] Error:failed to get input byte size\n",__FILE__, __LINE__, __FUNCTION__);
    goto exit;

  }

  ret = bm_image_get_byte_size(dst, dst_size);
  if(ret != BM_SUCCESS){
    printf("%s:%d[%s] Error:failed to get output byte size\n",__FILE__, __LINE__, __FUNCTION__);
    goto exit;
  }

  src_mem[0].u.device.device_addr = (unsigned long)p_phys_addr;
  src_mem[0].size = src_size[0];
  src_mem[0].flags.u.mem_type     = BM_MEM_TYPE_DEVICE;

  src_mem[1].u.device.device_addr = (unsigned long)(p_phys_addr + cb_offset);
  src_mem[1].size = src_size[1];
  src_mem[1].flags.u.mem_type     = BM_MEM_TYPE_DEVICE;

  src_mem[2].u.device.device_addr = (unsigned long)(p_phys_addr + cr_offset);
  src_mem[2].size = src_size[2];
  src_mem[2].flags.u.mem_type     = BM_MEM_TYPE_DEVICE;

  dst_mem[0].u.device.device_addr = (unsigned long)img.u->addr;
  dst_mem[0].size = dst_size[0];
  dst_mem[0].flags.u.mem_type     = BM_MEM_TYPE_DEVICE;

  ret = bm_image_attach(src, src_mem);
  if(ret != BM_SUCCESS) {
    printf("%s:%d[%s] Error:failed to attach input mem\n",__FILE__, __LINE__, __FUNCTION__);
    goto exit;
  }

  ret = bm_image_attach(dst, dst_mem);
  if(ret != BM_SUCCESS) {
    printf("%s:%d[%s] Error:failed to attach output mem\n",__FILE__, __LINE__, __FUNCTION__);
    goto exit;
  }

  ret = bmcv_image_vpp_csc_matrix_convert(handle, 1, src, &dst, csc_type,NULL,BMCV_INTER_NEAREST,NULL);
  if(ret != BM_SUCCESS) {
    printf("%s:%d[%s] Error:failed to convert\n",__FILE__, __LINE__, __FUNCTION__);
    goto exit;
  }

exit:
    bm_image_destroy(dst);
    bm_image_destroy(src);
#endif
    return ret;

}


bm_status_t enc_convert(const Mat& img, unsigned long output_device_addr0, unsigned long output_device_addr1, unsigned long output_device_addr2,
                     int* dst_stride, int height, int width, bm_image_format_ext input_fmt, bm_image_format_ext output_fmt, bmcv_resize_algorithm algorithm, csc_type_t csc_type )
{

  bm_status_t ret = BM_SUCCESS;
#ifdef HAVE_BMCV
  bm_handle_t handle = img.u->hid ? img.u->hid :  cv::bmcv::getCard();
  int src_stride[4] = {0};
  src_stride[0] = img.step[0];

  bm_image input ,output;
  bm_device_mem_t input_mem[4];
  bm_device_mem_t output_mem[4];
  int input_size[4] = {0};
  int output_size[4] = {0};

  ret = bm_image_create(handle, height, width, input_fmt, DATA_TYPE_EXT_1N_BYTE, &input, src_stride);
  if (ret != BM_SUCCESS) {
    return ret;
  }
  ret = bm_image_create(handle, height, width, output_fmt, DATA_TYPE_EXT_1N_BYTE, &output, dst_stride);
  if (ret != BM_SUCCESS) {
    bm_image_destroy(input);
    return ret;
  }

  ret = bm_image_get_byte_size(input, input_size);
  if(ret != BM_SUCCESS){
    printf("%s:%d[%s] Error:failed to get input byte size\n",__FILE__, __LINE__, __FUNCTION__);
    goto exit;
  }
  ret = bm_image_get_byte_size(output, output_size);
  if(ret != BM_SUCCESS){
    printf("%s:%d[%s] Error:failed to get output byte size\n",__FILE__, __LINE__, __FUNCTION__);
    goto exit;
  }

  input_mem[0].u.device.device_addr = img.u->addr + img.data - img.datastart;
  input_mem[0].size = input_size[0];
  input_mem[0].flags.u.mem_type     = BM_MEM_TYPE_DEVICE;

  output_mem[0].u.device.device_addr = output_device_addr0;
  output_mem[0].size = output_size[0];
  output_mem[0].flags.u.mem_type     = BM_MEM_TYPE_DEVICE;

  output_mem[1].u.device.device_addr = output_device_addr1;
  output_mem[1].size = output_size[1];
  output_mem[1].flags.u.mem_type     = BM_MEM_TYPE_DEVICE;

  output_mem[2].u.device.device_addr = output_device_addr2;
  output_mem[2].size = output_size[2];
  output_mem[2].flags.u.mem_type     = BM_MEM_TYPE_DEVICE;

  ret = bm_image_attach(input, input_mem);
  if(ret != BM_SUCCESS) {
    printf("%s:%d[%s] Error:failed to attach input mem\n",__FILE__, __LINE__, __FUNCTION__);
    goto exit;
  }
  ret = bm_image_attach(output, output_mem);
  if(ret != BM_SUCCESS) {
    printf("%s:%d[%s] Error:failed to attach output mem\n",__FILE__, __LINE__, __FUNCTION__);
    goto exit;
  }
  ret = bmcv_image_vpp_csc_matrix_convert(handle, 1, input, &output, csc_type, NULL, algorithm, NULL);
  if(ret != BM_SUCCESS) {
    printf("%s:%d[%s] Error:failed to convert\n",__FILE__, __LINE__, __FUNCTION__);
    goto exit;
  }
exit:
    bm_image_destroy(output);
    bm_image_destroy(input);
#endif
    return ret;

}

int BMJpegDecoder::outputMat(Mat& img, BmJpuJPEGDecInfo &info)
{
    AVFrame *f;
    int channels = img.channels();
    if (m_yuv_output)
    {
        f = fillAVFrame(info);
        if (f) {
            /* img.create will not create new avframe if avframe is already in mat.
             * But here is special that avframe contains decoded buffer, we expect it
             * to be updated. So release img at first
             */
            img.release();
            img.create(f, m_device_id);
        }
        return 0;
    }

    if (channels > 1)
    {
#ifdef USING_SOC
        img.allocator = hal::getAllocator();
        if (img.empty() || !(img.u && img.u->addr) || (BM_CARD_ID(img.card) != BM_CARD_ID(m_device_id)) ||
            (img.cols != (int)info.actual_frame_width) || (img.rows != (int)info.actual_frame_height) ||
            (img.type() != CV_8UC3)){
            img.create(info.actual_frame_height, info.actual_frame_width, CV_8UC3, m_device_id);
        }

        bm_jpu_phys_addr_t p_phys_addr = bm_mem_get_device_addr(*((info.framebuffer)->dma_buffer));

        int src_stride[4] = {0}, dst_stride[4] = {0};
        src_stride[0] = info.y_stride;
        dst_stride[0] = img.step[0];

        bm_image_format_ext src_fmt;
        csc_type_t csc_type = CSC_YPbPr2RGB_BT601;

        if (info.image_format == BM_JPU_IMAGE_FORMAT_YUV420P ||
            info.image_format == BM_JPU_IMAGE_FORMAT_NV12 ||
            info.image_format == BM_JPU_IMAGE_FORMAT_NV21)
        {
            if (info.image_format == BM_JPU_IMAGE_FORMAT_NV12) {
                src_fmt = FORMAT_NV12;
                src_stride[1] = src_stride[0];

            }
            else if (info.image_format == BM_JPU_IMAGE_FORMAT_NV21) {
                src_fmt = FORMAT_NV21;
                src_stride[1] = src_stride[0];
            }
            else {
                src_fmt = FORMAT_YUV420P;
                src_stride[1] = src_stride[0]/2;
                src_stride[2] = src_stride[1];
            }

#if 0
            static int abc= 0;
            bm_handle_t handle = bm_jpu_dec_get_bm_handle(m_device_id);
            unsigned long long vmem  = 0;
            bm_mem_mmap_device_mem(handle, (info.framebuffer)->dma_buffer, &vmem);
            uint8_t* p_virt_addr2 = (uint8_t*)vmem;
            dump_jpu_framebuf((char*)"jpudec", p_virt_addr2,*(info.framebuffer), info.actual_frame_width, info.actual_frame_height, DUMP_OUT, FORMAT_420, abc++);
            bm_mem_unmap_device_mem(handle, (void *)p_virt_addr2, (info.framebuffer)->dma_buffer->size);
#endif
        }
        else if(info.image_format == BM_JPU_IMAGE_FORMAT_YUV422P)
        {
            /* Map the DMA buffer of the decoded picture */

            bm_handle_t handle = bm_jpu_dec_get_bm_handle(m_device_id);
            unsigned long long vmem  = 0;
            bm_status_t ret = bm_mem_mmap_device_mem(handle, (info.framebuffer)->dma_buffer, &vmem);
            uint8_t* p_virt_addr = (uint8_t*)vmem;

            if (ret != BM_SUCCESS) {
                CV_Error(CV_HalMemErr, "mmap failed");
                return NULL;
            }

            uint8_t* u = p_virt_addr + info.cb_offset;
            uint8_t* v = p_virt_addr + info.cr_offset;
            int stride_u = info.cbcr_stride;
            int stride_v = info.cbcr_stride;
            //dump_jpu_virt_addr((char*)"jpudec", p_virt_addr, info, DUMP_IN, FORMAT_422, frame_dump_dec_num);
            libyuv::I422ToI420(NULL, 0,
                               u, stride_u,
                               v, stride_v,
                               NULL, 0,
                               u, stride_u,
                               v, stride_v,
                               info.actual_frame_width, info.actual_frame_height);

            /* Flush data from cache to dma buffer for VPP */
            bm_mem_flush_device_mem(handle, (info.framebuffer)->dma_buffer);

            /* Unmap the DMA buffer of the decoded picture */
           
            bm_mem_unmap_device_mem(handle, p_virt_addr,(info.framebuffer)->dma_buffer->size);
            src_fmt = FORMAT_YUV420P;
            src_stride[1] = src_stride[0]/2;
            src_stride[2] = src_stride[1];
        }
        else if(info.image_format == BM_JPU_IMAGE_FORMAT_YUV444P)
        {
            src_fmt = FORMAT_YUV444P;
            src_stride[1] = src_stride[0];
            src_stride[2] = src_stride[1];
        }
        else if(info.image_format == BM_JPU_IMAGE_FORMAT_RGB)
        {
            src_fmt = FORMAT_RGBP_SEPARATE;
            csc_type = CSC_MAX_ENUM;
            src_stride[1] = src_stride[0];
            src_stride[2] = src_stride[1];
        }
        else if(info.image_format == BM_JPU_IMAGE_FORMAT_GRAY)
        {
            /* Map the DMA buffer of the decoded picture */
            unsigned long long vmem = 0;
            bm_handle_t handle = bm_jpu_dec_get_bm_handle(m_device_id);
            bm_mem_mmap_device_mem(handle, (info.framebuffer)->dma_buffer, &vmem);
            uint8_t* p_virt_addr = (uint8_t*)vmem;
            uint8_t* dst_rgb24 = img.data;
            int dst_stride_rgb24 = img.step[0];
            int width  = info.actual_frame_width;
            int height = info.actual_frame_height;

            libyuv::J400ToRGB24(p_virt_addr + info. y_offset, info.y_stride,
                            dst_rgb24, dst_stride_rgb24,
                            width, height);

            bm_mem_unmap_device_mem(handle, p_virt_addr,(info.framebuffer)->dma_buffer->size);
            img.allocator->invalidate(img.u, img.u->size);
            return 0;
            // static int ccc=0;
            //  dump_jpu_mat((char*)"jpudec", img, DUMP_OUT, FORMAT_RGB24, ccc++ );
        }
        img.allocator->invalidate(img.u, img.u->size); // cache is not flushed when input is physical address
#ifdef HAVE_BMCV
          CV_Assert(BM_SUCCESS == dec_convert(img, info.actual_frame_height, info.actual_frame_width, p_phys_addr, info.cb_offset, info.cr_offset, src_stride, dst_stride, src_fmt, csc_type));
#endif

#else
        f = fillAVFrame(info);
        Mat frame(f, m_device_id);
        if (frame.avOK() &&
            (frame.avFormat() == AV_PIX_FMT_NV16 || frame.avFormat() == AV_PIX_FMT_YUV422P))
        {
            bmcv::toMAT(frame, img, false);
            bmcv::downloadMat(img);
        }
        else    // yuv444P, NV12, YUV420P, YUV400, RGBP format
        {
            std::vector<Rect> vrt;
            std::vector<Size> vsz;
            std::vector<Mat> out;
            Rect rt;
            Size sz;
            csc_type_t csc = (frame.avFormat() == AV_PIX_FMT_GBRP) ? CSC_MAX_ENUM : CSC_YPbPr2RGB_BT601;

            rt.x = rt.y = 0;
            rt.width = sz.width = frame.cols;
            rt.height = sz.height = frame.rows;
            vrt.push_back(rt);
            vsz.push_back(sz);
            out.push_back(img);

            CV_Assert(BM_SUCCESS == cv::bmcv::convert(frame, vrt, vsz, out, false, \
                        csc, NULL, BMCV_INTER_NEAREST));

            img = out.front();
            bmcv::downloadMat(img);    // flush to system memory

            vrt.pop_back();
            vsz.pop_back();
            out.pop_back();
        }
#endif
    }
    else
    {
        if (img.empty() || !(img.u && img.u->addr) || (BM_CARD_ID(img.card) != BM_CARD_ID(m_device_id)) ||
            (img.cols != (int)info.actual_frame_width) || (img.rows != (int)info.actual_frame_height) ||
            (img.type() != CV_8UC1) || (img.step[0] != info.y_stride)){
            if (!img.allocator) img.allocator = hal::getAllocator();
            img.create(info.actual_frame_height, info.y_stride, CV_8UC1, m_device_id);
            if (info.actual_frame_width < info.y_stride)
                img = img(cv::Rect(0, 0, info.actual_frame_width, info.actual_frame_height));
        }

#ifdef USING_SOC

        unsigned long long vmem = 0;
        bm_handle_t handle = bm_jpu_dec_get_bm_handle(m_device_id);
        bm_mem_mmap_device_mem(handle, (info.framebuffer)->dma_buffer, &vmem);
        uint8_t* p_virt_addr = (uint8_t*)vmem;

        uint8_t* addr_y  = p_virt_addr + info.y_offset;
        libyuv::CopyPlane(addr_y, info.y_stride,
                             img.data, img.step[0],
                             info.actual_frame_width, info.actual_frame_height);

        bm_mem_unmap_device_mem(handle,p_virt_addr, (info.framebuffer)->dma_buffer->size);
        if (!img.allocator) img.allocator = hal::getAllocator();
        img.allocator->invalidate(img.u, img.u->size);
#else
        bm_uint64 src_paddr = bm_mem_get_device_addr(*((info.framebuffer)->dma_buffer));
        int src_size = (info.framebuffer)->dma_buffer->size;
        bm_device_mem_t src_mem = bmcv::getDeviceMem(src_paddr, src_size);
        bm_device_mem_t dst_mem = bmcv::getDeviceMem(img.u->addr, img.u->size);

        bm_memcpy_d2d(img.u->hid, dst_mem, 0, src_mem, info.y_offset, info.y_stride*info.actual_frame_height);
        bmcv::downloadMat(img);
#endif
    }
    return 0;
}

int BMJpegDecoder::softDec(Mat& img, unsigned int iScale, unsigned int scale_w, unsigned int scale_h){

#if defined(PROFILING_DEC)
    struct timeval start;
    struct timeval end;
    double t;
#endif

    jpeg_decompress_struct* cinfo = &((JpegState*)m_state)->cinfo;
    JSAMPARRAY buffer = 0;

#if defined(PROFILING_DEC)
        gettimeofday(&start, NULL);
#endif
    bool color = img.channels() > 1;
    Mat dst;
    if (iScale) {
        dst = img;
        img.create( m_height, m_width, img.type(), m_device_id);
    } else {
        if (img.empty() || (img.rows != m_height) || (img.cols != m_width))
            img.create(m_height, m_width, img.type(), m_device_id);
    }

    if (cinfo->num_components == 4)
    {
        cinfo->out_color_space = JCS_CMYK;
        cinfo->out_color_components = 4;
    }
    else if (color)
    {
        cinfo->out_color_space = JCS_RGB;
        cinfo->out_color_components = 3;
    }
    else
    {
        cinfo->out_color_space = JCS_GRAYSCALE;
        cinfo->out_color_components = 1;
    }

    jpeg_start_decompress(cinfo);
    buffer = (*cinfo->mem->alloc_sarray)((j_common_ptr)cinfo,
                                         JPOOL_IMAGE, m_width*4, 1 );

    size_t step  = img.step;
    uint8_t* data = img.ptr();

    for (; m_height--; data += step)
    {
        jpeg_read_scanlines(cinfo, buffer, 1);
        if (color)
        {
            if (cinfo->out_color_components == 3)
                icvCvt_RGB2BGR_8u_C3R(buffer[0], 0, data, 0, Size(m_width,1));
            else
                icvCvt_CMYK2BGR_8u_C4C3R(buffer[0], 0, data, 0, Size(m_width,1));
        }
        else
        {
            if (cinfo->out_color_components == 1)
                memcpy(data, buffer[0], m_width);
            else
                icvCvt_CMYK2Gray_8u_C4C1R(buffer[0], 0, data, 0, Size(m_width,1));
        }
    }

    jpeg_finish_decompress( cinfo );

    m_yuv_output = 0;
    if (iScale != 0)
    {
        Size dsize = Size(scale_w, scale_h);
        if (dst.empty() || !(dst.u && dst.u->addr) ||
            (dst.rows != scale_h) || (dst.cols != scale_w) ||
            (BM_CARD_ID(dst.card) != BM_CARD_ID(m_device_id))){
            dst.release();
            dst.create(dsize, img.type(), m_device_id);
        }
        cv::resize(img, dst, dsize, 0, 0, INTER_NEAREST);
        img = dst;
    }

#if defined(PROFILING_DEC)
    gettimeofday(&end, NULL);
    t = (end.tv_sec*1000.0 + end.tv_usec/1000.0) - (start.tv_sec*1000.0 + start.tv_usec/1000.0);
    printf("SW dec: %.1fms\n", t);
    gettimeofday(&start, NULL);
#endif
    return 0;
}

static int get_bm_chip_id(int card)
{
    unsigned int chipid;
    bm_handle_t handle;

    int ret = bm_dev_request(&handle, card);
    if (ret != BM_SUCCESS) {
        printf("Create bm handle failed. ret = %d\n", ret);
        return -1;
    }
    bm_get_chipid(handle, &chipid);
    if(handle != NULL){
        bm_dev_free(handle);
    }

    return chipid;
}

bool BMJpegDecoder::readData( Mat& img )
{
#if defined(PROFILING_DEC)
    struct timeval start;
    struct timeval end;
    double t;
#endif

    if (!m_state || !m_width || !m_height)
    {
        close(DECODE_INIT);
        fprintf(stderr, "Error! img info abnomal m_width=%d,m_height=%d \n",m_width,m_height);
        return false;
    }

    jpeg_decompress_struct* cinfo = &((JpegState*)m_state)->cinfo;
    JpegErrorMgr* jerr = &((JpegState*)m_state)->jerr;
    JSAMPARRAY buffer = 0;

    int ret = setjmp(jerr->setjmp_buffer);
    if (ret)
    {
        close(DECODE_INIT);
        fprintf(stderr, "Error! setjmp(jerr->setjmp_buffer) error \n");
        return false;
    }

    /* check if this is a mjpeg image format */
    if (cinfo->ac_huff_tbl_ptrs[0] == NULL &&
        cinfo->ac_huff_tbl_ptrs[1] == NULL &&
        cinfo->dc_huff_tbl_ptrs[0] == NULL &&
        cinfo->dc_huff_tbl_ptrs[1] == NULL)
    {
        /* yes, this is a mjpeg image format, so load the correct
         * huffman table */
        my_jpeg_load_dht(cinfo,
                         my_jpeg_odml_dht,
                         cinfo->ac_huff_tbl_ptrs,
                         cinfo->dc_huff_tbl_ptrs);
    }

    int sampleFactor;
    if (1 == cinfo->num_components)
        sampleFactor = 0x1;
    else
        sampleFactor = ((cinfo->comp_info[0].h_samp_factor&3)<<2) | (cinfo->comp_info[0].v_samp_factor&3);

#if 0
    fprintf(stderr, "component_id = %d\n", cinfo->comp_info[0].component_id);
    fprintf(stderr, "component_index = %d\n", cinfo->comp_info[0].component_index);
    fprintf(stderr, "h_samp_factor = %d\n", cinfo->comp_info[0].h_samp_factor);
    fprintf(stderr, "v_samp_factor = %d\n", cinfo->comp_info[0].v_samp_factor);
    fprintf(stderr, "quant_tbl_no = %d\n", cinfo->comp_info[0].quant_tbl_no);
    switch(sampleFactor)
    {
    case 0xA: printf("Picture fmt : FORMAT_420\n"); break;
    case 0x9: printf("Picture fmt : FORMAT_422\n"); break;
    case 0x6: printf("Picture fmt : FORMAT_224\n"); break;
    case 0x5: printf("Picture fmt : FORMAT_444\n"); break;
    case 0x1:  printf("Picture fmt : FORMAT_400\n"); break;
    }
#endif

    unsigned int iScale = 0; /* 0-3 */
    unsigned int scale_w = m_width;
    unsigned int scale_h = m_height;
    // TODO try pixel number instead of width/height

    unsigned int max_h = 0;
    unsigned int max_w = 0;

    if(scale_w > MAX_RESOLUTION_W || scale_h > MAX_RESOLUTION_H)
    {

        if(CHIP_ID_1684 == get_bm_chip_id(img.card))
        {
            max_h = MAX_RESOLUTION_H;
            max_w = MAX_RESOLUTION_W;
        }
        else
        {
            max_h = MAX_RESOLUTION_H * 2;
            max_w = MAX_RESOLUTION_W * 2;
        }

        while (scale_w > max_w || scale_h > max_h)
        {
            iScale++;
            scale_w = m_width  >> iScale;
            scale_h = m_height >> iScale;
        }
    }

    m_device_id = img.card;

    int is_hw = determine_hw_decoding(cinfo);
    if (!is_hw)
    {
        printf("sampleFactor=%d, cinfo->num_components=%d (%dx%d, %dx%d, %dx%d)\n",
               sampleFactor, cinfo->num_components,
               cinfo->comp_info[0].h_samp_factor, cinfo->comp_info[0].v_samp_factor,
               cinfo->comp_info[1].h_samp_factor, cinfo->comp_info[1].v_samp_factor,
               cinfo->comp_info[2].h_samp_factor, cinfo->comp_info[2].v_samp_factor);

        /* For HW Not Supported jpeg format, use SW jpeg decoding */

        if(softDec(img,iScale,scale_w,scale_h) != 0){
            printf("Failed to decod!!!\n");
        }
    }
    else /* HW jpeg decoding */
    {
        BmJpuJPEGDecInfo info;
        uint8_t *buf = m_src_data;
        long    size = m_data_len;

#if defined(PROFILING_DEC)
        gettimeofday(&start, NULL);
#endif
        if ((0 == m_data_len) || (NULL == m_src_data))
        {
            printf("!!!Failed to get data!!!\n");
            close(DECODE_INIT);
            return false;
        }
        
        if(BM_JPU_DEC_RETURN_CODE_OK != bm_jpu_dec_load(BM_CARD_ID( m_device_id )))
        {
            fprintf(stderr, "Error! dec load failed, device id = %d \n",m_device_id);
            if((m_src_type != 0) && (m_src_data != NULL))
                free(m_src_data);
            m_src_data = NULL;
            return false;
        }
       
#ifndef _WIN32 //FIXME
        m_vpp_fd = g_vpp_fd[BM_CARD_ID( m_device_id )];
        assert(m_vpp_fd > 0);
#endif
        bmjpu_setup_logging();

        int dump_num = 0; //bm_jpu_jpeg_get_dump();
        if( dump_num > 0)
        {
            frame_dump_dec_num = dump_num;
            frame_dump_enc_num = dump_num;
        }
        /* Open the JPEG decoder */
        BmJpuDecOpenParams open_params;
        memset(&open_params, 0, sizeof(BmJpuDecOpenParams));
        open_params.min_frame_width = 0;
        open_params.min_frame_height = 0;
        open_params.max_frame_width = 0;
        open_params.max_frame_height = 0;
        open_params.device_index = img.card;

        open_params.chroma_interleave = BM_JPU_CHROMA_FORMAT_CBCR_SEPARATED;  //set all format to planar mode
        open_params.scale_ratio = iScale;
        open_params.bs_buffer_size = (size+BS_MASK)&(~BS_MASK); // in unit of 16k
#define JPU_PAGE_UNIT_SIZE 256 /* each page unit of jpu is 256 byte */
        /* Avoid the false alarm that bs buffer is empty (SA3SW-252) */
        if (open_params.bs_buffer_size - size < JPU_PAGE_UNIT_SIZE)
            open_params.bs_buffer_size += 16*1024;  // in unit of 16k
#undef JPU_PAGE_UNIT_SIZE

        BmJpuDecReturnCodes dec_ret;
        jpeg_decoder = NULL;
        
        dec_ret = bm_jpu_jpeg_dec_open(&(jpeg_decoder), &open_params, 0);
        if (dec_ret != BM_JPU_DEC_RETURN_CODE_OK)
        {
            fprintf(stderr, "Error! Failed to open bm_jpu_jpeg_dec_open() : %s\n",
            bm_jpu_dec_error_string(dec_ret));
            if((m_src_type != 0) && (m_src_data != NULL))
                free(m_src_data);
            m_src_data = NULL;
            close(DECODE_FAILED);
            return false;
        }

#if 0
        printf("encoded input frame:  size: %ld byte\n", size);
#endif

        /* Perform the actual JPEG decoding */
        dec_ret = bm_jpu_jpeg_dec_decode(jpeg_decoder, buf, size, 0, 0);
        if (dec_ret != BM_JPU_DEC_RETURN_CODE_OK)
        {
            //hw dec failed retry software
            if(m_retrySoftDec){
                if(this->softDec(img,iScale,scale_w,scale_h) == 0){
                    if((m_src_type != 0) && (m_src_data != NULL))
                        free(m_src_data);
                    m_src_data = NULL;
                    close(DECODE_FAILED);
                    return true;
                }
            }
            fprintf(stderr, "could not decode this JPEG image : %s\n",
                    bm_jpu_dec_error_string(dec_ret));
            if((m_src_type != 0) && (m_src_data != NULL))
                free(m_src_data);
            m_src_data = NULL;
            close(DECODE_FAILED);
            return false;
        }
        
        /* Get some information about the the frame
         * Note that the info is only available after calling bm_jpu_jpeg_dec_decode() */
        bm_jpu_jpeg_dec_get_info(jpeg_decoder, &info);

#if 0
        printf("aligned frame size: %u x %u\n",
               info.aligned_frame_width, info.aligned_frame_height);
        printf("actual frame size: %u x %u\n",
               info.actual_frame_width, info.actual_frame_height);
        printf("Y/Cb/Cr stride: %u/%u/%u\n",
               info.y_stride, info.cbcr_stride, info.cbcr_stride);
        printf("Y/Cb/Cr size: %u/%u/%u\n",
               info.y_size, info.cbcr_size, info.cbcr_size);
        printf("Y/Cb/Cr offset: %u/%u/%u\n",
               info.y_offset, info.cb_offset, info.cr_offset);
        printf("image format: %s\n",
               bm_jpu_image_format_string(info.image_format));
#endif

        if (info.framebuffer == NULL)
        {
            fprintf(stderr, "could not decode this JPEG image : no framebuffer returned\n");
            close(DECODE_FAILED);
            if((m_src_type != 0) && (m_src_data != NULL))
                free(m_src_data);
            m_src_data = NULL;
            return false;
        }

#if defined(PROFILING_DEC)
        gettimeofday(&end, NULL);
        t = (end.tv_sec*1000.0 + end.tv_usec/1000.0) - (start.tv_sec*1000.0 + start.tv_usec/1000.0);
        printf("HW dec 1 TICK: %.1fms\n", t);
        gettimeofday(&start, NULL);
#endif

        if(cinfo->jpeg_color_space == JCS_RGB)
        {
            info.image_format = BM_JPU_IMAGE_FORMAT_RGB;
            m_yuv_output = 0;
        }

        outputMat(img, info);
        img.fromhardware = 1;
#if 0
        static int ccc=0;
        dump_jpu_mat((char*)"jpudec", img, DUMP_OUT, FORMAT_RGB24, ccc++ );
#endif
#if defined(PROFILING_DEC)
        gettimeofday(&end, NULL);
        t = (end.tv_sec*1000.0 + end.tv_usec/1000.0) - (start.tv_sec*1000.0 + start.tv_usec/1000.0);
        printf("HW dec 2 TICK: %.1fms\n", t);
#endif
    }
    if (NULL != m_src_data)    /* shared both for hw/sw jpeg decoder */
    {
        if (0 != m_src_type)
            free(m_src_data);
        m_src_data = NULL;
    }

    close(DECODE_SUCCESS);

    return true;
}
/////////////////////// BMJpegEncoder ///////////////////

BMJpegEncoder::BMJpegEncoder()
:jpeg_encoder(NULL),m_device_id(0),m_vpp_fd(0)
{
    m_description = "JPEG files (*.jpeg;*.jpg;*.jpe)";
    m_buf_supported = true;
}

BMJpegEncoder::~BMJpegEncoder()
{
    close();
    bm_jpu_enc_unload(BM_CARD_ID( m_device_id ));
}

ImageEncoder BMJpegEncoder::newEncoder() const
{
    return makePtr<BMJpegEncoder>();
}

void BMJpegEncoder::close()
{
    /* Shut down the JPEG encoder */
    if (jpeg_encoder)
    {
        bm_jpu_jpeg_enc_close(jpeg_encoder);
        jpeg_encoder = NULL;
    }
}

static int copy_output_data(void *context, uint8_t const *data, uint32_t size,
                             BmJpuEncodedFrame *encoded_frame)
{
    (void)(encoded_frame);
    std::vector<uchar>* m_buf = (std::vector<uchar>*)context;
    m_buf->resize(size);

#if defined (FAST_MEMCPY)
    libyuv::fast_memcpy(&(*m_buf)[0], data, size);
#else
    memcpy(&(*m_buf)[0], data, size);
#endif

    if(frame_dump_enc_num > 0)
    {
#ifndef _WIN32
        char filename[255];
        pthread_t tid = pthread_self();
        sprintf(filename, "/tmp/pic%d_%u.jpg", frame_dump_enc_num--, (unsigned int)tid);
        FILE * fclr = fopen(filename, "wb");
        if(NULL!=fclr)
        {
            fwrite(data, 1, size, fclr);
            fclose(fclr);
        }
#endif
   }

   return 0;
}
static int write_output_data(void *context, uint8_t const *data, uint32_t size,
                             BmJpuEncodedFrame *encoded_frame)
{
    (void)(encoded_frame);
    FILE* fout = (FILE*)context;
    size_t sz = fwrite(data, 1, size, fout);
    if (sz != size) return -EIO;

    if(frame_dump_enc_num > 0)
    {
#ifndef _WIN32
        char filename[255];
        pthread_t tid = pthread_self();
        sprintf(filename, "/tmp/pic%d_%u.jpg", frame_dump_enc_num--, (unsigned int)tid);
        FILE * fclr = fopen(filename, "wb");
        if(NULL!=fclr)
        {
            fwrite(data, 1, size, fclr);
            fclose(fclr);
        }
#endif
   }

   return 0;
}

int BMJpegEncoder::fillFrameBuffer(Mat& img, BmJpuFramebuffer &framebuffer)
{
    int frame_total_size = 0;
 /* Initialize the input framebuffer */
    memset(&framebuffer, 0, sizeof(framebuffer));

    if (img.avOK()){
      framebuffer.y_stride    = img.avStep(4);
      framebuffer.cbcr_stride = img.avStep(5);
      framebuffer.y_offset    = 0;
      int frame_y_size = framebuffer.y_stride  * img.avRows();
      if ((framebuffer.y_stride & 0x7) || (framebuffer.cbcr_stride & 0x7))
      {
        fprintf(stderr, "Error! y/c stride [%d, %d] must be aligned by 8!\n", framebuffer.y_stride, framebuffer.cbcr_stride);
        return 0;
      }

      int format = img.avFormat();
      bm_uint64 addr_y_phy = img.avAddr(4);
      bm_uint64 addr_cb_phy = img.avAddr(5);
      bm_uint64 addr_cr_phy = img.avAddr(6);
      framebuffer.cb_offset = addr_cb_phy - addr_y_phy;
      framebuffer.cr_offset = addr_cr_phy - addr_y_phy;

      /* calcule y/cb/cr offset */
      int plane;
      int wscale[3], hscale[3], coff[3] = {0};
      int offset = img.data - img.datastart;
      int cx = offset % img.avStep(4);
      int cy = offset / img.avStep(4);

      plane = cv::av::get_scale_and_plane(format, wscale, hscale);
      if (plane > 1 && wscale[1] == 2)
        cx = cx & ~0x1;         // aligned due to uv downsample
      if (plane > 1 && hscale[1] == 2)
        cy = cy & ~0x1;         // aligned due to uv downsample

      for (int i = 0; i < plane; i++){
        coff[i] = cy / hscale[i] * img.avStep(4+i) + cx / wscale[i];
      }
      framebuffer.y_offset += coff[0];
      framebuffer.cb_offset += coff[1];
      framebuffer.cr_offset += coff[2];

      switch (format) {
      case AV_PIX_FMT_NV12:
      case AV_PIX_FMT_YUV420P:
          frame_total_size = frame_y_size * 3 / 2;
          break;
      case AV_PIX_FMT_NV16:
      case AV_PIX_FMT_YUV422P:
          frame_total_size = frame_y_size * 2;
          break;
      case AV_PIX_FMT_YUV444P:
      case AV_PIX_FMT_GBRP:
          frame_total_size = frame_y_size * 3;
          break;
      case AV_PIX_FMT_GRAY8:
          framebuffer.cb_offset = 0;
          framebuffer.cr_offset = 0;
          frame_total_size = frame_y_size;
          break;
      default:
          fprintf(stderr, "Error! unsupported input data format %d.\n", format);
          return 0;
      }

    }
    else if (img.u->addr && img.channels() == 1){ //GRAY
      framebuffer.y_stride = img.step[0];
      framebuffer.y_offset = img.data - img.datastart;
      framebuffer.cbcr_stride = 0;
      framebuffer.cb_offset = framebuffer.cr_offset = 0;
      frame_total_size = img.step[0] * img.rows;
    }
    else{
        fprintf(stderr, "%s AVFrame of Mat is invalid\n",__FUNCTION__);
        return 0;
    }

    return frame_total_size;
}
bool BMJpegEncoder::prepareDMABuffer(BmJpuFramebuffer &framebuffer, int width, int height, BmJpuImageFormat image_format,
                    const Mat& in_img, Mat& out_img, bm_device_mem_t &wrapped_mem)
{
    int frame_total_size;
    bm_uint64 luma_phy_addr;
    bm_uint64 offset = in_img.data - in_img.datastart;
    Mat bgr_img;
    if ((in_img.allocator && in_img.allocator != hal::getAllocator()) ||
        !(in_img.u && in_img.u->addr)
#ifdef USING_SOC
        || (!in_img.avOK() && ((in_img.u->addr+offset)&0x1f))
#endif
        || (!in_img.avOK() && ((in_img.step[0]/in_img.step[1]) & 0x7))
        )
    { // for python case
        bgr_img.allocator = hal::getAllocator();
        bgr_img.create(in_img.rows, (in_img.cols + 0x7) & ~0x7, in_img.type(), m_device_id);
        bgr_img = bgr_img(Rect(0, 0, in_img.cols, in_img.rows));

        in_img.copyTo(bgr_img);

#ifdef USING_SOC
        bgr_img.allocator->flush(bgr_img.u, bgr_img.u->size);
#else
        bmcv::uploadMat(bgr_img);
#endif
    } else {
        bgr_img = const_cast<Mat&>(in_img);
#ifdef USING_SOC
        bgr_img.allocator = hal::getAllocator();
        bgr_img.allocator->invalidate(bgr_img.u, bgr_img.u->size);
#else // PCIE Mode
        if (!bgr_img.avOK())
          bmcv::uploadMat(bgr_img);
#endif
    }

    if(bgr_img.avOK())
    {
        frame_total_size = fillFrameBuffer(bgr_img, framebuffer);
        luma_phy_addr = bgr_img.avAddr(4);
    }
    else
    {
#ifdef USING_SOC
        prepareInternalDMABuffer(framebuffer, width, height, image_format, bgr_img);
        return true;
#else
        if (image_format == BM_JPU_IMAGE_FORMAT_GRAY){
          out_img = bgr_img;
        }else{   // bgr format
          AVFrame *f = cv::av::create(height, width, m_device_id);  // yuv420 data
          out_img.create(f, m_device_id);

          std::vector<Rect> vrt;
          std::vector<Size> vsz;
          std::vector<Mat> out;
          Rect rt;
          Size sz;
          rt.x = rt.y = 0;
          rt.width = sz.width = bgr_img.cols;
          rt.height = sz.height = bgr_img.rows;
          vrt.push_back(rt);
          vsz.push_back(sz);
          out.push_back(out_img);

          CV_Assert(BM_SUCCESS == bmcv::convert(bgr_img, vrt, vsz, out, false, \
                CSC_RGB2YPbPr_BT601, NULL, BMCV_INTER_NEAREST));
          out_img = out.front();

          vrt.pop_back();
          vsz.pop_back();
          out.pop_back();
        }
        frame_total_size = fillFrameBuffer(out_img, framebuffer);
        luma_phy_addr = out_img.u->addr;
#endif
    }

    if(frame_total_size <= 0)
        return false;
    /* The input frames come in external DMA memory */
    /* The input frames already come in DMA / physically contiguous memory,
     * so the encoder can read from them directly. */
   
    wrapped_mem = bm_mem_from_device(luma_phy_addr, frame_total_size);
    framebuffer.dma_buffer = &wrapped_mem;

    return true;
}

bool BMJpegEncoder::prepareEncInputParams(BmJpuJPEGEncParams& encParams, int& bs_buffer_size,
                                          bool is_yuv_mat, const Mat& img, const std::vector<int>& params,
                                          void* file)
{
    int width, height, channels;
    int raw_size;
    BmJpuImageFormat out_image_format = BM_JPU_IMAGE_FORMAT_YUV420P;
    if(!is_yuv_mat)
    {
        width  = img.cols;
        height = img.rows;
        channels = img.channels();
        raw_size = width * height * 3/2; // will convert to yuv420 later
        if (channels != 3 && channels != 1)
        {
            fprintf(stderr, "Error! Unsupported img.channels %d.\n", channels);
            return false;
        }
        if (channels == 1)
        {
            out_image_format = BM_JPU_IMAGE_FORMAT_GRAY;
            raw_size = width * height;
        }
    }
    else
    {
        width  = img.cols;
        height = img.rows;
        raw_size = width * height;
        int format = img.avFormat();

        switch (format)
        {
        case AV_PIX_FMT_NV12:
            out_image_format = BM_JPU_IMAGE_FORMAT_NV12;
            raw_size = raw_size * 3/2;
            break;
        case AV_PIX_FMT_NV16:
            out_image_format = BM_JPU_IMAGE_FORMAT_NV16;
            raw_size = raw_size * 2;
            break;
        case AV_PIX_FMT_YUV444P:
        case AV_PIX_FMT_GBRP:
            out_image_format = BM_JPU_IMAGE_FORMAT_YUV444P;
            raw_size = raw_size * 3;
            break;
        case AV_PIX_FMT_GRAY8:
            out_image_format = BM_JPU_IMAGE_FORMAT_GRAY;
            raw_size = raw_size * 1;
            break;
        case AV_PIX_FMT_YUV420P:
            out_image_format = BM_JPU_IMAGE_FORMAT_YUV420P;
            raw_size = raw_size * 3/2;
            break;
        case AV_PIX_FMT_YUV422P:
            out_image_format = BM_JPU_IMAGE_FORMAT_YUV422P;
            raw_size = raw_size * 2;
            break;
        default:
            fprintf(stderr, "Error! Unsupported encoded image_format %d.\n", format);
            return false;
        }
    }

    if(width < 16 || height < 16)
    {
        fprintf(stderr, "Error! src size is abnormal:width=%d,height=%d\n",width,height);
        return false;
    }

    int quality = 85;
    int min_ratio = 1;   /* assume the min compression ratio is 1 */
    for( size_t i = 0; i < params.size(); i += 2 )  //get quality from params
    {
        if( params[i] == CV_IMWRITE_JPEG_QUALITY )
        {
            quality = params[i+1];
            quality = MIN(MAX(quality, 0), 100);
            if(quality >= 95)
                min_ratio = 1;  //for some picture it's hard to compress
        }
    }

    bs_buffer_size = raw_size/min_ratio;
    /* bitstream buffer size in unit of 16k */
    bs_buffer_size = (bs_buffer_size+BS_MASK)&(~BS_MASK);
    // if (bs_buffer_size >= 16*1023*1024)
    //     bs_buffer_size = 16*1023*1024;

    memset(&encParams, 0x0, sizeof(encParams));
    encParams.frame_width    = width;
    encParams.frame_height   = height;
    encParams.quality_factor = quality;
    encParams.image_format   = out_image_format;
    encParams.acquire_output_buffer = NULL;
    encParams.finish_output_buffer  = NULL;
    if (file)
    {
        encParams.write_output_data  = write_output_data;
        encParams.output_buffer_context = (void*)(file);
    }
    else
    {
#ifdef USING_SOC
        encParams.write_output_data  = copy_output_data;
        encParams.output_buffer_context = (void*)m_buf;
#else
        // To do for optimization
        encParams.write_output_data  = copy_output_data;
        encParams.output_buffer_context = (void*)m_buf;
        //encParams.acquire_output_buffer = NULL;
        //encParams.finish_output_buffer  = NULL;
#endif
    }
    return true;
}

#ifndef _WIN32
bool BMJpegEncoder::prepareInternalDMABuffer(BmJpuFramebuffer& framebuffer, int width, int height, BmJpuImageFormat image_format, const Mat& img)
{
    /* Initialize the input framebuffer */
    int framebuffer_alignment = 16;
    BmJpuColorFormat color_format = BM_JPU_COLOR_FORMAT_YUV420;
    BmJpuChromaFormat chroma_interleave = BM_JPU_CHROMA_FORMAT_CBCR_SEPARATED; // 0: I420; 1: NV12 : 2 NV21P_1_2n
    BmJpuFramebufferSizes calculated_sizes;
    int ret;

    // TODO: why chroma_interleave is always 0?
    switch (image_format) {
        case BM_JPU_IMAGE_FORMAT_YUV420P:
        case BM_JPU_IMAGE_FORMAT_NV12:
        case BM_JPU_IMAGE_FORMAT_NV21:
            color_format = BM_JPU_COLOR_FORMAT_YUV420;
            break;
        case BM_JPU_IMAGE_FORMAT_YUV422P:
        case BM_JPU_IMAGE_FORMAT_NV16:
        case BM_JPU_IMAGE_FORMAT_NV61:
            color_format = BM_JPU_COLOR_FORMAT_YUV422_HORIZONTAL;
            break;
        case BM_JPU_IMAGE_FORMAT_YUV444P:
            color_format = BM_JPU_COLOR_FORMAT_YUV444;
            break;
        case BM_JPU_IMAGE_FORMAT_GRAY:
            color_format = BM_JPU_COLOR_FORMAT_YUV400;
            break;
    }

    ret = bm_jpu_calc_framebuffer_sizes(width, height,
                                  framebuffer_alignment,
                                  image_format,
                                  &calculated_sizes);
    if (ret != 0) {
        fprintf(stderr, "Error! Failed to open bm_jpu_calc_framebuffer_sizes()\n");
        close();
        return false;
    }

    memset(&framebuffer, 0, sizeof(framebuffer));
    framebuffer.y_stride    = calculated_sizes.y_stride;
    framebuffer.cbcr_stride = calculated_sizes.cbcr_stride;
    framebuffer.y_offset    = 0;
    framebuffer.cb_offset   = calculated_sizes.y_size;
    framebuffer.cr_offset   = calculated_sizes.y_size + calculated_sizes.cbcr_size;

    /* Allocate a DMA buffer for the input pixels. In production,
     * it is typically more efficient to make sure the input frames
     * already come in DMA / physically contiguous memory, so the
     * encoder can read from them directly. */

    framebuffer.dma_buffer = (bm_device_mem_t*)malloc(sizeof(bm_device_mem_t));
    bm_handle_t handle = bm_jpu_enc_get_bm_handle(m_device_id);
    bm_status_t bm_ret =  bm_malloc_device_byte_heap_mask(handle, framebuffer.dma_buffer,
                                                HEAP_1_2, calculated_sizes.total_size);
    if (bm_ret != BM_SUCCESS) {
      printf("malloc device memory size = %d failed, ret = %d\n", calculated_sizes.total_size, ret);
      return -1;
    }
    if (framebuffer.dma_buffer == NULL)
    {
        fprintf(stderr, "could not allocate DMA buffer for input framebuffer\n");
        close();
        return false;
    }
#if defined(PROFILING_ENC)
    struct timeval start;
    struct timeval end;
    double t;
#endif

#if defined(PROFILING_ENC)
    gettimeofday(&start, NULL);
#endif

#if 0
        static int ccc=0;
        dump_jpu_mat((char*)"jpuenc", img, DUMP_IN, FORMAT_RGB24, ccc++ );
#endif

    bm_jpu_phys_addr_t p_phys_addr = bm_mem_get_device_addr(*(framebuffer.dma_buffer));
    unsigned long output_device_addr0 = (unsigned long)(p_phys_addr + framebuffer.y_offset);
    unsigned long output_device_addr1 = (unsigned long)(p_phys_addr + framebuffer.cb_offset);
    unsigned long output_device_addr2 = (unsigned long)(p_phys_addr + framebuffer.cr_offset);


    int dst_stride[4] = {0};
    dst_stride[0] = framebuffer.y_stride;
    dst_stride[1] = framebuffer.cbcr_stride;
    dst_stride[2] = dst_stride[1];

    bm_image_format_ext input_fmt ,output_fmt;
    input_fmt = FORMAT_BGR_PACKED;
    output_fmt = FORMAT_YUV420P;

    bmcv_resize_algorithm algorithm = BMCV_INTER_NEAREST;
    csc_type_t csc_type = CSC_RGB2YPbPr_BT601;

    if (color_format == BM_JPU_COLOR_FORMAT_YUV400)
    {
        csc_type = CSC_MAX_ENUM;
        algorithm = BMCV_INTER_LINEAR;
        input_fmt = FORMAT_GRAY;
        output_fmt = FORMAT_GRAY;
        dst_stride[1] = 0;
        dst_stride[2] = dst_stride[1];
    }
#ifdef HAVE_BMCV
    CV_Assert(BM_SUCCESS == enc_convert(img, output_device_addr0, output_device_addr1, output_device_addr2, dst_stride, height, width, input_fmt, output_fmt, algorithm, csc_type));
#endif

    if(((height & 1) || (width & 1)) && (color_format == BM_JPU_COLOR_FORMAT_YUV420))
    {
        uint8_t *src_rgb24, *dst_y, *dst_u, *dst_v;
        int src_stride_rgb24 = img.step[0];

        unsigned long long vmem = 0;
        bm_handle_t handle = bm_jpu_enc_get_bm_handle(m_device_id);

        bm_mem_mmap_device_mem(handle, framebuffer.dma_buffer, &vmem);
        uint8_t* p_virt_addr = (uint8_t*)vmem;
        if (p_virt_addr == NULL)
        {
            fprintf(stderr, "Error! bm_jpu_dma_buffer_map failed.\n");
            close();
            return false;
        }

        if(width & 1)
        {
            src_rgb24 = img.data + ((width -1)*3);
            dst_y = p_virt_addr + (width - 1);
            dst_u = p_virt_addr + framebuffer.cb_offset + ((width -1) >> 1) ;
            dst_v = p_virt_addr + framebuffer.cr_offset + ((width -1) >> 1) ;
            libyuv::RGB24ToJ420(src_rgb24, src_stride_rgb24,
                            dst_y, framebuffer.y_stride,
                            dst_u, framebuffer.cbcr_stride,
                            dst_v, framebuffer.cbcr_stride,
                            1, height);
        }
        if(height & 1)
        {
            src_rgb24 = img.data + src_stride_rgb24 * (height -1);
            dst_y = p_virt_addr + framebuffer.y_stride * (height -1);
            dst_u = p_virt_addr + framebuffer.cb_offset + (framebuffer.y_stride >> 1) * ((height -1) >> 1) ;
            dst_v = p_virt_addr + framebuffer.cr_offset + (framebuffer.y_stride >> 1) * ((height -1) >> 1) ;
            libyuv::RGB24ToJ420(src_rgb24, src_stride_rgb24,
                            dst_y, framebuffer.y_stride,
                            dst_u, framebuffer.cbcr_stride,
                            dst_v, framebuffer.cbcr_stride,
                            width, 1);
        }
        /* Flush cache to the DMA buffer */

        bm_mem_flush_device_mem(handle, framebuffer.dma_buffer);

        bm_mem_unmap_device_mem(handle, p_virt_addr, framebuffer.dma_buffer->size);
    }
#if 0
    static int ab= 0;
    unsigned long long vmem = 0;
    bm_handle_t handle = bm_jpu_enc_get_bm_handle(m_device_id);
    bm_mem_mmap_device_mem(handle, framebuffer.dma_buffer, &vmem);
    uint8_t* p_virt_addr2 = (uint8_t*)vmem;
    dump_jpu_framebuf((char*)"jpuenc", p_virt_addr2, framebuffer, width, height, DUMP_OUT, FORMAT_420, ab++);
    bm_mem_unmap_device_mem(handle, p_virt_addr2, framebuffer.dma_buffer->size);
#endif
#if defined(PROFILING_ENC)
    gettimeofday(&end, NULL);
    t = (end.tv_sec*1000.0 + end.tv_usec/1000.0) - (start.tv_sec*1000.0 + start.tv_usec/1000.0);
    printf("conversion time: %.1fms\n", t);
#endif
    return true;
}
#endif
bool BMJpegEncoder::write(const Mat& img, const std::vector<int>& params)
{
    (void)(params);
    int width, height, channels;
    Mat out_img;
    m_last_error.clear();

    struct fileWrapper
    {
        FILE* f;

        fileWrapper() : f(0) {}
        ~fileWrapper() { if(f) fclose(f); }
    };
    fileWrapper fw;

    m_device_id = img.card;
    if(BM_JPU_ENC_RETURN_CODE_OK != bm_jpu_enc_load(BM_CARD_ID( m_device_id )))
    {
        fprintf(stderr, "Error! enc load failed, device id = %d \n",m_device_id);
        return false;
    }
#ifndef _WIN32 //FIXME
    m_vpp_fd = g_vpp_fd[BM_CARD_ID( m_device_id )];
    assert(m_vpp_fd > 0);
#endif
    bmjpu_setup_logging();

    if (!m_buf)
    {
        fw.f = fopen(m_filename.c_str(), "wb");
        if (!fw.f)
        {
            fprintf(stderr, "Error! save file:%s open failed \n",m_filename.c_str());
            return false;
        }
    }

    int dump_num = 0; //bm_jpu_jpeg_get_dump();
    if( dump_num > 0)
    {
        frame_dump_enc_num = dump_num;
        frame_dump_dec_num = dump_num;
    }

    int bs_buffer_size;
    BmJpuFramebuffer framebuffer;

    bm_device_mem_t wrapped_mem;
    BmJpuJPEGEncParams enc_params; /* Set up the encoding parameters */

    bool b_yuv_mat = img.avOK();
    if (!prepareEncInputParams(enc_params, bs_buffer_size, b_yuv_mat, img, params, (void*)fw.f))
        return false;

    /* Open BM JPEG encoder */
    BmJpuEncReturnCodes enc_ret;
    enc_ret = bm_jpu_jpeg_enc_open(&(jpeg_encoder), 0, bs_buffer_size, BM_CARD_ID( m_device_id ));
    if (enc_ret != BM_JPU_ENC_RETURN_CODE_OK)
    {
        fprintf(stderr, "Error! Failed to open bm_jpu_jpeg_enc_open() :  %s\n",
                bm_jpu_enc_error_string(enc_ret));
        close();
        return false;
    }

    if(!prepareDMABuffer(framebuffer, enc_params.frame_width, enc_params.frame_height, enc_params.image_format, img, out_img, wrapped_mem))
        return false;

    /* Do the actual encoding */
    enc_ret = bm_jpu_jpeg_enc_encode(jpeg_encoder, &framebuffer,
                                     &enc_params, NULL, NULL);

    /* The framebuffer's DMA buffer isn't needed anymore, since we just
     * did the encoding, so deallocate it */

#ifdef USING_SOC
    if(!(img.avOK())){
        bm_handle_t handle = bm_jpu_enc_get_bm_handle(m_device_id);
        if (framebuffer.dma_buffer != NULL) {
            bm_free_device(handle, *(framebuffer.dma_buffer));
            free(framebuffer.dma_buffer);
            framebuffer.dma_buffer = NULL;
        }
    }
#endif

    if (enc_ret != BM_JPU_ENC_RETURN_CODE_OK)
    {
        fprintf(stderr, "could not encode this image : %s\n",
                bm_jpu_enc_error_string(enc_ret));
        close();
        return false;
    }

    close();

    return true;
}

}

#endif
/* End of file. */
