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
#include "grfmt_bmjpeg_1682.hpp"
#include "libyuv.h"

#ifdef VPP_BM1682

#ifdef _MSC_VER
//interaction between '_setjmp' and C++ object destruction is non-portable
#pragma warning(disable: 4611)
#endif

#include <stdio.h>
#include <stdarg.h>
#include <setjmp.h>
#include <sys/time.h>

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

#define FAST_MEMCPY

//#define PROFILING_DEC
//#define PROFILING_ENC
#define DEV_ID  0
static int frame_dump_enc_num = 0;
static int frame_dump_dec_num = 0;

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
    if ((0xA != sampleFactor) && (0x9 != sampleFactor) &&
        (0x5 != sampleFactor) && (0x1 != sampleFactor))
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
        cbcr_width = width/2;
        cbcr_height = height/2;
    }
    else if(FORMAT_422 == sampleFactor)
    {
        cbcr_width = width/2;
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

        write_plane(addr_y, width, height, framebuffer.y_stride, fout);            //write y
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

        write_plane(addr_y, width, height, info.y_stride, fout);            //write y
        write_plane(addr_cb, cbcr_width, cbcr_height, info.cbcr_stride, fout); //write u
        write_plane(addr_cr, cbcr_width, cbcr_height, info.cbcr_stride, fout); //write v

        if(NULL!=fout)
            fclose(fout);
    }
}
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
{
    m_signature = "\xFF\xD8\xFF";
    m_state = NULL;
    m_f = NULL;
    m_buf_supported = true;
    jpeg_decoder = NULL;

    m_src_data = NULL;
    m_data_len = 0;
    m_src_type = 0;

    b_bmjpu_dec_load = false;
}

BMJpegDecoder::~BMJpegDecoder()
{
    close();

    if (b_bmjpu_dec_load)
    {
        bm_jpu_dec_unload(0);
        b_bmjpu_dec_load = false;
    }
}

void  BMJpegDecoder::close()
{
    if (jpeg_decoder && !m_yuv_output)
    {
        bm_jpu_jpeg_dec_close(jpeg_decoder);
        jpeg_decoder = NULL;
    }

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
    close( );

    JpegState* state = new JpegState;
    m_state = state;
    state->cinfo.err = jpeg_std_error(&state->jerr.pub);
    state->jerr.pub.error_exit = error_exit;
    m_src_type = 0;

    int ret = setjmp(state->jerr.setjmp_buffer);
    if (ret != 0)
    {
        close();
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
            close();
            return false;
        }

        fseek(m_f, 0, SEEK_END);
        int fileSize = ftell( m_f );
        fseek(m_f, 0, SEEK_SET);

        m_src_data = (uint8_t *)malloc(fileSize);
        if (NULL == m_src_data)
        {
            printf("Error! malloc failed.\n");
            close();
            return false;
        }

        ret = fread(m_src_data, sizeof(uint8_t), fileSize, m_f);
        if (ret != fileSize)
        {
            printf("Error! fread failed.\n");
            close();
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
        close();
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

  UMatData *u = jpeg_opaque->data;

  bm_jpu_dma_buffer_unmap(jpeg_decoder->raw_frame.framebuffer->dma_buffer);

  /* Decoded frame is no longer needed,
   * so inform the decoder that it can reclaim it */
  bm_jpu_jpeg_dec_frame_finished(jpeg_decoder, jpeg_decoder->raw_frame.framebuffer);
  bm_jpu_jpeg_dec_close(jpeg_decoder);

  if (u) delete u;
  if (jpeg_opaque) delete jpeg_opaque;
 }

AVFrame *BMJpegDecoder::fillAVFrame(BmJpuJPEGDecInfo& info)
{
  if (info.color_format == BM_JPU_COLOR_FORMAT_YUV422_VERTICAL
   || info.color_format == BM_JPU_COLOR_FORMAT_RGB)
  {
    fprintf(stderr, "format error!, the format is not supported!\n");
    return NULL;
  }

  MatAllocator *a = av::getAllocator();
  UMatData *u = new UMatData(a);

  AVFrame *f = av_frame_alloc();
  f->height = info.actual_frame_height;
  f->width = info.actual_frame_width;

  BmJpuFramebuffer *fb = info.framebuffer;
  uint8_t *data = (uint8_t *)bm_jpu_dma_buffer_get_physical_address(fb->dma_buffer);
  int size = bm_jpu_dma_buffer_get_size(fb->dma_buffer);

  uint8_t *vddr = bm_jpu_dma_buffer_map(fb->dma_buffer, BM_JPU_MAPPING_FLAG_READ|BM_JPU_MAPPING_FLAG_WRITE);

  UMatOpaque *jpeg_opaque = new UMatOpaque;
  jpeg_opaque->magic_number = MAGIC_MAT;
  jpeg_opaque->data = u;

  jpeg_decoder->opaque = jpeg_opaque;
  f->opaque = jpeg_opaque;

  u->data = vddr;
  u->size = size;

  f->buf[4] = av_buffer_create(data, size, av_buffer_release, jpeg_decoder, AV_BUFFER_FLAG_READONLY);
  f->data[4] = data + fb->y_offset;
  f->data[0] = vddr + fb->y_offset;
  f->linesize[4] = fb->y_stride;
  f->linesize[0] = fb->y_stride;

  if (info.color_format == BM_JPU_COLOR_FORMAT_YUV420) {
    f->data[5] = data + fb->cb_offset;
    f->data[1] = vddr + fb->cb_offset;
    f->linesize[5] = fb->cbcr_stride;
    f->linesize[1] = fb->cbcr_stride;

    if (info.chroma_interleave) {
      f->format = AV_PIX_FMT_NV12;
    } else {
      f->data[6] = data + fb->cr_offset;
      f->data[2] = vddr + fb->cr_offset;
      f->linesize[6] = fb->cbcr_stride;
      f->linesize[2] = fb->cbcr_stride;

      f->format = AV_PIX_FMT_YUV420P;
    }
  }
  else if (info.color_format == BM_JPU_COLOR_FORMAT_YUV422_HORIZONTAL)
  {
    f->data[5] = data + fb->cb_offset;
    f->data[1] = vddr + fb->cb_offset;
    f->linesize[5] = fb->cbcr_stride;
    f->linesize[1] = fb->cbcr_stride;

    if (info.chroma_interleave) {
      f->format = AV_PIX_FMT_NV16;
    } else {
      f->data[6] = data + fb->cr_offset;
      f->data[2] = vddr + fb->cr_offset;
      f->linesize[6] = fb->cbcr_stride;
      f->linesize[2] = fb->cbcr_stride;

      f->format = AV_PIX_FMT_YUV422P;
    }
  }
  else if (info.color_format == BM_JPU_COLOR_FORMAT_YUV444)
  {
    f->data[5] = data + fb->cb_offset;
    f->data[1] = vddr + fb->cb_offset;
    f->linesize[5] = fb->cbcr_stride;
    f->linesize[1] = fb->cbcr_stride;

    f->data[6] = data + fb->cr_offset;
    f->data[2] = vddr + fb->cr_offset;
    f->linesize[6] = fb->cbcr_stride;
    f->linesize[2] = fb->cbcr_stride;

    f->format = AV_PIX_FMT_YUV444P;
  }
  else if (info.color_format == BM_JPU_COLOR_FORMAT_YUV400)
  {
    f->format = AV_PIX_FMT_GRAY8;
  }

  return f;
}

int BMJpegDecoder::outputMat(Mat& img, BmJpuJPEGDecInfo &info)
{
    uint8_t* p_virt_addr;

    if(!m_yuv_output)
    {
        BmJpuFramebuffer *framebuffer = info.framebuffer;
        bool color = img.channels() > 1;
        if(color)
        {
            unsigned long phys_addr;
            switch (info.color_format)
            {
                case BM_JPU_COLOR_FORMAT_YUV420:
                {
                    phys_addr = bm_jpu_dma_buffer_get_physical_address(framebuffer->dma_buffer);

                    IplImage in;
                    in.srcFmt = FMT_SRC_I420;
                    in.width  = info.actual_frame_width;
                    in.height = info.actual_frame_height;
                    in.step   = info.y_stride;

                    in.addr0  = (unsigned long)(phys_addr + info.y_offset);
                    in.addr1  = (unsigned long)(phys_addr + info.cb_offset);
                    in.addr2  = (unsigned long)(phys_addr + info.cr_offset);

                    img = cv::vpp::iplImageToMat(&in, (char)1);

                    if(frame_dump_dec_num > 0)
                    {
                        p_virt_addr = bm_jpu_dma_buffer_map(framebuffer->dma_buffer,
                                                            BM_JPU_MAPPING_FLAG_READ);
                        dump_jpu_virt_addr((char*)"jpudec", p_virt_addr, info, DUMP_IN, FORMAT_420, frame_dump_dec_num);
                        bm_jpu_dma_buffer_unmap(framebuffer->dma_buffer);
                        dump_jpu_mat((char*)"jpudec", img, DUMP_OUT, FORMAT_420,frame_dump_dec_num);
                        frame_dump_dec_num--;
                    }
                    break;
                }
                case BM_JPU_COLOR_FORMAT_YUV422_HORIZONTAL:
                {
                    /* Map the DMA buffer of the decoded picture */
                    p_virt_addr = bm_jpu_dma_buffer_map(framebuffer->dma_buffer,
                                                   (BM_JPU_MAPPING_FLAG_READ|BM_JPU_MAPPING_FLAG_WRITE));

                    uint8_t* u = p_virt_addr + info.cb_offset;
                    uint8_t* v = p_virt_addr + info.cr_offset;
                    int stride_u = info.cbcr_stride;
                    int stride_v = info.cbcr_stride;

                    int width  = info.actual_frame_width;
                    int height = info.actual_frame_height;
                    if(frame_dump_dec_num > 0)
                    {
                        dump_jpu_virt_addr((char*)"jpudec", p_virt_addr, info, DUMP_IN, FORMAT_422, frame_dump_dec_num);
                    }

                    libyuv::I422ToI420(NULL, 0,
                                       u, stride_u,
                                       v, stride_v,
                                       NULL, 0,
                                       u, stride_u,
                                       v, stride_v,
                                       width, height);

                    /* Flush data from cache to dma buffer for VPP */
                    bm_jpu_dma_buffer_flush(framebuffer->dma_buffer);

                    /* Unmap the DMA buffer of the decoded picture */
                    bm_jpu_dma_buffer_unmap(framebuffer->dma_buffer);

                    IplImage in;
                    in.srcFmt = FMT_SRC_I420;
                    in.width  = width;
                    in.height = height;
                    in.step   = info.y_stride;

                    phys_addr = bm_jpu_dma_buffer_get_physical_address(framebuffer->dma_buffer);
                    in.addr0  = (unsigned long)(phys_addr + info.y_offset);
                    in.addr1  = (unsigned long)(phys_addr + info.cb_offset);
                    in.addr2  = (unsigned long)(phys_addr + info.cr_offset);

                    /* J420 to BGR */
                    img = cv::vpp::iplImageToMat(&in, (char)1);
                    if(frame_dump_dec_num > 0)
                    {
                        dump_jpu_mat((char*)"jpudec", img, DUMP_OUT, FORMAT_422, frame_dump_dec_num);
                        frame_dump_dec_num--;
                    }
                    break;
                }
                case BM_JPU_COLOR_FORMAT_YUV444:
                case BM_JPU_COLOR_FORMAT_RGB:
                {
                    p_virt_addr = bm_jpu_dma_buffer_map(framebuffer->dma_buffer,
                                               (BM_JPU_MAPPING_FLAG_READ|BM_JPU_MAPPING_FLAG_WRITE));

                    uint8_t* u = p_virt_addr + info.cb_offset;
                    uint8_t* v = p_virt_addr + info.cr_offset;
                    int src_stride_u = info.cbcr_stride;
                    int src_stride_v = info.cbcr_stride;
                    int dst_stride_u = (info.cbcr_stride+1)>>1;
                    int dst_stride_v = (info.cbcr_stride+1)>>1;

                    int width  = info.actual_frame_width;
                    int height = info.actual_frame_height;

                    if(frame_dump_dec_num > 0)
                    {
                       dump_jpu_virt_addr((char*)"jpudec", p_virt_addr, info, DUMP_IN, FORMAT_444, frame_dump_dec_num);
                    }

                    IplImage in;
                    if(info.color_format == BM_JPU_COLOR_FORMAT_RGB)
                    {
                        in.srcFmt = FMT_SRC_RGBP;
                    }
                    else
                    {
                        libyuv::I444ToI420(NULL, 0,
                                   u, src_stride_u,
                                   v, src_stride_v,
                                   NULL, 0,
                                   u, dst_stride_u,
                                   v, dst_stride_v,
                                   width, height);

                        /* Flush data from cache to DMA buffer for VPP */
                        bm_jpu_dma_buffer_flush(framebuffer->dma_buffer);
                        in.srcFmt = FMT_SRC_I420;
                    }
                    /* Unmap the DMA buffer of the decoded picture */
                    bm_jpu_dma_buffer_unmap(framebuffer->dma_buffer);

                    in.width  = width;
                    in.height = height;
                    in.step   = info.y_stride;

                    phys_addr = bm_jpu_dma_buffer_get_physical_address(framebuffer->dma_buffer);
                    in.addr0  = (unsigned long)(phys_addr + info. y_offset);
                    in.addr1  = (unsigned long)(phys_addr + info.cb_offset);
                    in.addr2  = (unsigned long)(phys_addr + info.cr_offset);

                    /* J420/RGBP to BGR */
                    img = cv::vpp::iplImageToMat(&in, (char)1);

                    if(frame_dump_dec_num > 0)
                    {
                        dump_jpu_mat((char*)"jpudec", img, DUMP_OUT, FORMAT_444, frame_dump_dec_num);
                        frame_dump_dec_num--;
                    }
                    break;
                }
                case BM_JPU_COLOR_FORMAT_YUV400:
                {
                    /* Map the DMA buffer of the decoded picture */
                    p_virt_addr = bm_jpu_dma_buffer_map(framebuffer->dma_buffer, BM_JPU_MAPPING_FLAG_READ);
                    Mat dst(info.actual_frame_height, info.actual_frame_width, CV_8UC3);
                    uint8_t* dst_rgb24 = dst.data;
                    int dst_stride_rgb24 = dst.step[0];
                    int width  = info.actual_frame_width;
                    int height = info.actual_frame_height;

                    libyuv::J400ToRGB24(p_virt_addr + info. y_offset, info.y_stride,
                                    dst_rgb24, dst_stride_rgb24,
                                    width, height);

                    img = dst;
                    break;
                }
                default:
                    fprintf(stderr, "Error! Not Supported output format : %d\n", info.color_format);
            }
        }
        else
        {
            Mat dst(info.actual_frame_height, info.actual_frame_width, CV_8UC1);
            p_virt_addr = bm_jpu_dma_buffer_map(framebuffer->dma_buffer,
                                                   BM_JPU_MAPPING_FLAG_READ);
            uint8_t* addr_y  = p_virt_addr + info.y_offset;

            libyuv::CopyPlane(addr_y, info.y_stride,
                                 dst.data, dst.step[0],
                                 info.actual_frame_width, info.actual_frame_height);

            bm_jpu_dma_buffer_unmap(framebuffer->dma_buffer);
            img = dst;
        }

    }
    else
    {
#if 0
        if(info.color_format == BM_JPU_COLOR_FORMAT_RGB)
        {
            BmJpuFramebuffer *framebuffer = info.framebuffer;
            p_virt_addr = bm_jpu_dma_buffer_map(framebuffer->dma_buffer,
                                               (BM_JPU_MAPPING_FLAG_READ|BM_JPU_MAPPING_FLAG_WRITE));
            dump_jpu_virt_addr((char*)"jpudec", p_virt_addr, info, DUMP_IN, FORMAT_444, 0);
            bm_jpu_dma_buffer_unmap(framebuffer->dma_buffer);
        }
#endif
        AVFrame *f = fillAVFrame(info);
        if(f){
            img.release();
            img.create(f, 0);
        }
    }
    return 0;
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
        close();
        fprintf(stderr, "Error! img info abnomal m_width=%d,m_height=%d \n",m_width,m_height);
        return false;
    }

    jpeg_decompress_struct* cinfo = &((JpegState*)m_state)->cinfo;
    JpegErrorMgr* jerr = &((JpegState*)m_state)->jerr;
    JSAMPARRAY buffer = 0;

    int ret = setjmp(jerr->setjmp_buffer);
    if (ret)
    {
        close();
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
    default:  printf("Picture fmt : FORMAT_400\n"); break;
    }
#endif

    unsigned int iScale = 0; /* 0-3 */
    unsigned int scale_w = m_width;
    unsigned int scale_h = m_height;
    // TODO try pixel number instead of width/height
    while (scale_w > MAX_RESOLUTION_W || scale_h > MAX_RESOLUTION_H)
    {
        iScale++;
        scale_w = m_width  >> iScale;
        scale_h = m_height >> iScale;
        if (iScale >= 3)
            break;
    }

    int is_hw = determine_hw_decoding(cinfo);
    if (!is_hw)
    {
        printf("sampleFactor=%d, cinfo->num_components=%d (%dx%d, %dx%d, %dx%d)\n",
               sampleFactor, cinfo->num_components,
               cinfo->comp_info[0].h_samp_factor, cinfo->comp_info[0].v_samp_factor,
               cinfo->comp_info[1].h_samp_factor, cinfo->comp_info[1].v_samp_factor,
               cinfo->comp_info[2].h_samp_factor, cinfo->comp_info[2].v_samp_factor);

        /* For HW Not Supported jpeg format, use SW jpeg decoding */
#if defined(PROFILING_DEC)
        gettimeofday(&start, NULL);
#endif
        bool color = img.channels() > 1;
        img.create( m_height, m_width, img.type() );

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
            Mat dst;
            Size dsize = Size(scale_w, scale_h);
            cv::resize(img, dst, dsize, 0, 0, INTER_NEAREST);
            img = dst;
        }

#if defined(PROFILING_DEC)
        gettimeofday(&end, NULL);
        t = (end.tv_sec*1000.0 + end.tv_usec/1000.0) - (start.tv_sec*1000.0 + start.tv_usec/1000.0);
        printf("SW dec: %.1fms\n", t);
        gettimeofday(&start, NULL);
#endif
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
            close();
            return false;
        }

        if (!b_bmjpu_dec_load)
        {
            bm_jpu_dec_load(DEV_ID);
            b_bmjpu_dec_load = true;

            bmjpu_setup_logging();
        }

#if 0
        printf("m_width =%d\n", m_width);
        printf("m_height=%d\n", m_height);
#endif

        int dump_num = bm_jpu_jpeg_get_dump();
        if( dump_num > 0)
        {
            frame_dump_dec_num = dump_num;
            frame_dump_enc_num = dump_num;
            cv::vpp::setDump(dump_num);
        }
        /* Open the JPEG decoder */
        BmJpuDecOpenParams open_params;
        memset(&open_params, 0, sizeof(BmJpuDecOpenParams));
        open_params.frame_width  = 0;
        open_params.frame_height = 0;
        open_params.device_index = DEV_ID;
        if(!m_yuv_output)
            open_params.chroma_interleave = 0; // 0: non-interleave
        else{
            if(sampleFactor == FORMAT_420 || sampleFactor == FORMAT_422)
                open_params.chroma_interleave = 1; // 1: interleave
            else
                open_params.chroma_interleave = 0; // 0: non-interleave
        }
        open_params.scale_ratio = iScale;
        open_params.bs_buffer_size = (size+BS_MASK)&(~BS_MASK); // in unit of 16k
#define JPU_PAGE_UNIT_SIZE 256 /* each page unit of jpu is 256 byte */
        /* Avoid the false alarm that bs buffer is empty (SA3SW-252) */
        if (open_params.bs_buffer_size - size < JPU_PAGE_UNIT_SIZE)
            open_params.bs_buffer_size += 16*1024;  // in unit of 16k
#undef JPU_PAGE_UNIT_SIZE

        BmJpuDecReturnCodes dec_ret;
        jpeg_decoder = NULL;
        dec_ret = bm_jpu_jpeg_dec_open(&(jpeg_decoder), &open_params, NULL, 0);
        if (dec_ret != BM_JPU_DEC_RETURN_CODE_OK)
        {
            fprintf(stderr, "Error! Failed to open bm_jpu_jpeg_dec_open() : %s\n",
            bm_jpu_dec_error_string(dec_ret));
            close();
            return false;
        }

#if 0
        printf("encoded input frame:  size: %ld byte\n", size);
#endif

        /* Perform the actual JPEG decoding */
        dec_ret = bm_jpu_jpeg_dec_decode(jpeg_decoder, buf, size);
        if (dec_ret != BM_JPU_DEC_RETURN_CODE_OK)
        {
            fprintf(stderr, "could not decode this JPEG image : %s\n",
                    bm_jpu_dec_error_string(dec_ret));
            close();
            return false;
        }

        /* Input data is not needed anymore, so free the input buffer */

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
        printf("color format: %s\n",
               bm_jpu_color_format_string(info.color_format));
        printf("chroma interleave: %d\n",
               info.chroma_interleave);
#endif

        if (info.framebuffer == NULL)
        {
            fprintf(stderr, "could not decode this JPEG image : no framebuffer returned\n");
            close();
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
            info.color_format = BM_JPU_COLOR_FORMAT_RGB;
            m_yuv_output = 0;
        }

        outputMat(img, info);

        /* Decoded frame is no longer needed,
         * so inform the decoder that it can reclaim it */
        bm_jpu_jpeg_dec_frame_finished(jpeg_decoder, info.framebuffer);

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

    close();

    return true;
}
/////////////////////// BMJpegEncoder ///////////////////

BMJpegEncoder::BMJpegEncoder()
{
    m_description = "JPEG files (*.jpeg;*.jpg;*.jpe)";
    m_buf_supported = true;
    jpeg_encoder = NULL;

    b_bmjpu_enc_load = false;
}

BMJpegEncoder::~BMJpegEncoder()
{
    close();

    if (b_bmjpu_enc_load)
    {
        bm_jpu_enc_unload(DEV_ID);
        b_bmjpu_enc_load = false;
    }
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
        char filename[255];
        pthread_t tid = pthread_self();
        sprintf(filename, "/data/pic%d_%u.jpg", frame_dump_enc_num--, (unsigned int)tid);
        FILE * fclr = fopen(filename, "wb");
        if(NULL!=fclr)
        {
            fwrite(data, 1, size, fclr);
            fclose(fclr);
        }
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
        char filename[255];
        pthread_t tid = pthread_self();
        sprintf(filename, "/data/pic%d_%u.jpg", frame_dump_enc_num--, (unsigned int)tid);
        FILE * fclr = fopen(filename, "wb");
        if(NULL!=fclr)
        {
            fwrite(data, 1, size, fclr);
            fclose(fclr);
        }
   }

   return 0;
}

bool BMJpegEncoder::prepareExternalDMABuffer(BmJpuFramebuffer &framebuffer, int width, int height,
                                             const Mat& img, bm_device_mem_t &wrapped_dma_mem)
{
    int frame_total_size;
    /* Initialize the input framebuffer */
    memset(&framebuffer, 0, sizeof(framebuffer));
    framebuffer.y_stride    = img.avStep(4);
    framebuffer.cbcr_stride = img.avStep(5);
    framebuffer.y_offset    = 0;
    int frame_y_size = framebuffer.y_stride  * height;

    int format = img.avFormat();
    ulong addr_y_phy = img.avAddr(4);
    ulong addr_cb_phy = img.avAddr(5);
    ulong addr_cr_phy = img.avAddr(6);
    framebuffer.cb_offset = addr_cb_phy - addr_y_phy;
    framebuffer.cr_offset = addr_cr_phy - addr_y_phy;

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
        return false;
    }

    //  flush external DMA buffer
    if(img.avOK())
        av::flush((AVFrame*)img.u->frame);
    else
    {
        fprintf(stderr, "%s AVFrame of Mat is invalid\n",__FUNCTION__);
        return false;
    }

    /* The input frames come in external DMA memory */
    /* The input frames already come in DMA / physically contiguous memory,
     * so the encoder can read from them directly. */
    // bm_jpu_init_wrapped_dma_buffer(&wrapped_dma_buffer);

    /* The EXTERNAL DMA buffer filled with frame data */
    wrapped_dma_mem.u.device.device_addr = img.avAddr(4);
    /* The size of EXTERNAL DMA buffer */
    wrapped_dma_mem.size = frame_total_size;

    framebuffer.dma_buffer = &wrapped_dma_mem;

    return true;
}

bool BMJpegEncoder::prepareInternalDMABuffer(BmJpuFramebuffer& framebuffer, int width, int height, BmJpuColorFormat color_format, const Mat& img)
{
    /* Initialize the input framebuffer */
    int framebuffer_alignment = 16;
    int chroma_interleave = 0; // 0: I420; 1: NV12 : 2 NV21
    BmJpuFramebufferSizes calculated_sizes;
    bm_jpu_calc_framebuffer_sizes(color_format,
                                  width, height,
                                  framebuffer_alignment,
                                  chroma_interleave,
                                  &calculated_sizes);

#if 0
    printf("\twidth =%d\n", width);
    printf("\theight=%d\n", height);
    printf("calculated_sizes:\n");
    printf("\tchroma_interleave=%d\n", calculated_sizes.chroma_interleave);
    printf("\taligned_frame_width =%d\n", calculated_sizes.aligned_frame_width);
    printf("\taligned_frame_height=%d\n", calculated_sizes.aligned_frame_height);
    printf("\ty_stride=%d\n", calculated_sizes.y_stride);
    printf("\tcbcr_stride=%d\n", calculated_sizes.cbcr_stride);
    printf("\ty_size=%d\n", calculated_sizes.y_size);
    printf("\tcbcr_size=%d\n", calculated_sizes.cbcr_size);
    printf("\ttotal_size=%d\n", calculated_sizes.total_size);
#endif

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
    framebuffer.dma_buffer = bm_jpu_dma_buffer_allocate(bm_jpu_enc_get_default_allocator(),
                                                        calculated_sizes.total_size, 1, BM_JPU_ALLOCATION_FLAG_DEFAULT, 0);
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
    /* Load the input pixels into the DMA buffer */
    uint8_t* p_virt_addr = bm_jpu_dma_buffer_map(framebuffer.dma_buffer, BM_JPU_MAPPING_FLAG_WRITE);
    if (p_virt_addr == NULL)
    {
        fprintf(stderr, "Error! bm_jpu_dma_buffer_map failed.\n");
        close();
        return false;
    }
    uint8_t* dst_y = p_virt_addr + framebuffer. y_offset;
    uint8_t* dst_u = p_virt_addr + framebuffer.cb_offset;
    uint8_t* dst_v = p_virt_addr + framebuffer.cr_offset;
    int dst_stride_y = framebuffer.   y_stride;
    int dst_stride_u = framebuffer.cbcr_stride;
    int dst_stride_v = framebuffer.cbcr_stride;
    if (color_format == BM_JPU_COLOR_FORMAT_YUV420)
    {
        uint8_t* src_rgb24   = img.data;
        int src_stride_rgb24 = img.step[0];

        if(frame_dump_enc_num > 0)
        {
            dump_jpu_mat((char*)"jpuenc", img, DUMP_IN, FORMAT_RGB24, frame_dump_dec_num);
        }
        /* BGR to J420 */
        libyuv::RGB24ToJ420(src_rgb24, src_stride_rgb24,
                            dst_y, dst_stride_y,
                            dst_u, dst_stride_u,
                            dst_v, dst_stride_v,
                            width, height);

    }
    else if (color_format == BM_JPU_COLOR_FORMAT_YUV400)
    {
        uint8_t* src_y   = img.data;
        int src_stride_y = img.step[0];

        libyuv::CopyPlane(src_y, src_stride_y,
                          dst_y, dst_stride_y,
                          width, height);
    }
    else
    {
        fprintf(stderr, "Error! Unsupported color_format %d.\n", color_format);
        close();
        return false;
    }

    if( frame_dump_enc_num > 0 )
    {
        dump_jpu_framebuf((char*)"jpuenc", p_virt_addr, framebuffer, width, height, DUMP_OUT, FORMAT_420, frame_dump_enc_num);
    }

    /* Flush cache to the DMA buffer */
    bm_jpu_dma_buffer_flush(framebuffer.dma_buffer);

    /* Unmap the DMA buffer */
    bm_jpu_dma_buffer_unmap(framebuffer.dma_buffer);

#if defined(PROFILING_ENC)
    gettimeofday(&end, NULL);
    t = (end.tv_sec*1000.0 + end.tv_usec/1000.0) - (start.tv_sec*1000.0 + start.tv_usec/1000.0);
    printf("conversion time: %.1fms\n", t);
#endif
    return true;
}

bool BMJpegEncoder::prepareEncInputParams(BmJpuJPEGEncParams& encParams, int& bs_buffer_size,
                                          bool is_yuv_mat, const Mat& img, const std::vector<int>& params,
                                          void* file)
{
    int width, height, channels;
    int chroma_interleave = 0;
    BmJpuColorFormat out_color_format  = BM_JPU_COLOR_FORMAT_YUV420;
    if(!is_yuv_mat)
    {
        width  = img.cols;
        height = img.rows;
        channels = img.channels();
        if (channels != 3 && channels != 1)
        {
            fprintf(stderr, "Error! Unsupported img.channels %d.\n", channels);
            return false;
        }
        if (channels == 1)
        {
            out_color_format = BM_JPU_COLOR_FORMAT_YUV400;
        }
    }
    else
    {
        width  = img.avCols();
        height = img.avRows();
        int format = img.avFormat();

        switch (format)
        {
        case AV_PIX_FMT_NV12:
            out_color_format = BM_JPU_COLOR_FORMAT_YUV420;
            chroma_interleave = 1;
            break;
        case AV_PIX_FMT_NV16:
            out_color_format = BM_JPU_COLOR_FORMAT_YUV422_HORIZONTAL;
            chroma_interleave = 1;
            break;
        case AV_PIX_FMT_YUV444P:
        case AV_PIX_FMT_GBRP:
            out_color_format = BM_JPU_COLOR_FORMAT_YUV444;
            break;
        case AV_PIX_FMT_GRAY8:
            out_color_format = BM_JPU_COLOR_FORMAT_YUV400;
            break;
        case AV_PIX_FMT_YUV420P:
            out_color_format = BM_JPU_COLOR_FORMAT_YUV420;
            break;
        case AV_PIX_FMT_YUV422P:
            out_color_format = BM_JPU_COLOR_FORMAT_YUV422_HORIZONTAL;
            break;
        default:
            fprintf(stderr, "Error! Unsupported encoded color_format %d.\n", format);
            return false;
        }
    }

    if(width <= 0 || height <= 0)
    {
        fprintf(stderr, "Error! src size abnormal:width=%d,height=%d\n",width,height);
        return false;
    }

    int raw_size = width*height*3/2; /* the size for YUV420 */
    int quality = 85;
    int min_ratio = 2;   /* assume the min compression ratio is 2 */
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
    if (bs_buffer_size >= 16*1023*1024)
        bs_buffer_size = 16*1023*1024;

    memset(&encParams, 0x0, sizeof(encParams));
    encParams.frame_width    = width;
    encParams.frame_height   = height;
    encParams.quality_factor = quality;
    encParams.color_format   = out_color_format;
    encParams.chroma_interleave = chroma_interleave;
    encParams.acquire_output_buffer = NULL;
    encParams.finish_output_buffer  = NULL;
    if (file)
    {
        encParams.write_output_data  = write_output_data;
        encParams.output_buffer_context = (void*)(file);
    }
    else
    {
        encParams.write_output_data  = copy_output_data;
        encParams.output_buffer_context = (void*)m_buf;
    }
    return true;
}

bool BMJpegEncoder::write(const Mat& img, const std::vector<int>& params)
{
    (void)(params);
    int width, height, channels;
    m_last_error.clear();

    struct fileWrapper
    {
        FILE* f;
        fileWrapper() : f(0) {}
        ~fileWrapper() { if(f) fclose(f); }
    };
    fileWrapper fw;

    if (!m_buf)
    {
        fw.f = fopen(m_filename.c_str(), "wb");
        if (!fw.f)
        {
            fprintf(stderr, "Error! save file:%s open failed \n",m_filename.c_str());
            return false;
        }
    }

    if (!b_bmjpu_enc_load)
    {
        bm_jpu_enc_load(DEV_ID);
        b_bmjpu_enc_load = true;

        bmjpu_setup_logging();
    }

    int dump_num = bm_jpu_jpeg_get_dump();
    if( dump_num > 0)
    {
        frame_dump_enc_num = dump_num;
        frame_dump_dec_num = dump_num;
        cv::vpp::setDump(dump_num);
    }

    int bs_buffer_size;
    BmJpuFramebuffer framebuffer;
    // BmJpuWrappedDMABuffer wrapped_dma_buffer;
    bm_device_mem_t wrapped_dma_mem;
    BmJpuJPEGEncParams enc_params; /* Set up the encoding parameters */

    bool b_yuv_mat = img.avOK();
    if(!prepareEncInputParams(enc_params, bs_buffer_size, b_yuv_mat, img, params, (void*)fw.f ))
        return false;

    /* Open BM JPEG encoder */
    BmJpuEncReturnCodes enc_ret;
    enc_ret = bm_jpu_jpeg_enc_open(&(jpeg_encoder), NULL, bs_buffer_size, DEV_ID);
    if (enc_ret != BM_JPU_ENC_RETURN_CODE_OK)
    {
        fprintf(stderr, "Error! Failed to open bm_jpu_jpeg_enc_open() :  %s\n",
                bm_jpu_enc_error_string(enc_ret));
        close();
        return false;
    }

    if(!b_yuv_mat)
    {
        if(!prepareInternalDMABuffer(framebuffer, enc_params.frame_width, enc_params.frame_height, enc_params.color_format, img))
            return false;
    }
    else
    {
        if(!prepareExternalDMABuffer(framebuffer, enc_params.frame_width, enc_params.frame_height, img, wrapped_dma_mem))
            return false;
    }
    /* Do the actual encoding */
    enc_ret = bm_jpu_jpeg_enc_encode(jpeg_encoder, &framebuffer,
                                     &enc_params, NULL, NULL);

    /* The framebuffer's DMA buffer isn't needed anymore, since we just
     * did the encoding, so deallocate it */
    bm_jpu_dma_buffer_deallocate(framebuffer.dma_buffer);

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
