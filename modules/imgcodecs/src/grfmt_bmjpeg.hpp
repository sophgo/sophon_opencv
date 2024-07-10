/*M///////////////////////////////////////////////////////////////////////////////////////
//
//  IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
//
//  By downloading, copying, installing or using the software you agree to this license.
//  If you do not agree to this license, do not download, install,
//  copy or use the software.
//
//
//                           License Agreement
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

#ifndef _GRFMT_BMJPEG_H_
#define _GRFMT_BMJPEG_H_

#include "grfmt_base.hpp"
#include "bitstrm.hpp"
#include "bm_jpeg_interface.h"

#ifndef USING_SOC
#define MAX_RESOLUTION_W    (4096)
#define MAX_RESOLUTION_H    (4096)
#endif

// JPU Jpeg codec
namespace cv
{
// JPU Decoder
class BMJpegDecoder CV_FINAL : public BaseImageDecoder
{
public:

    BMJpegDecoder();
    virtual ~BMJpegDecoder();

    bool  readData( Mat& img ) CV_OVERRIDE;
    bool  readHeader() CV_OVERRIDE;
    void  close(int decode_status);

    ImageDecoder newDecoder() const CV_OVERRIDE;

protected:
    FILE* m_f;
    void* m_state;

    BmJpuJPEGDecoder *jpeg_decoder;
    uint8_t* m_src_data;
    int      m_data_len;
    int      m_src_type;
    int      m_device_id;
    int      m_vpp_fd;
private:
    BMJpegDecoder(const BMJpegDecoder &); // copy disabled
    BMJpegDecoder& operator=(const BMJpegDecoder &); // assign disabled
    int outputMat(Mat& img, BmJpuJPEGDecInfo& info);
    AVFrame *fillAVFrame(BmJpuJPEGDecInfo& info);
    int softDec(Mat& img, unsigned int iScale, unsigned int scale_w, unsigned int scale_h);
};

// JPU Encoder
class BMJpegEncoder CV_FINAL : public BaseImageEncoder
{
public:
    BMJpegEncoder();
    virtual ~BMJpegEncoder();

    bool  write( const Mat& img, const std::vector<int>& params ) CV_OVERRIDE;
    void  close();
    ImageEncoder newEncoder() const CV_OVERRIDE;
private:
    bool prepareInternalDMABuffer(BmJpuFramebuffer& framebuffer, int width, int height, BmJpuImageFormat image_format, const Mat& img);
    bool prepareDMABuffer(BmJpuFramebuffer& framebuffer, int width, int height, BmJpuImageFormat image_format, const Mat& in_img, Mat& out_img, bm_device_mem_t &wrapped_mem);
    int fillFrameBuffer(Mat& img, BmJpuFramebuffer& framebuffer);
    bool prepareEncInputParams(BmJpuJPEGEncParams& encParams, int& bs_buffer_size, bool is_yuv_mat, const Mat& img, const std::vector<int>& params, void* file);
protected:
    BmJpuJPEGEncoder *jpeg_encoder;
    int   m_device_id;
    int   m_vpp_fd;
};

}

#endif/*_GRFMT_BMJPEG_H_*/
