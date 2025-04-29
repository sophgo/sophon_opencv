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
//################################################################################
//
//                    Created by Kumataro
//
//################################################################################

#include "precomp.hpp"

#include <ft2build.h>
#include FT_FREETYPE_H
#include FT_OUTLINE_H
#include FT_IMAGE_H
#include FT_BBOX_H
#include FT_GLYPH_H
#include FT_STROKER_H


#include <hb.h>
#include <hb-ft.h>
#include <map>
#include <unistd.h>

namespace cv {
namespace freetype {

using namespace std;

#define SCALEBITS 8
#define ONE_HALF  (1 << (SCALEBITS - 1))
#define FIX(x)    ((int) ((x) * (1 << SCALEBITS) + 0.5))

#define FFMIN(a,b) ((a) > (b) ? (b) : (a))
#define FFMAX(a,b) ((a) > (b) ? (a) : (b))

#define RGB_TO_Y_JPEG(r, g, b) \
(FFMIN((FIX(0.29900) * (r) + FIX(0.58700) * (g) + \
  FIX(0.11400) * (b) + (ONE_HALF)) >> SCALEBITS, 255))

#define RGB_TO_U_JPEG(r1, g1, b1)\
(((- FIX(0.16874) * r1 - FIX(0.33126) * g1 + \
     FIX(0.50000) * b1 + (ONE_HALF) - 1) >> (SCALEBITS)) + 128)

#define RGB_TO_V_JPEG(r1, g1, b1)\
(((FIX(0.50000) * r1 - FIX(0.41869) * g1 - \
   FIX(0.08131) * b1 + (ONE_HALF) - 1) >> (SCALEBITS)) + 128)

#define GET_UTF8(val, GET_BYTE, ERROR)\
    val= (GET_BYTE);\
    {\
        uint32_t top = (val & 128) >> 1;\
        if ((val & 0xc0) == 0x80 || val >= 0xFE)\
            ERROR\
        while (val & top) {\
            int tmp= (GET_BYTE) - 128;\
            if(tmp>>6)\
                ERROR\
            val= (val<<6) + tmp;\
            top <<= 5;\
        }\
        val &= (top << 1) - 1;\
    }

#define BLEND_PIXEL_1   1
#define BLEND_PIXEL_2   2

typedef struct Glyph {
    FT_Glyph glyph;
    uint32_t code;
    unsigned int fontsize;
    FT_Bitmap bitmap; ///< array holding bitmaps of font
    FT_Bitmap border_bitmap; ///< array holding bitmaps of font border
    FT_BBox bbox;
    int advance;
    int bitmap_left;
    int bitmap_top;
} Glyph;

class CV_EXPORTS_W FreeType2Impl CV_FINAL : public FreeType2
{
public:
    FreeType2Impl();
    ~FreeType2Impl();
    void loadFontData(String fontFileName, int id) CV_OVERRIDE;
    void setSplitNumber( int num ) CV_OVERRIDE;
    void putText(
        InputOutputArray img, const String& text, Point org,
        int fontHeight, Scalar color,
        int thickness, int line_type, bool bottomLeftOrigin
    ) CV_OVERRIDE;
    Size getTextSize(
        const String& text, int fontHeight, int thickness,
        CV_OUT int* baseLine
    ) CV_OVERRIDE;

private:
    FT_Library       mLibrary;
    FT_Face          mFace;
    FT_Outline_Funcs mFn;

    bool             mIsFaceAvailable;
    int              mCtoL;
    hb_font_t        *mHb_font;
    std::map<uint64_t,Glyph*> mGlyph;

    void putTextBitmapMono(
        InputOutputArray img, const String& text, Point org,
        int fontHeight, Scalar color,
        int thickness, int line_type, bool bottomLeftOrigin
    );
    void putTextBitmapBlendYUV(
        InputOutputArray img, const String& text, Point org,
        int fontHeight, Scalar color
    );
    void putTextBitmapBlend(
        InputOutputArray img, const String& text, Point org,
        int fontHeight, Scalar color,
        int thickness, int line_type, bool bottomLeftOrigin
    );
    void putTextOutline(
        InputOutputArray img, const String& text, Point org,
        int fontHeight, Scalar color,
        int thickness, int line_type, bool bottomLeftOrigin
    );

    void blend_pixel(uint8_t *dst, unsigned src, unsigned alpha,
                        const uint8_t *mask, int mask_linesize, int l2depth,
                        unsigned w, unsigned h, unsigned shift, unsigned xm0, int islastrow);

    static int mvFn( const FT_Vector *to, void * user);
    static int lnFn( const FT_Vector *to, void * user);
    static int coFn( const FT_Vector *cnt,
                     const FT_Vector *to,
                     void * user);
    static int cuFn( const FT_Vector *cnt1,
                     const FT_Vector *cnt2,
                     const FT_Vector *to,
                     void * user);

    // Offset value to handle the position less than 0.
    static const unsigned int cOutlineOffset = 0x80000000;

    /**
     * Convert from 26.6 real to signed integer
     */
    static int ftd(unsigned int fixedInt){
        unsigned int ret = ( ( fixedInt + (1 << 5)  ) >> 6 );
        return (int)ret - ( cOutlineOffset >> 6 );
    }

    class PathUserData{
    private:
    public:
        PathUserData( InputOutputArray _img) : mImg(_img) {};

        InputOutputArray mImg;
        Scalar mColor;
        int    mThickness;
        int    mLine_type;
        FT_Vector        mOldP;
        int              mCtoL;
        std::vector < Point > mPts;
    };
};

static const int ITUR_BT_601_SHIFT = 20;
static const int ITUR_BT_601_CRY =  269484;
static const int ITUR_BT_601_CGY =  528482;
static const int ITUR_BT_601_CBY =  102760;
static const int ITUR_BT_601_CRU = -155188;
static const int ITUR_BT_601_CGU = -305135;
static const int ITUR_BT_601_CBU =  460324;
static const int ITUR_BT_601_CGV = -385875;
static const int ITUR_BT_601_CBV = -74448;

static void color_convert(Scalar _color, uchar *color_yuv)
{
    uchar bc = saturate_cast<uchar>(_color.val[0]);
    uchar gc = saturate_cast<uchar>(_color.val[1]);
    uchar rc = saturate_cast<uchar>(_color.val[2]);

    int yy = ITUR_BT_601_CRY * rc + ITUR_BT_601_CGY * gc + ITUR_BT_601_CBY * bc + (16 << ITUR_BT_601_SHIFT) + (1 << (ITUR_BT_601_SHIFT - 1));
    int uu = ITUR_BT_601_CRU * rc + ITUR_BT_601_CGU * gc + ITUR_BT_601_CBU * bc + (1 << (ITUR_BT_601_SHIFT - 1)) + (128 << ITUR_BT_601_SHIFT);
    int vv = ITUR_BT_601_CBU * rc + ITUR_BT_601_CGV * gc + ITUR_BT_601_CBV * bc + (1 << (ITUR_BT_601_SHIFT - 1)) + (128 << ITUR_BT_601_SHIFT);

    color_yuv[0] = saturate_cast<uchar>(yy >> ITUR_BT_601_SHIFT);
    color_yuv[1] = saturate_cast<uchar>(uu >> ITUR_BT_601_SHIFT);
    color_yuv[2] = saturate_cast<uchar>(vv >> ITUR_BT_601_SHIFT);

    return;
}

FreeType2Impl::FreeType2Impl()
{
    FT_Init_FreeType(&(this->mLibrary) );

    mCtoL        = 16;
    mFn.shift    = 0;
    mFn.delta    = 0;
    mFn.move_to  = FreeType2Impl::mvFn;
    mFn.line_to  = FreeType2Impl::lnFn;
    mFn.cubic_to = FreeType2Impl::cuFn;
    mFn.conic_to = FreeType2Impl::coFn;

    mIsFaceAvailable = false;
}

FreeType2Impl::~FreeType2Impl()
{
    std::map<uint64_t,Glyph*>::iterator  iteGlyph = mGlyph.begin();
    while(iteGlyph != mGlyph.end()){
        Glyph * temp = iteGlyph->second;
        iteGlyph = mGlyph.erase(iteGlyph);
        if(temp != NULL){
            FT_Done_Glyph(temp->glyph);
            free(temp);
        }
    }

    if( mIsFaceAvailable  == true ){
        hb_font_destroy (mHb_font);
        CV_Assert(!FT_Done_Face(mFace));
        mIsFaceAvailable = false;
    }
    CV_Assert(!FT_Done_FreeType(mLibrary));
}

void FreeType2Impl::loadFontData(String fontFileName, int idx)
{
    if( mIsFaceAvailable  == true ){
        hb_font_destroy (mHb_font);
        CV_Assert(!FT_Done_Face(mFace));
    }
    CV_Assert(!FT_New_Face( mLibrary, fontFileName.c_str(), idx, &(mFace) ) );
    mHb_font = hb_ft_font_create (mFace, NULL);
    CV_Assert( mHb_font != NULL );
    mIsFaceAvailable = true;
}

void FreeType2Impl::setSplitNumber(int num ){
    CV_Assert( num > 0 );
    mCtoL        = num;
}

void FreeType2Impl::putText(
    InputOutputArray _img, const String& _text, Point _org,
    int _fontHeight, Scalar _color,
    int _thickness, int _line_type, bool _bottomLeftOrigin
)
{
    CV_Assert( mIsFaceAvailable == true );
    CV_Assert( ( _img.empty()    == false ) &&
               ( _img.isMat()    == true  ) &&
               ( _img.depth()    == CV_8U ) &&
               ( _img.dims()     == 2     ) );
    Mat img = _img.getMat();
    if(!img.avOK()){
        CV_Assert( ( _img.channels() == 3 ) );
    }
    CV_Assert( ( _line_type == CV_AA) ||
               ( _line_type == 4 ) ||
               ( _line_type == 8 ) );
    CV_Assert( _fontHeight >= 0 );

    if ( _text.empty() )
    {
         return;
    }
    if ( _fontHeight == 0 )
    {
         return;
    }

    if( _line_type == CV_AA && _img.depth() != CV_8U ){
        _line_type = 8;
    }

    CV_Assert(!FT_Set_Pixel_Sizes( mFace, _fontHeight, _fontHeight ));

    if( _thickness < 0 ) // CV_FILLED
    {
        if ( _line_type == CV_AA ) {
            putTextBitmapBlend( _img, _text, _org, _fontHeight, _color,
                _thickness, _line_type, _bottomLeftOrigin );
        }else{
            putTextBitmapMono( _img, _text, _org, _fontHeight, _color,
                _thickness, _line_type, _bottomLeftOrigin );
        }
    }else{
            putTextOutline( _img, _text, _org, _fontHeight, _color,
                _thickness, _line_type, _bottomLeftOrigin );
    }
}

void FreeType2Impl::putTextOutline(
   InputOutputArray _img, const String& _text, Point _org,
   int _fontHeight, Scalar _color,
   int _thickness, int _line_type, bool _bottomLeftOrigin )
{
    hb_buffer_t *hb_buffer = hb_buffer_create ();
    CV_Assert( hb_buffer != NULL );

    unsigned int textLen;
    hb_buffer_guess_segment_properties (hb_buffer);
    hb_buffer_add_utf8 (hb_buffer, _text.c_str(), -1, 0, -1);

    hb_glyph_info_t *info =
        hb_buffer_get_glyph_infos(hb_buffer,&textLen );
    CV_Assert( info != NULL );

    hb_shape (mHb_font, hb_buffer, NULL, 0);

    if( _bottomLeftOrigin == true ){
        _org.y -= _fontHeight;
    }

    PathUserData *userData = new PathUserData( _img );
    userData->mColor     = _color;
    userData->mCtoL      = mCtoL;
    userData->mThickness = _thickness;
    userData->mLine_type = _line_type;

    for( unsigned int i = 0 ; i < textLen ; i ++ ){
        CV_Assert(!FT_Load_Glyph(mFace, info[i].codepoint, 0 ));

        FT_GlyphSlot slot  = mFace->glyph;
        FT_Outline outline = slot->outline;

        // Flip
        FT_Matrix mtx = { 1 << 16 , 0 , 0 , -(1 << 16) };
        FT_Outline_Transform(&outline, &mtx);

        // Move
        FT_Outline_Translate(&outline,
                             cOutlineOffset,
                             cOutlineOffset );
        // Move
        FT_Outline_Translate(&outline,
                             (FT_Pos)(_org.x << 6),
                             (FT_Pos)( (_org.y + _fontHeight) << 6) );

        // Draw
        CV_Assert( !FT_Outline_Decompose(&outline, &mFn, (void*)userData) );

        // Draw (Last Path)
        mvFn( NULL, (void*)userData );

        _org.x += ( mFace->glyph->advance.x ) >> 6;
        _org.y += ( mFace->glyph->advance.y ) >> 6;
   }
   delete userData;
   hb_buffer_destroy (hb_buffer);
}

void FreeType2Impl::putTextBitmapBlendYUV(
   InputOutputArray _img, const String& _text, Point _org,
   int _fontHeight, Scalar _color )
{
    Mat dst = _img.getMat();
    if(dst.avOK()){
#ifdef HAVE_BMCV
        bmcv::downloadMat(dst);
#endif
    }
    int i = 0;
    uint8_t *p1;
    uint32_t code = 0;
    unsigned char *in_text = (unsigned char*)malloc(_text.size()+1);
    strcpy((char*)in_text, _text.c_str());
    uchar bc = saturate_cast<uchar>(_color.val[0]);
    uchar gc = saturate_cast<uchar>(_color.val[1]);
    uchar rc = saturate_cast<uchar>(_color.val[2]);

    unsigned int color_y = RGB_TO_Y_JPEG(rc, gc, bc);
    unsigned int color_u = RGB_TO_U_JPEG(rc, gc, bc);
    unsigned int color_v = RGB_TO_V_JPEG(rc, gc, bc);
    unsigned int alpha = (0x10307 * 255 + 0x3) >> 8;

    for (i = 0, p1 = in_text; *p1; i++) {
        Glyph *glyph = NULL;

        GET_UTF8(code, *p1++, continue;);
        std::map<uint64_t,Glyph*>::iterator  iteGlyph;
        uint64_t code_key = _fontHeight;
        code_key = (code_key << 32) | code;
        iteGlyph = mGlyph.find(code_key);
        if(iteGlyph == mGlyph.end()){
            FT_BitmapGlyph bitmapglyph;
            /* load glyph into s->face->glyph */
            if (FT_Load_Char(mFace, code, FT_LOAD_DEFAULT))
                return;

            glyph = (Glyph*)malloc(sizeof(*glyph));
            if (!glyph) {
                return;
            }
            glyph->code  = code;
            glyph->fontsize = _fontHeight;
            if (FT_Get_Glyph(mFace->glyph, &glyph->glyph)) {
                return;
            }
            if (FT_Glyph_To_Bitmap(&glyph->glyph, FT_RENDER_MODE_NORMAL, 0, 1)) {
                return;
            }
            bitmapglyph = (FT_BitmapGlyph) glyph->glyph;
            glyph->bitmap      = bitmapglyph->bitmap;
            glyph->bitmap_left = bitmapglyph->left;
            glyph->bitmap_top  = bitmapglyph->top;
            glyph->advance     = mFace->glyph->advance.x >> 6;

            /* measure text height to calculate text_height (or the maximum text height) */
            FT_Glyph_Get_CBox(glyph->glyph, ft_glyph_bbox_pixels, &glyph->bbox);
            mGlyph.insert(pair<uint64_t,Glyph*>(code_key, glyph));
        }else{
            glyph = iteGlyph->second;
        }

        Point org;
        org.x = _org.x + (glyph->advance - glyph->bitmap.pitch)/2;
        org.y = _org.y;
        uchar *ptry = NULL;
        uchar *ptru = NULL;
        uchar *ptrv = NULL;
        const uint8_t *mask = NULL;
        mask = glyph->bitmap.buffer;
        int islastrow = 0;
        for(unsigned int m=0; m<glyph->bitmap.rows; m++){
            if(m == (glyph->bitmap.rows-1)){
                islastrow = 1;
            }
            for(int n=0; n<glyph->bitmap.pitch; n++){

                int offsetY = ((org.y-glyph->bitmap_top)+m);
                int offsetX = org.x + n;
                if((offsetY <= 0) || (offsetY >= dst.rows)) continue;
                if((offsetX <= 0) || (offsetX >= dst.cols)) continue;
                ptry = (uchar*)dst.u->frame->data[0] + ((org.y-glyph->bitmap_top)+m)*dst.u->frame->linesize[0] + org.x + n;
                ptru = (uchar*)dst.u->frame->data[1] + ((org.y-glyph->bitmap_top)+m)/2*dst.u->frame->linesize[1] + (org.x + n)/2;
                ptrv = (uchar*)dst.u->frame->data[2] + ((org.y-glyph->bitmap_top)+m)/2*dst.u->frame->linesize[2] + (org.x + n)/2;
                //Y ok
                blend_pixel(ptry, color_y, alpha,
                        mask, glyph->bitmap.pitch, 3,   //yuv420 is 3;  nv12 is 2;
                        BLEND_PIXEL_1, BLEND_PIXEL_1, 0, n, islastrow);
                if(m == 0){
                    if((org.y%2) == 0){
                        if(n == 0){
                            if((org.x %2) == 1){
                                //U
                                blend_pixel(ptru, color_u, alpha,
                                        mask, glyph->bitmap.pitch, 3,   //yuv420 is 3;  nv12 is 2;
                                        BLEND_PIXEL_1, BLEND_PIXEL_2, 2, n, islastrow);
                                //V
                                blend_pixel(ptrv, color_v, alpha,
                                        mask, glyph->bitmap.pitch, 3,   //yuv420 is 3;  nv12 is 2;
                                        BLEND_PIXEL_1, BLEND_PIXEL_2, 2, n, islastrow);
                            }else{
                                //U
                                blend_pixel(ptru, color_u, alpha,
                                        mask, glyph->bitmap.pitch, 3,   //yuv420 is 3;  nv12 is 2;
                                        BLEND_PIXEL_2, BLEND_PIXEL_2, 2, n, islastrow);
                                //V
                                blend_pixel(ptrv, color_v, alpha,
                                        mask, glyph->bitmap.pitch, 3,   //yuv420 is 3;  nv12 is 2;
                                        BLEND_PIXEL_2, BLEND_PIXEL_2, 2, n, islastrow);
                            }
                        }
                        else if(n == glyph->bitmap.pitch -1){
                            if(((org.x + glyph->bitmap.pitch)%2) == 1){
                                //U
                                blend_pixel(ptru, color_u, alpha,
                                        mask, glyph->bitmap.pitch, 3,   //yuv420 is 3;  nv12 is 2;
                                        BLEND_PIXEL_1, BLEND_PIXEL_2, 2, n, islastrow);
                                //V
                                blend_pixel(ptrv, color_v, alpha,
                                        mask, glyph->bitmap.pitch, 3,   //yuv420 is 3;  nv12 is 2;
                                        BLEND_PIXEL_1, BLEND_PIXEL_2, 2, n, islastrow);
                            }
                        }
                        else if((org.x + n)%2 == 0){
                            //U
                            blend_pixel(ptru, color_u, alpha,
                                    mask, glyph->bitmap.pitch, 3,   //yuv420 is 3;  nv12 is 2;
                                    BLEND_PIXEL_2, BLEND_PIXEL_2, 2, n, islastrow);
                            //V
                            blend_pixel(ptrv, color_v, alpha,
                                    mask, glyph->bitmap.pitch, 3,   //yuv420 is 3;  nv12 is 2;
                                    BLEND_PIXEL_2, BLEND_PIXEL_2, 2, n, islastrow);
                        }
                    }else{
                        if(n == 0){
                            if((org.x %2) == 1){
                                //U
                                blend_pixel(ptru, color_u, alpha,
                                        mask, glyph->bitmap.pitch, 3,   //yuv420 is 3;  nv12 is 2;
                                        BLEND_PIXEL_1, BLEND_PIXEL_1, 2, n, islastrow);
                                //V
                                blend_pixel(ptrv, color_v, alpha,
                                        mask, glyph->bitmap.pitch, 3,   //yuv420 is 3;  nv12 is 2;
                                        BLEND_PIXEL_1, BLEND_PIXEL_1, 2, n, islastrow);
                            }else{
                                //U
                                blend_pixel(ptru, color_u, alpha,
                                        mask, glyph->bitmap.pitch, 3,   //yuv420 is 3;  nv12 is 2;
                                        BLEND_PIXEL_2, BLEND_PIXEL_1, 2, n, islastrow);
                                //V
                                blend_pixel(ptrv, color_v, alpha,
                                        mask, glyph->bitmap.pitch, 3,   //yuv420 is 3;  nv12 is 2;
                                        BLEND_PIXEL_2, BLEND_PIXEL_1, 2, n, islastrow);
                            }
                        }
                        else if(n == glyph->bitmap.pitch -1){
                            if(((org.x + glyph->bitmap.pitch)%2) == 1){
                                //U
                                blend_pixel(ptru, color_u, alpha,
                                        mask, glyph->bitmap.pitch, 3,   //yuv420 is 3;  nv12 is 2;
                                        BLEND_PIXEL_1, BLEND_PIXEL_1, 2, n, islastrow);
                                //V
                                blend_pixel(ptrv, color_v, alpha,
                                        mask, glyph->bitmap.pitch, 3,   //yuv420 is 3;  nv12 is 2;
                                        BLEND_PIXEL_1, BLEND_PIXEL_1, 2, n, islastrow);
                            }
                        }
                        else if((org.x + n)%2 == 0){
                            //U
                            blend_pixel(ptru, color_u, alpha,
                                    mask, glyph->bitmap.pitch, 3,   //yuv420 is 3;  nv12 is 2;
                                    BLEND_PIXEL_2, BLEND_PIXEL_1, 2, n, islastrow);
                            //V
                            blend_pixel(ptrv, color_v, alpha,
                                    mask, glyph->bitmap.pitch, 3,   //yuv420 is 3;  nv12 is 2;
                                    BLEND_PIXEL_2, BLEND_PIXEL_1, 2, n, islastrow);
                        }
                    }

                }
                else if(m == (glyph->bitmap.rows-1)){
                    if((org.y + glyph->bitmap.rows)%2 == 1){
                        if(n == 0){
                            if((org.x %2) == 1){
                                //U
                                blend_pixel(ptru, color_u, alpha,
                                        mask, glyph->bitmap.pitch, 3,   //yuv420 is 3;  nv12 is 2;
                                        BLEND_PIXEL_1, BLEND_PIXEL_1, 2, n, islastrow);
                                //V
                                blend_pixel(ptrv, color_v, alpha,
                                        mask, glyph->bitmap.pitch, 3,   //yuv420 is 3;  nv12 is 2;
                                        BLEND_PIXEL_1, BLEND_PIXEL_1, 2, n, islastrow);
                            }else{
                                //U
                                blend_pixel(ptru, color_u, alpha,
                                        mask, glyph->bitmap.pitch, 3,   //yuv420 is 3;  nv12 is 2;
                                        BLEND_PIXEL_2, BLEND_PIXEL_1, 2, n, islastrow);
                                //V
                                blend_pixel(ptrv, color_v, alpha,
                                        mask, glyph->bitmap.pitch, 3,   //yuv420 is 3;  nv12 is 2;
                                        BLEND_PIXEL_2, BLEND_PIXEL_1, 2, n, islastrow);
                            }
                        }
                        else if(n == glyph->bitmap.pitch -1){
                            if(((org.x + glyph->bitmap.pitch)%2) == 1){
                                //U
                                blend_pixel(ptru, color_u, alpha,
                                        mask, glyph->bitmap.pitch, 3,   //yuv420 is 3;  nv12 is 2;
                                        BLEND_PIXEL_1, BLEND_PIXEL_1, 2, n, islastrow);
                                //V
                                blend_pixel(ptrv, color_v, alpha,
                                        mask, glyph->bitmap.pitch, 3,   //yuv420 is 3;  nv12 is 2;
                                        BLEND_PIXEL_1, BLEND_PIXEL_1, 2, n, islastrow);
                            }
                        }
                        else if((org.x + n)%2 == 0){
                            //U
                            blend_pixel(ptru, color_u, alpha,
                                    mask, glyph->bitmap.pitch, 3,   //yuv420 is 3;  nv12 is 2;
                                    BLEND_PIXEL_2, BLEND_PIXEL_1, 2, n, islastrow);
                            //V
                            blend_pixel(ptrv, color_v, alpha,
                                    mask, glyph->bitmap.pitch, 3,   //yuv420 is 3;  nv12 is 2;
                                    BLEND_PIXEL_2, BLEND_PIXEL_1, 2, n, islastrow);
                        }
                    }
                }
                else if((org.y + m)%2 == 0){
                    if(n == 0){
                        if((org.x %2) == 1){
                            //U
                            blend_pixel(ptru, color_u, alpha,
                                    mask, glyph->bitmap.pitch, 3,   //yuv420 is 3;  nv12 is 2;
                                    BLEND_PIXEL_1, BLEND_PIXEL_2, 2, n, islastrow);
                            //V
                            blend_pixel(ptrv, color_v, alpha,
                                    mask, glyph->bitmap.pitch, 3,   //yuv420 is 3;  nv12 is 2;
                                    BLEND_PIXEL_1, BLEND_PIXEL_2, 2, n, islastrow);
                        }else{
                            //U
                            blend_pixel(ptru, color_u, alpha,
                                    mask, glyph->bitmap.pitch, 3,   //yuv420 is 3;  nv12 is 2;
                                    BLEND_PIXEL_2, BLEND_PIXEL_2, 2, n, islastrow);
                            //V
                            blend_pixel(ptrv, color_v, alpha,
                                    mask, glyph->bitmap.pitch, 3,   //yuv420 is 3;  nv12 is 2;
                                    BLEND_PIXEL_2, BLEND_PIXEL_2, 2, n, islastrow);
                        }
                    }
                    else if(n == glyph->bitmap.pitch -1){
                        if(((org.x + glyph->bitmap.pitch)%2) == 1){
                            //U
                            blend_pixel(ptru, color_u, alpha,
                                    mask, glyph->bitmap.pitch, 3,   //yuv420 is 3;  nv12 is 2;
                                    BLEND_PIXEL_1, BLEND_PIXEL_2, 2, n, islastrow);
                            //V
                            blend_pixel(ptrv, color_v, alpha,
                                    mask, glyph->bitmap.pitch, 3,   //yuv420 is 3;  nv12 is 2;
                                    BLEND_PIXEL_1, BLEND_PIXEL_2, 2, n, islastrow);
                        }
                    }
                    else if((org.x + n)%2 == 0){
                        //U
                        blend_pixel(ptru, color_u, alpha,
                                mask, glyph->bitmap.pitch, 3,   //yuv420 is 3;  nv12 is 2;
                                BLEND_PIXEL_2, BLEND_PIXEL_2, 2, n, islastrow);
                        //V
                        blend_pixel(ptrv, color_v, alpha,
                                mask, glyph->bitmap.pitch, 3,   //yuv420 is 3;  nv12 is 2;
                                BLEND_PIXEL_2, BLEND_PIXEL_2, 2, n, islastrow);
                    }
                }
            }
            mask += glyph->bitmap.pitch;
        }
        _org.x += glyph->advance;
    }
    if(in_text != NULL){
        free(in_text);
        in_text = NULL;
    }
    if(dst.avOK()){
#ifdef HAVE_BMCV
        bmcv::uploadMat(dst);
#endif
    }
    return;
}

void FreeType2Impl::putTextBitmapMono(
   InputOutputArray _img, const String& _text, Point _org,
   int _fontHeight, Scalar _color,
   int _thickness, int _line_type, bool _bottomLeftOrigin )
{
    CV_Assert( _thickness < 0 );
    CV_Assert( _line_type == 4 || _line_type == 8);

    Mat dst = _img.getMat();
    hb_buffer_t *hb_buffer = hb_buffer_create ();
    CV_Assert( hb_buffer != NULL );
    uchar color_yuv[4] = {0};
    if(dst.avOK()){
#ifdef HAVE_BMCV
        bmcv::downloadMat(dst);
#endif
        color_convert(_color,color_yuv);
    }

    unsigned int textLen;
    hb_buffer_guess_segment_properties (hb_buffer);
    hb_buffer_add_utf8 (hb_buffer, _text.c_str(), -1, 0, -1);
    hb_glyph_info_t *info =
        hb_buffer_get_glyph_infos(hb_buffer,&textLen );
    CV_Assert( info != NULL );

    hb_shape (mHb_font, hb_buffer, NULL, 0);

    _org.y += _fontHeight;
    if( _bottomLeftOrigin == true ){
        _org.y -= _fontHeight;
    }
    for( unsigned int i = 0 ; i < textLen ; i ++ ){
        CV_Assert( !FT_Load_Glyph(mFace, info[i].codepoint, 0 ) );
        CV_Assert( !FT_Render_Glyph( mFace->glyph, FT_RENDER_MODE_MONO ) );
        FT_Bitmap    *bmp = &(mFace->glyph->bitmap);

        Point gPos = _org;
        gPos.y -= ( mFace->glyph->metrics.horiBearingY >> 6) ;
        gPos.x += ( mFace->glyph->metrics.horiBearingX >> 6) ;

        uchar *ptry = NULL;
        uchar *ptru = NULL;
        uchar *ptrv = NULL;

        for (int row = 0; row < (int)bmp->rows; row ++) {
            if( gPos.y + row < 0 ) {
                continue;
            }
            if( gPos.y + row >= dst.rows ) {
                break;
            }

            for (int col = 0; col < bmp->pitch; col ++) {
                int cl = bmp->buffer[ row * bmp->pitch + col ];
                if ( cl == 0 ) {
                    continue;
                }
                for(int bit = 7; bit >= 0; bit -- ){
                    if( gPos.x + col * 8 + (7 - bit) < 0 )
                    {
                        continue;
                    }
                    if( gPos.x + col * 8 + (7 - bit) >= dst.cols )
                    {
                        break;
                    }

                    if ( ( (cl >> bit) & 0x01 ) == 1 ) {
                        if(dst.avOK())
                        {
                            ptry = (uchar*)dst.u->frame->data[0] + (gPos.y+row)*dst.u->frame->linesize[0] + (gPos.x + col * 8 + (7 - bit));
                            ptru = (uchar*)dst.u->frame->data[1] + (gPos.y/2+row/2)*dst.u->frame->linesize[1] + (gPos.x + col * 8 + (7 - bit))/2;
                            ptrv = (uchar*)dst.u->frame->data[2] + (gPos.y/2+row/2)*dst.u->frame->linesize[2] + (gPos.x + col * 8 + (7 - bit))/2;
                            *ptry = (uchar)color_yuv[0];
                            *ptru = (uchar)color_yuv[1];
                            *ptrv = (uchar)color_yuv[2];
                        }
                        else
                        {
                            cv::Vec3b* ptr = dst.ptr<cv::Vec3b>( gPos.y + row,  gPos.x + col * 8 + (7 - bit) );
                            (*ptr)[0] = (double)_color[0];
                            (*ptr)[1] = (double)_color[1];
                            (*ptr)[2] = (double)_color[2];
                        }
                    }
                }
            }
        }

        _org.x += ( mFace->glyph->advance.x ) >> 6;
        _org.y += ( mFace->glyph->advance.y ) >> 6;
    }
    hb_buffer_destroy (hb_buffer);
    if(dst.avOK()){
#ifdef HAVE_BMCV
        bmcv::uploadMat(dst);
#endif
    }
}

void FreeType2Impl::putTextBitmapBlend(
   InputOutputArray _img, const String& _text, Point _org,
   int _fontHeight, Scalar _color,
   int _thickness, int _line_type, bool _bottomLeftOrigin )
{
    CV_Assert( _thickness < 0 );
    CV_Assert( _line_type == 16 );

    Mat dst = _img.getMat();
    if(dst.avOK()){
        putTextBitmapBlendYUV(_img, _text, _org, _fontHeight, _color);
        return;
    }
    hb_buffer_t *hb_buffer = hb_buffer_create ();
    CV_Assert( hb_buffer != NULL );

    unsigned int textLen;
    hb_buffer_guess_segment_properties (hb_buffer);
    hb_buffer_add_utf8 (hb_buffer, _text.c_str(), -1, 0, -1);
    hb_glyph_info_t *info =
        hb_buffer_get_glyph_infos(hb_buffer,&textLen );
    CV_Assert( info != NULL );

    hb_shape (mHb_font, hb_buffer, NULL, 0);

    _org.y += _fontHeight;
    if( _bottomLeftOrigin == true ){
        _org.y -= _fontHeight;
    }

    for( unsigned int i = 0 ; i < textLen ; i ++ ){
        CV_Assert( !FT_Load_Glyph(mFace, info[i].codepoint, 0 ) );
        CV_Assert( !FT_Render_Glyph( mFace->glyph, FT_RENDER_MODE_NORMAL ) );
        FT_Bitmap    *bmp = &(mFace->glyph->bitmap);

        Point gPos = _org;
        gPos.y -= ( mFace->glyph->metrics.horiBearingY >> 6) ;
        gPos.x += ( mFace->glyph->metrics.horiBearingX >> 6) ;

        for (int row = 0; row < (int)bmp->rows; row ++) {
            if( gPos.y + row < 0 ) {
                continue;
            }
            if( gPos.y + row >= dst.rows ) {
                break;
            }

            for (int col = 0; col < bmp->pitch; col ++) {
                int cl = bmp->buffer[ row * bmp->pitch + col ];
                if ( cl == 0 ) {
                    continue;
                }
                if( gPos.x + col < 0 )
                {
                    continue;
                }
                if( gPos.x + col >= dst.cols )
                {
                    break;
                }
                cv::Vec3b* ptr = dst.ptr<cv::Vec3b>( gPos.y + row , gPos.x + col);
                double blendAlpha = (double ) cl / 255.0;

                (*ptr)[0] = (double) _color[0] * blendAlpha + (*ptr)[0] * (1.0 - blendAlpha );
                (*ptr)[1] = (double) _color[1] * blendAlpha + (*ptr)[1] * (1.0 - blendAlpha );
                (*ptr)[2] = (double) _color[2] * blendAlpha + (*ptr)[2] * (1.0 - blendAlpha );
            }
        }
        _org.x += ( mFace->glyph->advance.x ) >> 6;
        _org.y += ( mFace->glyph->advance.y ) >> 6;
    }
    hb_buffer_destroy (hb_buffer);
}

Size FreeType2Impl::getTextSize(
    const String& _text,
    int _fontHeight,
    int _thickness,
    CV_OUT int* _baseLine)
{
    if ( _text.empty() )
    {
         return Size(0,0);
    }

    CV_Assert( _fontHeight >= 0 ) ;
    if ( _fontHeight == 0 )
    {
         return Size(0,0);
    }

    CV_Assert(!FT_Set_Pixel_Sizes( mFace, _fontHeight, _fontHeight ));

    hb_buffer_t *hb_buffer = hb_buffer_create ();
    CV_Assert( hb_buffer != NULL );
    Point _org(0,0);

    unsigned int textLen;
    hb_buffer_guess_segment_properties (hb_buffer);
    hb_buffer_add_utf8 (hb_buffer, _text.c_str(), -1, 0, -1);
    hb_glyph_info_t *info =
        hb_buffer_get_glyph_infos(hb_buffer,&textLen );
    CV_Assert( info != NULL );
    hb_shape (mHb_font, hb_buffer, NULL, 0);

    _org.y -= _fontHeight;
    int xMin = INT_MAX, xMax = INT_MIN;
    int yMin = INT_MAX, yMax = INT_MIN;

    for( unsigned int i = 0 ; i < textLen ; i ++ ){
        CV_Assert(!FT_Load_Glyph(mFace, info[i].codepoint, 0 ));

        FT_GlyphSlot slot  = mFace->glyph;
        FT_Outline outline = slot->outline;
        FT_BBox bbox ;

        // Flip
        FT_Matrix mtx = { 1 << 16 , 0 , 0 , -(1 << 16) };
        FT_Outline_Transform(&outline, &mtx);

        // Move
        FT_Outline_Translate(&outline,
                             cOutlineOffset,
                             cOutlineOffset );

        // Move
        FT_Outline_Translate(&outline,
                             (FT_Pos)(_org.x << 6 ),
                             (FT_Pos)((_org.y + _fontHeight) << 6 ) );

        CV_Assert( !FT_Outline_Get_BBox( &outline, &bbox ) );

        // If codepoint is space(0x20), it has no glyph.
        // A dummy boundary box is needed when last code is space.
        if(
            (bbox.xMin == 0 ) && (bbox.xMax == 0 ) &&
            (bbox.yMin == 0 ) && (bbox.yMax == 0 )
        ){
            bbox.xMin = (_org.x << 6);
            bbox.xMax = (_org.x << 6 ) + ( mFace->glyph->advance.x );
            bbox.yMin = yMin;
            bbox.yMax = yMax;

            bbox.xMin += cOutlineOffset;
            bbox.xMax += cOutlineOffset;
            bbox.yMin += cOutlineOffset;
            bbox.yMax += cOutlineOffset;
        }

        xMin = cv::min ( xMin, ftd(bbox.xMin) );
        xMax = cv::max ( xMax, ftd(bbox.xMax) );
        yMin = cv::min ( yMin, ftd(bbox.yMin) );
        yMax = cv::max ( yMax, ftd(bbox.yMax) );

        _org.x += ( mFace->glyph->advance.x ) >> 6;
        _org.y += ( mFace->glyph->advance.y ) >> 6;
    }

    hb_buffer_destroy (hb_buffer);

    int width  = xMax - xMin ;
    int height = -yMin ;

    if ( _thickness > 0 ) {
        width  = cvRound(width  + _thickness * 2);
        height = cvRound(height + _thickness * 1);
    }else{
        width  = cvRound(width  + 1);
        height = cvRound(height + 1);
    }

    if ( _baseLine ) {
        *_baseLine = yMax;
    }

    return Size( width, height );
}


void FreeType2Impl::blend_pixel(uint8_t *dst, unsigned src, unsigned alpha,
                        const uint8_t *mask, int mask_linesize, int l2depth,
                        unsigned w, unsigned h, unsigned shift, unsigned xm0, int islastrow)
{
    unsigned xm, x, y, t = 0;
    unsigned xmshf = 3 - l2depth;
    unsigned xmmod = 7 >> l2depth;
    unsigned mbits = (1 << (1 << l2depth)) - 1;
    unsigned mmult = 255 / mbits;
    for (y = 0; y < h; y++) {
        if(islastrow == 1) break;
        xm = xm0;
        for (x = 0; x < w; x++) {
            if((int)(xm>>xmshf) >= mask_linesize) {
                break;
            }
            t += ((mask[xm >> xmshf] >> ((~xm & xmmod) << l2depth)) & mbits)
                 * mmult;
            xm++;
        }
        mask += mask_linesize;
    }
    alpha = (t >> shift) * alpha;
    *dst = ((0x1010101 - alpha) * *dst + alpha * src) >> 24;
}

int FreeType2Impl::mvFn( const FT_Vector *to, void * user)
{
    if(user == NULL ) { return 1; }
    PathUserData *p = (PathUserData*)user;

    if( p->mPts.size() > 0 ){
        Mat dst = p->mImg.getMat();
        const Point *ptsList[] = { &(p->mPts[0]) };
        int npt[1]; npt[0] = p->mPts.size();
        polylines(
            dst,
            ptsList,
            npt,
            1,
            false,
            p->mColor,
            p->mThickness,
            p->mLine_type,
            0
        );
    }

    p->mPts.clear();

    if( to == NULL ) { return 1; }

    p->mPts.push_back( Point ( ftd(to->x), ftd(to->y) ) );
    p->mOldP = *to;
    return 0;
}

int FreeType2Impl::lnFn( const FT_Vector *to, void * user)
{
    if(to   == NULL ) { return 1; }
    if(user == NULL ) { return 1; }

    PathUserData *p = (PathUserData *)user;
    p->mPts.push_back( Point ( ftd(to->x), ftd(to->y) ) );
    p->mOldP = *to;
    return 0;
}

int FreeType2Impl::coFn( const FT_Vector *cnt,
                     const FT_Vector *to,
                     void * user)
{
    if(cnt  == NULL ) { return 1; }
    if(to   == NULL ) { return 1; }
    if(user == NULL ) { return 1; }

    PathUserData *p = (PathUserData *)user;

    // Bezier to Line
    for(int i = 0;i <= p->mCtoL; i++){
        double u = (double)i * 1.0 / (p->mCtoL) ;
        double nu = 1.0 - u;
        double p0 =                  nu * nu;
        double p1 = 2.0 * u *        nu;
        double p2 =       u * u;

        double X = (p->mOldP.x) * p0 + cnt->x * p1 + to->x * p2;
        double Y = (p->mOldP.y) * p0 + cnt->y * p1 + to->y * p2;
        p->mPts.push_back( Point ( ftd(X), ftd(Y) ) );
    }
    p->mOldP = *to;
    return 0;
}

int FreeType2Impl::cuFn( const FT_Vector *cnt1,
                     const FT_Vector *cnt2,
                     const FT_Vector *to,
                     void * user)
{
    if(cnt1 == NULL ) { return 1; }
    if(cnt2 == NULL ) { return 1; }
    if(to   == NULL ) { return 1; }
    if(user == NULL ) { return 1; }

    PathUserData *p = (PathUserData *)user;

    // Bezier to Line
    for(int i = 0; i <= p->mCtoL ;i++){
        double u = (double)i * 1.0 / (p->mCtoL) ;
        double nu = 1.0 - u;
        double p0 =                  nu * nu * nu;
        double p1 = 3.0 * u *        nu * nu;
        double p2 = 3.0 * u * u *    nu;
        double p3 =       u * u * u;

        double X = (p->mOldP.x) * p0 + (cnt1->x)    * p1 +
                   (cnt2->x   ) * p2 + (to->x  )    * p3;
        double Y = (p->mOldP.y) * p0 + (cnt1->y)    * p1 +
                   (cnt2->y   ) * p2 + (to->y  )    * p3;

        p->mPts.push_back( Point ( ftd(X), ftd(Y) ) );
    }
    p->mOldP = *to;
    return 0;
}

unsigned long BmcpuFreeType2Impl::get_file_size(const char *filename)
{
    unsigned long size;
    FILE * fp = fopen(filename, "rb");
    if (fp==NULL)
    {
        printf("ERROR: Open file %s failed.\n" , filename);
        return 0;
    }
    fseek(fp, SEEK_SET, SEEK_END);
    size= ftell(fp);
    fclose (fp);
    return size;
}

void BmcpuFreeType2Impl::setCardId(int cardid)
{
    m_cardId = cardid;
}

void BmcpuFreeType2Impl::loadFontData(String fontFileName, int id)
{
#if !defined(USING_SOC) && defined(BM1684_CHIP) && defined(ENABLE_BMCPU)
    void *font_buf = NULL;
    m_fontFileName = fontFileName;
    int font_bufsize = get_file_size(m_fontFileName.c_str());

    FILE* file = fopen(m_fontFileName.c_str(), "rb");
    if (!file)
    {
        return;
    }

    char * buf = (char*)malloc(font_bufsize);
    fread(buf, font_bufsize, 1, file);
    fclose(file);
    m_vctHandle.resize(8);
    font_buf = (void*)buf;
    BMCpuSender sender(BM_CARD_ID(m_cardId), (font_bufsize+8192));
    CV_Assert(0 == sender.put(font_buf, font_bufsize));
    CV_Assert(0 == sender.put(m_fontFileName));
    CV_Assert(0 == sender.put(id));
    CV_Assert(0 == sender.put(m_vctHandle));

    CV_Assert(0 == sender.run("bmcpu_loadFontData"));
    CV_Assert(0 == sender.skip(font_buf, font_bufsize));
    CV_Assert(0 == sender.skip(m_fontFileName));
    CV_Assert(0 == sender.skip(id));
    CV_Assert(0 == sender.get(m_vctHandle));
#endif
    return;
}

void BmcpuFreeType2Impl::unloadFontData()
{
#if !defined(USING_SOC) && defined(BM1684_CHIP) && defined(ENABLE_BMCPU)
    char *font_buf = NULL;

    BMCpuSender sender(BM_CARD_ID(m_cardId), 8192);
    CV_Assert(0 == sender.put(m_vctHandle));
    CV_Assert(0 == sender.put(m_fontFileName));
    CV_Assert(0 == sender.run("bmcpu_unloadFontData"));

#endif
    return;
}


void BmcpuFreeType2Impl::setSplitNumber( int num)
{
#if !defined(USING_SOC) && defined(BM1684_CHIP) && defined(ENABLE_BMCPU)

    BMCpuSender sender(BM_CARD_ID(m_cardId), 8192);
    CV_Assert(0 == sender.put(m_vctHandle));
    CV_Assert(0 == sender.put(num));
    CV_Assert(0 == sender.run("bmcpu_setSplitNumber"));

#endif
    return;
}

void BmcpuFreeType2Impl::putText(
    InputOutputArray _img, const String& _text, Point _org,
    int _fontHeight, Scalar _color,
    int _thickness, int _line_type, bool _bottomLeftOrigin
)
{
#if !defined(USING_SOC) && defined(BM1684_CHIP) && defined(ENABLE_BMCPU)

    int img_flag = 0;
    String text = _text.c_str();

    CV_Assert( ( _img.empty()    == false ) &&
               ( _img.isMat()    == true  ) &&
               ( _img.depth()    == CV_8U ) &&
               ( _img.dims()     == 2     ) );
    Mat img = _img.getMat();
    if(!img.avOK()){
        CV_Assert( ( _img.channels() == 3 ) );
    }
    CV_Assert( ( _line_type == CV_AA) ||
               ( _line_type == 4 ) ||
               ( _line_type == 8 ) );
    CV_Assert( _fontHeight >= 0 );

    if ( _text.empty() )
    {
         return;
    }
    if ( _fontHeight == 0 )
    {
         return;
    }

    if (!img.u || !img.u->addr){
        bmcv::attachDeviceMemory(img);
        bmcv::uploadMat(img);
        img_flag = 1;
    }

    BMCpuSender sender(BM_CARD_ID(img.card), 8192);
    CV_Assert(0 == sender.put(m_vctHandle));
    CV_Assert(0 == sender.put(img));
    CV_Assert(0 == sender.put(text));
    CV_Assert(0 == sender.put(_org));
    CV_Assert(0 == sender.put(_fontHeight));
    CV_Assert(0 == sender.put((Scalar &)_color));
    CV_Assert(0 == sender.put(_thickness));
    CV_Assert(0 == sender.put(_line_type));
    CV_Assert(0 == sender.put(_bottomLeftOrigin));

    CV_Assert(0 == sender.run("bmcpu_ft2_putText"));

    if (img_flag){
        bmcv::downloadMat(img);
        if (img.u && CV_XADD(&img.u->refcount, -1) == 1 )
            img.deallocate();
    }
#endif
    return;
}

Size BmcpuFreeType2Impl::getTextSize(
    const String& _text, int fontHeight, int thickness,
    CV_OUT int* baseLine)
{
    Size text_size;
#if !defined(USING_SOC) && defined(BM1684_CHIP) && defined(ENABLE_BMCPU)
    String text = _text.c_str();

    BMCpuSender sender(BM_CARD_ID(m_cardId), 8192);
    CV_Assert(0 == sender.put(m_vctHandle));
    CV_Assert(0 == sender.put(text));
    CV_Assert(0 == sender.put(fontHeight));
    CV_Assert(0 == sender.put(thickness));
    CV_Assert(0 == sender.put(*baseLine));
    CV_Assert(0 == sender.put(text_size));

    CV_Assert(0 == sender.run("bmcpu_getTextSize"));

    CV_Assert(0 == sender.put(m_vctHandle));
    CV_Assert(0 == sender.skip(text));
    CV_Assert(0 == sender.skip(fontHeight));
    CV_Assert(0 == sender.skip(thickness));
    CV_Assert(0 == sender.get(*baseLine));
    CV_Assert(0 == sender.get(text_size));
#endif
    return text_size;
}

CV_EXPORTS_W Ptr<FreeType2> createFreeType2()
{
    return Ptr<FreeType2Impl> (new FreeType2Impl () );
}

CV_EXPORTS_W void* createFreeType2_p()
{
    return (void*) (new FreeType2Impl());
}

}} // namespace freetype2
