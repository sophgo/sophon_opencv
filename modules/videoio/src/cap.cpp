/*M///////////////////////////////////////////////////////////////////////////////////////
//
//  IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
//
//  By downloading, copying, installing or using the software you agree to this license.
//  If you do not agree to this license, do not download, install,
//  copy or use the software.
//
//
//                        Intel License Agreement
//                For Open Source Computer Vision Library
//
// Copyright (C) 2000, Intel Corporation, all rights reserved.
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

#include "opencv2/videoio/registry.hpp"
#include "videoio_registry.hpp"

namespace cv {

static bool param_VIDEOIO_DEBUG = utils::getConfigurationParameterBool("OPENCV_VIDEOIO_DEBUG", false);
static bool param_VIDEOCAPTURE_DEBUG = utils::getConfigurationParameterBool("OPENCV_VIDEOCAPTURE_DEBUG", false);
static bool param_VIDEOWRITER_DEBUG = utils::getConfigurationParameterBool("OPENCV_VIDEOWRITER_DEBUG", false);


void DefaultDeleter<CvCapture>::operator ()(CvCapture* obj) const { cvReleaseCapture(&obj); }
void DefaultDeleter<CvVideoWriter>::operator ()(CvVideoWriter* obj) const { cvReleaseVideoWriter(&obj); }


VideoCapture::VideoCapture()
{}

VideoCapture::VideoCapture(const String& filename, int apiPreference, int id)
{
    CV_TRACE_FUNCTION();
    open(filename, apiPreference, id);
}

VideoCapture::VideoCapture(int index, int apiPreference)
{
    CV_TRACE_FUNCTION();
    open(index, apiPreference);
}

VideoCapture::~VideoCapture()
{
    CV_TRACE_FUNCTION();
    release();
}

bool VideoCapture::open(const String& filename, int apiPreference, int id)
{
    CV_TRACE_FUNCTION();

    if (isOpened()) release();

    const std::vector<VideoBackendInfo> backends = cv::videoio_registry::getAvailableBackends_CaptureByFilename();
    for (size_t i = 0; i < backends.size(); i++)
    {
        const VideoBackendInfo& info = backends[i];
        if (apiPreference == CAP_ANY || apiPreference == info.id)
        {
            if (param_VIDEOIO_DEBUG || param_VIDEOCAPTURE_DEBUG)
                CV_LOG_WARNING(NULL, cv::format("VIDEOIO(%s): trying capture filename='%s' ...", info.name, filename.c_str()));
            CV_Assert(!info.backendFactory.empty());
            const Ptr<IBackend> backend = info.backendFactory->getBackend();
            if (!backend.empty())
            {
                try
                {
                    icap = backend->createCapture(filename, id);
                    if (!icap.empty())
                    {
                        if (param_VIDEOIO_DEBUG || param_VIDEOCAPTURE_DEBUG)
                            CV_LOG_WARNING(NULL, cv::format("VIDEOIO(%s): created, isOpened=%d",
                                                            info.name, icap->isOpened()));
                        if (icap->isOpened())
                            return true;
                        icap.release();
                    }
                    else
                    {
                        if (param_VIDEOIO_DEBUG || param_VIDEOCAPTURE_DEBUG)
                            CV_LOG_WARNING(NULL, cv::format("VIDEOIO(%s): can't create capture", info.name));
                    }
                } catch(const cv::Exception& e) {
                    CV_LOG_ERROR(NULL, cv::format("VIDEOIO(%s): raised OpenCV exception:\n\n%s\n", info.name, e.what()));
                } catch (const std::exception& e) {
                    CV_LOG_ERROR(NULL, cv::format("VIDEOIO(%s): raised C++ exception:\n\n%s\n", info.name, e.what()));
                } catch(...) {
                    CV_LOG_ERROR(NULL, cv::format("VIDEOIO(%s): raised unknown C++ exception!\n\n", info.name));
                }
            }
            else
            {
                if (param_VIDEOIO_DEBUG || param_VIDEOCAPTURE_DEBUG) \
                    CV_LOG_WARNING(NULL, cv::format("VIDEOIO(%s): backend is not available (plugin is missing, or can't be loaded due dependencies or it is not compatible)", info.name));
            }
        }
    }
    return false;
}

bool  VideoCapture::open(int cameraNum, int apiPreference)
{
    CV_TRACE_FUNCTION();

    if (isOpened()) release();

    if (apiPreference == CAP_ANY)
    {
        // interpret preferred interface (0 = autodetect)
        int backendID = (cameraNum / 100) * 100;
        if (backendID)
        {
            cameraNum %= 100;
            apiPreference = backendID;
        }
    }

    const std::vector<VideoBackendInfo> backends = cv::videoio_registry::getAvailableBackends_CaptureByIndex();
    for (size_t i = 0; i < backends.size(); i++)
    {
        const VideoBackendInfo& info = backends[i];
        if (apiPreference == CAP_ANY || apiPreference == info.id)
        {
            if (param_VIDEOIO_DEBUG || param_VIDEOCAPTURE_DEBUG)
                CV_LOG_WARNING(NULL, cv::format("VIDEOIO(%s): trying capture cameraNum=%d ...", info.name, cameraNum));
            CV_Assert(!info.backendFactory.empty());
            const Ptr<IBackend> backend = info.backendFactory->getBackend();
            if (!backend.empty())
            {
                try
                {
                    icap = backend->createCapture(cameraNum);
                    if (!icap.empty())
                    {
                        if (param_VIDEOIO_DEBUG || param_VIDEOCAPTURE_DEBUG)
                            CV_LOG_WARNING(NULL, cv::format("VIDEOIO(%s): created, isOpened=%d",
                                                            info.name, icap->isOpened()));
                        if (icap->isOpened())
                            return true;
                        icap.release();
                    }
                    else
                    {
                        if (param_VIDEOIO_DEBUG || param_VIDEOCAPTURE_DEBUG)
                            CV_LOG_WARNING(NULL, cv::format("VIDEOIO(%s): can't create capture", info.name));
                    }
                } catch(const cv::Exception& e) {
                    CV_LOG_ERROR(NULL, cv::format("VIDEOIO(%s): raised OpenCV exception:\n\n%s\n", info.name, e.what()));
                } catch (const std::exception& e) {
                    CV_LOG_ERROR(NULL, cv::format("VIDEOIO(%s): raised C++ exception:\n\n%s\n", info.name, e.what()));
                } catch(...) {
                    CV_LOG_ERROR(NULL, cv::format("VIDEOIO(%s): raised unknown C++ exception!\n\n", info.name));
                }
            }
            else
            {
                if (param_VIDEOIO_DEBUG || param_VIDEOCAPTURE_DEBUG) \
                    CV_LOG_WARNING(NULL, cv::format("VIDEOIO(%s): backend is not available (plugin is missing, or can't be loaded due dependencies or it is not compatible)", info.name));
            }
        }
    }
    return false;
}

bool VideoCapture::isOpened() const
{
    return !icap.empty() ? icap->isOpened() : false;
}

String VideoCapture::getBackendName() const
{
    int api = 0;
    if (icap)
        api = icap->isOpened() ? icap->getCaptureDomain() : 0;
    CV_Assert(api != 0);
    return cv::videoio_registry::getBackendName((VideoCaptureAPIs)api);
}

void VideoCapture::release()
{
    CV_TRACE_FUNCTION();
    if (isOpened() && (icap.use_count() == 1))
        icap->release(); // IVideoCapture release resouce. Here thread competetion risk exists for this patch. But capture is seldom shared with other thread, so this is acceptable. 
    icap.release(); // shared_ptr release pointer
}

bool VideoCapture::grab()
{
    CV_INSTRUMENT_REGION();
    return !icap.empty() ? icap->grabFrame() : false;
}

bool VideoCapture::grab(char *buf, unsigned int len_in, unsigned int *len_out)
{
    CV_INSTRUMENT_REGION();
    if ((buf == NULL) || (len_out == NULL)) {
        return grab();
    }

    return !icap.empty() ? icap->grabFrame(buf, len_in, len_out) : false;
}
bool VideoCapture::retrieve(OutputArray image, int channel)
{
    CV_INSTRUMENT_REGION();
    if (!icap.empty())
        return icap->retrieveFrame(channel, image);
    return false;
}

bool VideoCapture::read(OutputArray image)
{
    CV_INSTRUMENT_REGION();

    if(grab())
        retrieve(image);
    else
        image.release();
    return !image.empty();
}

bool VideoCapture::read_record(OutputArray image, char *buf, unsigned int len_in, unsigned int *len_out)
{
    CV_INSTRUMENT_REGION();
    if ((buf == NULL) || (len_out == NULL)) {
        return read(image);
    }

    if(grab(buf, len_in, len_out))
        retrieve(image);
    else
        image.release();
    return !image.empty();
}

bool VideoCapture::set_resampler(int den, int num)
{
    return !icap.empty() ? icap->SetResampler(den, num) : false;
}

bool VideoCapture::get_resampler(int *den, int *num)
{
    return !icap.empty() ? icap->GetResampler(den, num) : false;
}

VideoCapture& VideoCapture::operator >> (Mat& image)
{
#ifdef WINRT_VIDEO
    // FIXIT grab/retrieve methods() should work too
    if (grab())
    {
        if (retrieve(image))
        {
            std::lock_guard<std::mutex> lock(VideoioBridge::getInstance().inputBufferMutex);
            VideoioBridge& bridge = VideoioBridge::getInstance();

            // double buffering
            bridge.swapInputBuffers();
            auto p = bridge.frontInputPtr;

            bridge.bIsFrameNew = false;

            // needed here because setting Mat 'image' is not allowed by OutputArray in read()
            Mat m(bridge.getHeight(), bridge.getWidth(), CV_8UC3, p);
            image = m;
        }
    }
#else
    read(image);
#endif
    return *this;
}

VideoCapture& VideoCapture::operator >> (UMat& image)
{
    CV_INSTRUMENT_REGION();

    read(image);
    return *this;
}

bool VideoCapture::set(int propId, double value)
{
    CV_CheckNE(propId, (int)CAP_PROP_BACKEND, "Can't set read-only property");
    return !icap.empty() ? icap->setProperty(propId, value) : false;
}

double VideoCapture::get(int propId) const
{
    if (propId == CAP_PROP_BACKEND)
    {
        int api = 0;
        if (icap)
            api = icap->isOpened() ? icap->getCaptureDomain() : 0;
        if (api <= 0)
            return -1.0;
        return (double)api;
    }
    return !icap.empty() ? icap->getProperty(propId) : 0;
}


//=================================================================================================



VideoWriter::VideoWriter()
{}

VideoWriter::VideoWriter(const String& filename, int _fourcc, double fps, Size frameSize, bool isColor, int id)
{
    open(filename, _fourcc, fps, frameSize, isColor, id);
}


VideoWriter::VideoWriter(const String& filename, int apiPreference, int _fourcc, double fps, Size frameSize, bool isColor, int id)
{
    open(filename, apiPreference, _fourcc, fps, frameSize, isColor, id);
}

void VideoWriter::release()
{
    iwriter.release();
}

VideoWriter::~VideoWriter()
{
    release();
}

bool VideoWriter::open(const String& filename, int _fourcc, double fps, Size frameSize, bool isColor, int id)
{
    return open(filename, CAP_ANY, _fourcc, fps, frameSize, isColor, id);
}

bool VideoWriter::open(const String& filename, int _fourcc, double fps, Size frameSize, 
                       const String& encodeParams, bool isColor, int id)
{
    CV_INSTRUMENT_REGION();

    if (isOpened()) release();
    int apiPreference = CAP_ANY;
    const std::vector<VideoBackendInfo> backends = cv::videoio_registry::getAvailableBackends_Writer();
    for (size_t i = 0; i < backends.size(); i++)
    {
        const VideoBackendInfo& info = backends[i];
        if(strcmp(info.name,"FFMPEG") != 0){
            continue;
        }
        if (apiPreference == CAP_ANY || apiPreference == info.id)
        {
            if (param_VIDEOIO_DEBUG || param_VIDEOWRITER_DEBUG)
                CV_LOG_WARNING(NULL, cv::format("VIDEOIO(%s): trying writer with filename='%s' fourcc=0x%08x fps=%g sz=%dx%d isColor=%d...",
                                                info.name, filename.c_str(), (unsigned)_fourcc, fps, frameSize.width, frameSize.height, (int)isColor));
            CV_Assert(!info.backendFactory.empty());
            const Ptr<IBackend> backend = info.backendFactory->getBackend();
            if (!backend.empty())
            {
                try
                {
                    iwriter = backend->createWriter(filename, _fourcc, fps, frameSize, isColor, id, encodeParams);
                    if (!iwriter.empty())
                    {
                        if (param_VIDEOIO_DEBUG || param_VIDEOWRITER_DEBUG)
                            CV_LOG_WARNING(NULL, cv::format("VIDEOIO(%s): created, isOpened=%d",
                                                            info.name, iwriter->isOpened()));
                        if (iwriter->isOpened())
                            return true;
                        iwriter.release();
                    }
                    else
                    {
                        if (param_VIDEOIO_DEBUG || param_VIDEOWRITER_DEBUG)
                            CV_LOG_WARNING(NULL, cv::format("VIDEOIO(%s): can't create writer", info.name));
                    }
                } catch(const cv::Exception& e) {
                    CV_LOG_ERROR(NULL, cv::format("VIDEOIO(%s): raised OpenCV exception:\n\n%s\n", info.name, e.what()));
                } catch (const std::exception& e) {
                    CV_LOG_ERROR(NULL, cv::format("VIDEOIO(%s): raised C++ exception:\n\n%s\n", info.name, e.what()));
                } catch(...) {
                    CV_LOG_ERROR(NULL, cv::format("VIDEOIO(%s): raised unknown C++ exception!\n\n", info.name));
                }
            }
            else
            {
                if (param_VIDEOIO_DEBUG || param_VIDEOWRITER_DEBUG) \
                    CV_LOG_WARNING(NULL, cv::format("VIDEOIO(%s): backend is not available (plugin is missing, or can't be loaded due dependencies or it is not compatible)", info.name));
            }
        }
    }
    return false;
}


bool VideoWriter::open(const String& filename, int apiPreference, int _fourcc, double fps, Size frameSize, bool isColor, int id)
{
    CV_INSTRUMENT_REGION();

    if (isOpened()) release();

    const std::vector<VideoBackendInfo> backends = cv::videoio_registry::getAvailableBackends_Writer();
    for (size_t i = 0; i < backends.size(); i++)
    {
        const VideoBackendInfo& info = backends[i];
        if (apiPreference == CAP_ANY || apiPreference == info.id)
        {
            if (param_VIDEOIO_DEBUG || param_VIDEOWRITER_DEBUG)
                CV_LOG_WARNING(NULL, cv::format("VIDEOIO(%s): trying writer with filename='%s' fourcc=0x%08x fps=%g sz=%dx%d isColor=%d...",
                                                info.name, filename.c_str(), (unsigned)_fourcc, fps, frameSize.width, frameSize.height, (int)isColor));
            CV_Assert(!info.backendFactory.empty());
            const Ptr<IBackend> backend = info.backendFactory->getBackend();
            if (!backend.empty())
            {
                try
                {
                    iwriter = backend->createWriter(filename, _fourcc, fps, frameSize, isColor, id);
                    if (!iwriter.empty())
                    {
                        if (param_VIDEOIO_DEBUG || param_VIDEOWRITER_DEBUG)
                            CV_LOG_WARNING(NULL, cv::format("VIDEOIO(%s): created, isOpened=%d",
                                                            info.name, iwriter->isOpened()));
                        if (iwriter->isOpened())
                            return true;
                        iwriter.release();
                    }
                    else
                    {
                        if (param_VIDEOIO_DEBUG || param_VIDEOWRITER_DEBUG)
                            CV_LOG_WARNING(NULL, cv::format("VIDEOIO(%s): can't create writer", info.name));
                    }
                } catch(const cv::Exception& e) {
                    CV_LOG_ERROR(NULL, cv::format("VIDEOIO(%s): raised OpenCV exception:\n\n%s\n", info.name, e.what()));
                } catch (const std::exception& e) {
                    CV_LOG_ERROR(NULL, cv::format("VIDEOIO(%s): raised C++ exception:\n\n%s\n", info.name, e.what()));
                } catch(...) {
                    CV_LOG_ERROR(NULL, cv::format("VIDEOIO(%s): raised unknown C++ exception!\n\n", info.name));
                }
            }
            else
            {
                if (param_VIDEOIO_DEBUG || param_VIDEOWRITER_DEBUG) \
                    CV_LOG_WARNING(NULL, cv::format("VIDEOIO(%s): backend is not available (plugin is missing, or can't be loaded due dependencies or it is not compatible)", info.name));
            }
        }
    }
    return false;
}

bool VideoWriter::isOpened() const
{
    return !iwriter.empty();
}


bool VideoWriter::set(int propId, double value)
{
    CV_CheckNE(propId, (int)CAP_PROP_BACKEND, "Can't set read-only property");

    if (!iwriter.empty())
        return iwriter->setProperty(propId, value);
    return false;
}

double VideoWriter::get(int propId) const
{
    if (propId == CAP_PROP_BACKEND)
    {
        int api = 0;
        if (iwriter)
            api = iwriter->getCaptureDomain();
        if (api <= 0)
            return -1.0;
        return (double)api;
    }
    if (!iwriter.empty())
        return iwriter->getProperty(propId);
    return 0.;
}

String VideoWriter::getBackendName() const
{
    int api = 0;
    if (iwriter)
        api = iwriter->getCaptureDomain();
    CV_Assert(api != 0);
    return cv::videoio_registry::getBackendName((VideoCaptureAPIs)api);
}

void VideoWriter::write(InputArray image)
{
    CV_INSTRUMENT_REGION();

    if( iwriter )
        iwriter->write(image);
}

void VideoWriter::write(InputArray image, char *data, int *len)
{
    CV_INSTRUMENT_REGION();

    if( iwriter )
        iwriter->write(image, data, len);
}

void VideoWriter::write(InputArray image, char *data, int *len, CV_RoiInfo *roiinfo)
{
    CV_INSTRUMENT_REGION();

    if( iwriter )
        iwriter->write(image, data, len, (void*)roiinfo);
}

VideoWriter& VideoWriter::operator << (const Mat& image)
{
    CV_INSTRUMENT_REGION();

    write(image);
    return *this;
}

VideoWriter& VideoWriter::operator << (const UMat& image)
{
    CV_INSTRUMENT_REGION();
    write(image);
    return *this;
}

// FIXIT OpenCV 4.0: make inline
int VideoWriter::fourcc(char c1, char c2, char c3, char c4)
{
    return (c1 & 255) + ((c2 & 255) << 8) + ((c3 & 255) << 16) + ((c4 & 255) << 24);
}

} // namespace
