SOPHGO OPENCV README
====================

This is SOPHGO opencv project optimized for SOPHGO BM1688 AI chips based on opencv-4.8.0. It provides hardware accelerations for video codecs 
and image processing. Also some enhancement for streaming protocal are included and yuv mat extend. 

## Changes
With officical opecv, following changes are done in this project. 

** Supports hardware acceleration of picture h264 h265 video codec

    About video support h264 h265 encoding and decoding, about picture support jpeg picture encode and decode

** Support vpp(Video Post Processor) hardware acceleration

    The main functions of vpp include:crop、reisze、csc、pixel sample convert、rectangle border、padding、video data compression.

** Extended support for sophon yuv mat 

** Support GB28181 protocol

    Support to pull GB28181 h264 h265 video stream as GB28181 client.

** Support rtsp rtmp push stream

    When the encoding is done through hardware acceleration, the stream can be pushed out of the videoWrite by rtsp rtmp.

** Many improvements are customized for Chinese market

## build

download toolchain from linaro gnu toolchain
```base
wget https://releases.linaro.org/components/toolchain/gcc-linaro/6.3-2017.05/gcc-linaro-6.3-2017.05.tar.xz
```

```bash
#!/bin/bash

extern_lib_path="${PWD}/extern_lib"

mkdir build
cd build

cmake -DCHIP=bm1688         \
    -DPRODUCTFORM=soc       \
    -DWITH_FFMPEG=ON        \
    -DHAVE_FFMPEG=ON        \
    -DWITH_GSTREAMER=OFF    \
    -DWITH_GTK=OFF          \
    -DWITH_1394=OFF         \
    -DWITH_V4L=ON           \
    -DWITH_CUDA=OFF         \
    -DWITH_OPENCL=OFF       \
    -DWITH_LAPACK=OFF       \
    -DWITH_TBB=ON           \
    -DBUILD_TBB=ON          \
    -DWITH_TIFF=ON          \
    -DBUILD_TIFF=ON         \
    -DWITH_JPEG=ON          \
    -DOPENCV_GENERATE_PKGCONFIG=ON  \
    -DFFMPEG_INCLUDE_DIRS="${extern_lib_path}/3rdparty/ffmpeg/include;          \
                            ${extern_lib_path}/3rdparty/libbmlib/include;       \
                            ${extern_lib_path}/3rdparty/libyuv/include;         \
                            ${extern_lib_path}/3rdparty/libjpeg-turbo/include;  \
                            ${extern_lib_path}/hardware/jpeg/inc;               \
                            ${extern_lib_path}/hardware/video/dec/inc;          \
                            ${extern_lib_path}/hardware/video/enc/inc;          \
                            ${extern_lib_path}/hardware/bmcv/include"           \
    -DFFMPEG_LIBRARY_DIRS="${extern_lib_path}/3rdparty/ffmpeg/lib;              \
                            ${extern_lib_path}/3rdparty/libbmlib/lib;           \
                            ${extern_lib_path}/3rdparty/libyuv/lib;             \
                            ${extern_lib_path}/hardware/bmcv/lib;               \
                            ${extern_lib_path}/hardware/jpeg/lib;               \
                            ${extern_lib_path}/hardware/video/dec/lib;          \
                            ${extern_lib_path}/hardware/video/enc/lib"          \
    -DBUILD_JPEG=OFF            \
    -DABI_FLAG=1                \
    -DOPENCV_ENABLE_NONFREE=OFF \
    -DCMAKE_MAKE_PROGRAM=make   \
    -DHAVE_opencv_python3=ON    \
    -DBUILD_opencv_python3=ON   \
    -DPYTHON3_INCLUDE_PATH=${extern_lib_path}/prebuilt/include/python3.5   \
    -DPYTHON3_LIBRARIES=${extern_lib_path}/prebuilt/lib/libpython3.5m.so   \
    -DPYTHON3_EXECUTABLE=${extern_lib_path}/prebuilt/bin/python3           \
    -DPYTHON3_NUMPY_INCLUDE_DIRS=${extern_lib_path}/prebuilt/lib/local/python3.5/dist-packages/numpy/core/include \
    -DPYTHON3_PACKAGES_PATH=${PWD}/opencv-python                           \
    -DHAVE_opencv_python2=ON                                               \
    -DBUILD_opencv_python2=ON                                              \
    -DPYTHON2_INCLUDE_PATH=${extern_lib_path}/prebuilt/include/python2.7   \
    -DPYTHON2_LIBRARIES=${extern_lib_path}/prebuilt/lib/libpython2.7.so    \
    -DPYTHON2_NUMPY_INCLUDE_DIRS=${extern_lib_path}/prebuilt/lib/local/python2.7/dist-packages/numpy/core/include \
    -DPYTHON2_EXECUTABLE=${extern_lib_path}/prebuilt/bin/python2.7         \
    -DPYTHON2_PACKAGES_PATH=${PWD}/opencv-python/python2                   \
    -DPYTHON_DEFAULT_EXECUTABLE=/usr/bin/python                            \
    -DOPENCV_SKIP_PYTHON_LOADER=ON                                         \
    -DENABLE_NEON=ON                                                       \
    -DOPENCV_FORCE_3RDPARTY_BUILD=OFF                                      \
    -DJPEG_LIBRARY=${extern_lib_path}/3rdparty/libjpeg-turbo/lib/libturbojpeg.a \
    -DJPEG_INCLUDE_DIR=${extern_lib_path}/3rdparty/libjpeg-turbo/include   \
    -DFREETYPE_LIBRARY=${extern_lib_path}/prebuilt/lib/libfreetype.a       \
    -DFREETYPE_INCLUDE_DIRS=${extern_lib_path}/prebuilt/include/freetype2  \
    -DHARFBUZZ_INCLUDE_DIRS=${extern_lib_path}/prebuilt/include/harfbuzz   \
    -DCMAKE_TOOLCHAIN_FILE=../platforms/linux/aarch64-gnu.toolchain.cmake  \
    ..

core_number=`cat /proc/cpuinfo | grep "processor" | wc -l`
make -j${core_number}

make install


```

## License

This project is licensed under the Apache License Version 2.0. Please refere to the LICNESE file for detailed information. 

## Authors

Sophgo multimedia team
	
## Contributing

This project is maintained by Sophgo multimedia team. Welcome to submit codes and patches to us by email to yujing.shen@sophgo.com