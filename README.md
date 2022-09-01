SOPHGO OPENCV README
====================

This is SOPHGO opencv project optimized for SOPHGO BM1682/1684/1684x AI chips. It provides hardware accelerations for video codecs 
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

** Extended support for bmcpu function

    Opencv has a large number of image processing functions implemented on the host cpu, so in the PCIE environment,
    there is a need to exchange and synchronize memory between the host and the board device, and the speed 
    of this memory synchronization is much slower than the memory cache. The data synchronization speed is high, which
    creates a bottleneck for application development in the PCIE environment. And each SOC on our BM168x board has 
    a powerful ARM Cortex A53 processor resource, which is currently idle in the PCIE environment, so BMCPU Opencv 
    tries to map the functions between Host Opencv and Device Opencv, The operation of Host Opencv is actually implemented 
    by the operation of Device Opencv to ensure that all data is carried out in Device Memory,and there is no need to 
    exchange PCIE and host, thereby reducing the pressure on the Host CPU and reducing the processing performance of 
    the CPU processor. requirements, and on the other hand improve the operating speed and eliminate the bottleneck
    caused by the PCIE bandwidth.

** Several domenstic CPUs are supported

    loongson、SW、PHYTIUM、x86、arm.

** Many improvements are customized for Chinese market

##build

Compilation steps:
 1、Create folder: mkdir build;cd build
 2、Create a shell script and copy the following to the shell script

#!/bin/bash
PWD=$(pwd)

echo $PWD

echo $PWD/../extern_lib/prebuilt/x86_64/python3.5/lib/libpython3.5m.so

MM_TOP_DIR=$PWD/../extern_lib/

MM_OUTPUT_DIR=$PWD/install

OPENCV_PATH=$PWD/../

OPENCV_INSTALL_PATH=$PWD/install

CHIP=bm1684

SUBTYPE=asic

PRODUCTFORM=soc

OPENCV_CONTRIB_ENABLE_NONFREE=OFF

BUILD_TYPE=release

if [ $PRODUCTFORM != soc ]; then
    HWACCEL_PATH=$PWD/../extern_lib/bm_hardware_accele/decode_x86_64
    ENABLE_BMCPU=1
    cmake -DHAVE_opencv_python3=ON -DBUILD_opencv_python3=ON \
          -DPYTHON3_INCLUDE_PATH=$MM_TOP_DIR/prebuilt/x86_64/python3.5/include/python3.5m \
          -DPYTHON3_LIBRARIES=$MM_TOP_DIR/prebuilt/x86_64/python3.5/lib/libpython3.5m.so \
          -DPYTHON3_EXECUTABLE=$MM_TOP_DIR/prebuilt/x86_64/python3.5/bin\
          -DPYTHON3_NUMPY_INCLUDE_DIRS=$MM_TOP_DIR/prebuilt/x86_64/python3.5/dist-packages/numpy/core/include \
          -DPYTHON3_PACKAGES_PATH=$MM_OUTPUT_DIR/opencv-python \
          -DHAVE_opencv_python2=ON -DBUILD_opencv_python2=ON \
          -DPYTHON2_INCLUDE_PATH=$MM_TOP_DIR/prebuilt/x86_64/python2.7/include/python2.7 \
          -DPYTHON2_LIBRARIES=$MM_TOP_DIR/prebuilt/x86_64/python2.7/lib/libpython2.7.so \
          -DPYTHON2_NUMPY_INCLUDE_DIRS=$MM_TOP_DIR/prebuilt/x86_64/python2.7/dist-packages/numpy/core/include \
          -DPYTHON2_EXECUTABLE=$MM_TOP_DIR/prebuilt/x86_64/python2.7/bin\
          -DPYTHON2_PACKAGES_PATH=$MM_OUTPUT_DIR/opencv-python/python2 \
          -DPYTHON_DEFAULT_EXECUTABLE=/usr/bin/python \
          -DOPENCV_SKIP_PYTHON_LOADER=ON \
          -DWITH_FFMPEG=ON -DWITH_GSTREAMER=OFF \
          -DWITH_GTK=OFF -DWITH_1394=OFF -DWITH_V4L=ON \
          -DWITH_CUDA=OFF -DWITH_OPENCL=OFF -DWITH_LAPACK=OFF \
          -DWITH_TBB=ON -DBUILD_TBB=ON \
          -DWITH_TIFF=ON -DBUILD_TIFF=ON \
          -DWITH_JPEG=ON -DBUILD_JPEG=OFF \
          -DOPENCV_FORCE_3RDPARTY_BUILD=ON \
          -DJPEG_LIBRARY=$OPENCV_PATH/3rdparty/libjpeg-turbo/binary/lib/pcie/libturbojpeg.a \
          -DJPEG_INCLUDE_DIR=$OPENCV_PATH/3rdparty/libjpeg-turbo/binary/include \
          -DFREETYPE_LIBRARY=$MM_TOP_DIR/prebuilt/x86_64/lib/libfreetype.a \
          -DFREETYPE_INCLUDE_DIRS=$MM_TOP_DIR/prebuilt/include/freetype2 \
          -DHARFBUZZ_INCLUDE_DIRS=$MM_TOP_DIR/prebuilt/include/harfbuzz \
          -DCMAKE_TOOLCHAIN_FILE=../platforms/linux/x86_64-gnu.toolchain.cmake \
          -DCMAKE_INSTALL_PREFIX=$OPENCV_INSTALL_PATH \
          -DCMAKE_HWACCEL_PATH=$HWACCEL_PATH \
          -DCMAKE_MAKE_PROGRAM=make \
          -DCHIP=${CHIP} \
          -DSUBTYPE=${SUBTYPE} \
          -DPRODUCTFORM=${PRODUCTFORM} \
          -DENABLE_BMCPU=${ENABLE_BMCPU} \
          -DOPENCV_EXTRA_MODULES_PATH=${OPENCV_CONTRIB_PATH} \
          -DOPENCV_ENABLE_NONFREE=${OPENCV_CONTRIB_ENABLE_NONFREE} \
          -DOPENCV_GENERATE_PKGCONFIG=ON \
          -DCMAKE_BUILD_TYPE=${BUILD_TYPE} \
          -DABI_FLAG=${ABI_FLAG} ..

else
    HWACCEL_PATH=$PWD/../extern_lib/bm_hardware_accele/decode_arm64
    ENABLE_BMCPU=0
    cmake -DHAVE_opencv_python3=ON -DBUILD_opencv_python3=ON \
          -DPYTHON3_INCLUDE_PATH=$MM_TOP_DIR/prebuilt/include/python3.5 \
          -DPYTHON3_LIBRARIES=$MM_TOP_DIR/prebuilt/lib/libpython3.5m.so \
          -DPYTHON3_EXECUTABLE=$MM_TOP_DIR/prebuilt/bin/python3 \
          -DPYTHON3_NUMPY_INCLUDE_DIRS=$MM_TOP_DIR/prebuilt/lib/local/python3.5/dist-packages/numpy/core/include \
          -DPYTHON3_PACKAGES_PATH=$MM_OUTPUT_DIR/opencv-python \
          -DHAVE_opencv_python2=ON -DBUILD_opencv_python2=ON \
          -DPYTHON2_INCLUDE_PATH=$MM_TOP_DIR/prebuilt/include/python2.7 \
          -DPYTHON2_LIBRARIES=$MM_TOP_DIR/prebuilt/lib/libpython2.7.so \
          -DPYTHON2_NUMPY_INCLUDE_DIRS=$MM_TOP_DIR/prebuilt/lib/local/python2.7/dist-packages/numpy/core/include \
          -DPYTHON2_EXECUTABLE=$MM_TOP_DIR/prebuilt/bin/python2.7 \
          -DPYTHON2_PACKAGES_PATH=$MM_OUTPUT_DIR/opencv-python/python2 \
          -DPYTHON_DEFAULT_EXECUTABLE=/usr/bin/python \
          -DOPENCV_SKIP_PYTHON_LOADER=ON \
          -DWITH_FFMPEG=ON -DWITH_GSTREAMER=OFF \
          -DWITH_GTK=OFF -DWITH_1394=OFF -DWITH_V4L=ON \
          -DWITH_CUDA=OFF -DWITH_OPENCL=OFF \
          -DWITH_TBB=ON -DBUILD_TBB=ON \
          -DWITH_TIFF=ON -DBUILD_TIFF=ON \
          -DWITH_JPEG=ON -DBUILD_JPEG=OFF \
          -DJPEG_LIBRARY=$OPENCV_PATH/3rdparty/libjpeg-turbo/binary/lib/soc/libturbojpeg.a \
          -DJPEG_INCLUDE_DIR=$OPENCV_PATH/3rdparty/libjpeg-turbo/binary/include \
          -DFREETYPE_LIBRARY=$MM_TOP_DIR/prebuilt/lib/libfreetype.a \
          -DFREETYPE_INCLUDE_DIRS=$MM_TOP_DIR/prebuilt/include/freetype2 \
          -DHARFBUZZ_INCLUDE_DIRS=$MM_TOP_DIR/prebuilt/include/harfbuzz \
          -DCMAKE_TOOLCHAIN_FILE=../platforms/linux/aarch64-gnu.toolchain.cmake \
          -DCMAKE_INSTALL_PREFIX=$MM_OUTPUT_DIR/opencv \
          -DCMAKE_HWACCEL_PATH=$HWACCEL_PATH \
          -DCMAKE_MAKE_PROGRAM=make \
          -DENABLE_NEON=ON \
          -DCHIP=${CHIP} \
          -DSUBTYPE=${SUBTYPE} \
          -DPRODUCTFORM=${PRODUCTFORM} \
          -DENABLE_BMCPU=${ENABLE_BMCPU} \
          -DOPENCV_EXTRA_MODULES_PATH=${OPENCV_CONTRIB_PATH} \
          -DOPENCV_ENABLE_NONFREE=${OPENCV_CONTRIB_ENABLE_NONFREE} \
          -DCMAKE_BUILD_TYPE=${BUILD_TYPE} ..
fi

## License

This project is licensed under the Apache License Version 2.0. Please refere to the LICNESE file for detailed information. 

## Authorts

	- xun.li  
	- tao.han
	- yujing.shen
	- yuyuan.yang
    - haotian.luo
    - mingxi.shen
    - yu.yang
	
## Contributing

This project is maintained by Sophgo multimedia team. Welcome to submit codes and patches to us by email to yujing.shen@sophgo.com



