
if(CMAKE_SYSTEM_NAME MATCHES "Windows")
    set(CMAKE_IMPORT_LIBRARY_PREFIX lib)
    if (NOT DEFINED FFMPEG_INCLUDE_DIRS)
        set(FFMPEG_INCLUDE_DIRS
            "${OpenCV_SOURCE_DIR}/3rdparty/libbmcv/include"
            "${OpenCV_SOURCE_DIR}/out/ffmpeg/usr/local/include"
            "${OpenCV_SOURCE_DIR}/out/decode/include"
            "${OpenCV_SOURCE_DIR}/out/vpp/include"
        )
    else()
        string(REPLACE " " ";" FFMPEG_INCLUDE_DIRS ${FFMPEG_INCLUDE_DIRS})
    endif()
    if (NOT DEFINED FFMPEG_LIBRARY_DIRS)
        set(FFMPEG_LIBRARY_DIRS
            "${OpenCV_SOURCE_DIR}/3rdparty/libbmcv/lib/${PRODUCTFORM}"
            "${OpenCV_SOURCE_DIR}/out/ffmpeg/usr/local/lib"
            "${OpenCV_SOURCE_DIR}/out/decode/lib"
        )
    else()
        string(REPLACE " " ";" FFMPEG_LIBRARY_DIRS ${FFMPEG_LIBRARY_DIRS})
    endif()
    LINK_DIRECTORIES(${FFMPEG_LIBRARY_DIRS})
else() #"Linux"

    if (NOT DEFINED FFMPEG_INCLUDE_DIRS)
        set(FFMPEG_INCLUDE_DIRS
            "${BM_HWACCEL_DIR}/include"
        )
    else()
        string(REPLACE " " ";" FFMPEG_INCLUDE_DIRS ${FFMPEG_INCLUDE_DIRS})
    endif()
    if (NOT DEFINED FFMPEG_LIBRARY_DIRS)
        set(FFMPEG_LIBRARY_DIRS
            "${BM_HWACCEL_DIR}/lib"
        )
    else()
        string(REPLACE " " ";" FFMPEG_LIBRARY_DIRS ${FFMPEG_LIBRARY_DIRS})
    endif()
endif()
if(${CHIP} STREQUAL "bm1682")
  set(FFMPEG_LIBRARIES avcodec avformat avutil swscale swresample bmjpulite bmjpuapi bmvideo yuv)
endif()
if(${CHIP} STREQUAL "bm1684")
  set(FFMPEG_LIBRARIES avcodec avformat avutil swscale swresample
  ${CMAKE_IMPORT_LIBRARY_PREFIX}bmcv
  ${CMAKE_IMPORT_LIBRARY_PREFIX}bmlib
  ${CMAKE_IMPORT_LIBRARY_PREFIX}bmjpulite
  ${CMAKE_IMPORT_LIBRARY_PREFIX}bmjpuapi
  ${CMAKE_IMPORT_LIBRARY_PREFIX}bmvpulite
  ${CMAKE_IMPORT_LIBRARY_PREFIX}bmvpuapi
  ${CMAKE_IMPORT_LIBRARY_PREFIX}bmvideo
  ${CMAKE_IMPORT_LIBRARY_PREFIX}yuv)

  list(APPEND FFMPEG_LIBRARIES
  ${CMAKE_IMPORT_LIBRARY_PREFIX}bmion
  ${CMAKE_IMPORT_LIBRARY_PREFIX}bmvppapi)
endif()

if(${PRODUCTFORM} STREQUAL "soc")
  #set(USING_SOC ON)
  #set(HAVE_LIBYUV ON)
  add_definitions(-DUSING_SOC)
  add_definitions(-DHAVE_LIBYUV)
  message(STATUS ${CMAKE_TOOLCHAIN_FILE})
endif()

if(${CHIP} STREQUAL "bm1684")
    if(${BUILD_opencv_world})
        set(HAVE_BMCV OFF)
    else()
        set(HAVE_BMCV ON)
    endif()
    if(${ENABLE_BMCPU})
        add_definitions(-DENABLE_BMCPU)
    endif()
  add_definitions(-DBM1684_CHIP)
endif()

if(${CHIP} STREQUAL "bm1682")
  add_definitions(-DVPP_BM1682)
endif()
if(${CHIP} STREQUAL "bm1684")
  add_definitions(-DVPP_BM1684)
endif()
if(${CHIP} STREQUAL "bm1880")
  add_definitions(-DVPP_BM1880)
endif()
