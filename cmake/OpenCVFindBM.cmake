
if(CMAKE_SYSTEM_NAME MATCHES "Windows")
    set(CMAKE_IMPORT_LIBRARY_PREFIX lib)
    if (NOT DEFINED FFMPEG_INCLUDE_DIRS)
        set(FFMPEG_INCLUDE_DIRS
            "${OpenCV_SOURCE_DIR}/3rdparty/libbmcv/include"
            "${OpenCV_SOURCE_DIR}/../ffmpeg_install/include"
            "${OpenCV_SOURCE_DIR}/../bmvid/jpeg/driver/bmjpuapi/inc"
            "${OpenCV_SOURCE_DIR}/../bmvid/3rdparty/libyuv/include"
            "${OpenCV_SOURCE_DIR}/../bmvid/vpp/driver/include/bm1684"
            "${OpenCV_SOURCE_DIR}/../bmvid/bmcv/include"
        )
    else()
        string(REPLACE " " ";" FFMPEG_INCLUDE_DIRS ${FFMPEG_INCLUDE_DIRS})
    endif()
    if (NOT DEFINED FFMPEG_LIBRARY_DIRS)
        set(FFMPEG_LIBRARY_DIRS
            "${OpenCV_SOURCE_DIR}/3rdparty/libbmcv/lib/${PRODUCTFORM}"
            "${OpenCV_SOURCE_DIR}/../ffmpeg_install/lib"
            "${OpenCV_SOURCE_DIR}/../ffmpeg_install/bin"
            "${OpenCV_SOURCE_DIR}/../bmvid/release/lib"
            "${OpenCV_SOURCE_DIR}/../prebuilt/windows/lib"
        )
    else()
        string(REPLACE " " ";" FFMPEG_LIBRARY_DIRS ${FFMPEG_LIBRARY_DIRS})
    endif()
    LINK_DIRECTORIES(${FFMPEG_LIBRARY_DIRS})
else() #"Linux"
    if (NOT DEFINED FFMPEG_INCLUDE_DIRS)
        set(FFMPEG_INCLUDE_DIRS
            "${OpenCV_SOURCE_DIR}/3rdparty/libbmcv/include"
            "${CMAKE_INSTALL_PREFIX}/../ffmpeg/usr/local/include"
            "${CMAKE_INSTALL_PREFIX}/../decode/include"
            "${CMAKE_INSTALL_PREFIX}/../vpp/include"
            "${CMAKE_INSTALL_PREFIX}/../bmcv/include"
        )
    else()
        string(REPLACE " " ";" FFMPEG_INCLUDE_DIRS ${FFMPEG_INCLUDE_DIRS})
    endif()
    if (NOT DEFINED FFMPEG_LIBRARY_DIRS)
        set(FFMPEG_LIBRARY_DIRS
            "${OpenCV_SOURCE_DIR}/3rdparty/libbmcv/lib/${PRODUCTFORM}"
            "${CMAKE_INSTALL_PREFIX}/../ffmpeg/usr/local/lib"
            "${CMAKE_INSTALL_PREFIX}/../decode/lib"
            "${CMAKE_INSTALL_PREFIX}/../bmcv/lib"
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
  ${CMAKE_IMPORT_LIBRARY_PREFIX}vpp_cmodel
  ${CMAKE_IMPORT_LIBRARY_PREFIX}yuv)

  list(APPEND FFMPEG_LIBRARIES
  ${CMAKE_IMPORT_LIBRARY_PREFIX}bmion
  #${CMAKE_IMPORT_LIBRARY_PREFIX}bmvppapi
  )
  if((NOT CMAKE_SYSTEM_NAME MATCHES "Windows")
    AND (${PRODUCTFORM} STREQUAL "soc" OR ${PRODUCTFORM} STREQUAL "pcie"))
    list(APPEND FFMPEG_LIBRARIES
    ${CMAKE_IMPORT_LIBRARY_PREFIX}openblas)
  endif()
endif()

if(${PRODUCTFORM} STREQUAL "soc")
  #set(USING_SOC ON)
  #set(HAVE_LIBYUV ON)
  add_definitions(-DUSING_SOC)
  add_definitions(-DHAVE_LIBYUV)
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
