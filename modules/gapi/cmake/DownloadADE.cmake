if(ANDROID)
  # FIXME: Android build will be enabled separately
  return()
endif()

if (${CMAKE_CXX_COMPILER_VERSION}  VERSION_GREATER  "13.0.0" AND ${CMAKE_HOST_SYSTEM_NAME} MATCHES "Linux" )
set(ade_src_dir "${OpenCV_BINARY_DIR}/3rdparty/ade")
set(ade_filename "v0.1.2c.zip")
set(ade_subdir "ade-0.1.2c")
set(ade_md5 "4e996f545f9dddf2348cf593cbb2726f")
ocv_download(FILENAME ${ade_filename}
             HASH ${ade_md5}
             URL
               "${OPENCV_ADE_URL}"
               "$ENV{OPENCV_ADE_URL}"
               "https://github.com/opencv/ade/archive/"
             DESTINATION_DIR ${ade_src_dir}
             ID ADE
             STATUS res
             UNPACK RELATIVE_URL)
else()
set(ade_src_dir "${OpenCV_BINARY_DIR}/3rdparty/ade")
set(ade_filename "v0.1.1d.zip")
set(ade_subdir "ade-0.1.1d")
set(ade_md5 "37479d90e3a5d47f132f512b22cbe206")
ocv_download(FILENAME ${ade_filename}
             HASH ${ade_md5}
             URL
               "${OPENCV_ADE_URL}"
               "$ENV{OPENCV_ADE_URL}"
               "https://github.com/opencv/ade/archive/"
             DESTINATION_DIR ${ade_src_dir}
             ID ADE
             STATUS res
             UNPACK RELATIVE_URL)
endif()

if (NOT res)
    return()
endif()

set(ADE_root "${ade_src_dir}/${ade_subdir}/sources/ade")
file(GLOB_RECURSE ADE_sources "${ADE_root}/source/*.cpp")
file(GLOB_RECURSE ADE_include "${ADE_root}/include/ade/*.hpp")
add_library(ade STATIC ${ADE_include} ${ADE_sources})
target_include_directories(ade PUBLIC $<BUILD_INTERFACE:${ADE_root}/include>)
set_target_properties(ade PROPERTIES POSITION_INDEPENDENT_CODE True)

if(NOT BUILD_SHARED_LIBS)
  ocv_install_target(ade EXPORT OpenCVModules ARCHIVE DESTINATION ${OPENCV_3P_LIB_INSTALL_PATH} COMPONENT dev)
endif()

ocv_install_3rdparty_licenses(ade "${ade_src_dir}/${ade_subdir}/LICENSE")
