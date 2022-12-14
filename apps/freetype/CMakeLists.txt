set(OPENCV_APPLICATION_DEPS opencv_core opencv_imgproc opencv_imgcodecs opencv_freetype)
ocv_check_dependencies(${OPENCV_APPLICATION_DEPS})

if(NOT OCV_DEPENDENCIES_FOUND)
  message("ERROR: OCV_DEPENDENCIES_FOUND")
  return()
endif()

project(freetype_demo)
set(the_target freetype_demo)

ocv_target_include_directories(${the_target} PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}" "${OpenCV_SOURCE_DIR}/include/opencv")
ocv_target_include_modules_recurse(${the_target} ${OPENCV_APPLICATION_DEPS})

file(GLOB SRCS *.cpp)

ocv_add_executable(${the_target} ${SRCS})
ocv_target_link_libraries(${the_target} ${OPENCV_APPLICATION_DEPS})

set_target_properties(${the_target} PROPERTIES
                      DEBUG_POSTFIX "${OPENCV_DEBUG_POSTFIX}"
                      RUNTIME_OUTPUT_DIRECTORY ${EXECUTABLE_OUTPUT_PATH}
                      OUTPUT_NAME "freetype_demo")

set_target_properties(${the_target} PROPERTIES FOLDER "applications")

if(INSTALL_CREATE_DISTRIB)
  if(BUILD_SHARED_LIBS)
    install(TARGETS ${the_target} RUNTIME DESTINATION ${OPENCV_BIN_INSTALL_PATH} CONFIGURATIONS Release COMPONENT libs)
  endif()
else()
  install(TARGETS ${the_target} RUNTIME DESTINATION ${OPENCV_BIN_INSTALL_PATH} COMPONENT libs)
endif()
