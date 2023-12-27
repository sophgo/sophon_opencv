if(WITH_QT)
    ocv_add_application(vidbasic MODULES opencv_videoio opencv_imgcodecs opencv_highgui Qt5Core Qt5Gui Qt5Widgets Qt5Test Qt5Concurrent SRCS opencv_vidbasic.cpp)
else()
    ocv_add_application(vidbasic MODULES opencv_videoio opencv_imgcodecs opencv_highgui SRCS opencv_vidbasic.cpp)
endif()

