FFMPEG_LIBS=-L../../../install/soc_bm1682_asic/ffmpeg/usr/local/lib/ -lavdevice                        \
            -lavformat -lavfilter -lavcodec -lswresample -lswscale -lavutil                          \
			-L../../../prebuilt/lib                                                      \
            -L../../../install/soc_bm1682_asic/decode/lib/ -lbmvideo -lbmjpulite -lbmjpuapi           \
            -L../../../install/soc_bm1682_asic/opencv/lib/ -lopencv_imgproc -lopencv_core -lopencv_highgui -lopencv_imgcodecs -lopencv_videoio -lyuv

CFLAGS += -Wall -g

LDLIBS := $(FFMPEG_LIBS) -lm -lpthread


.phony: all clean

all:
	aarch64-linux-gnu-g++ -o ffmpeg-jpeg_g ffmpeg-jpeg.cpp $(LDLIBS) -I../../../install/soc_bm1682_asic/ffmpeg/usr/local/include -I../../../install/soc_bm1682_asic/opencv/include
	aarch64-linux-gnu-strip -o ffmpeg-jpeg ffmpeg-jpeg_g
clean:
	$(RM) ffmpeg-jpeg_g ffmpeg-jpeg
