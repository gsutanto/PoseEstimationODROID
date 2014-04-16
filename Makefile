CFLAGS_OPENCV = \
		-I/usr/include \
		-I/usr/include/opencv \

LDFLAGS_OPENCV = -lopencv_highgui -lopencv_core -lopencv_legacy -lopencv_video -lopencv_features2d -lopencv_calib3d -lopencv_imgproc

BUILD_TYPE=debug
#BUILD_TYPE=release

CFLAGS_COMMON = -Wno-multichar -g $(CFLAGS_OPENCV) -MD

ifeq ($(BUILD_TYPE), debug)
	CFLAGS = $(CFLAGS_COMMON)
endif
ifeq ($(BUILD_TYPE), release)
	CFLAGS = $(CFLAGS_COMMON) -O2
endif

LDFLAGS = $(LDFLAGS_OPENCV) -lX11 -lXext -lrt -lstdc++ -lm

TARGETS = libpose_estimation.a main

main.o: main.cpp PoseEstimation.h
	gcc -c -o main.o main.cpp $(CFLAGS)

camdrv.o: CameraDriver.cpp PoseEstimation.h
	gcc -c -o camdrv.o CameraDriver.cpp $(CFLAGS)

pe.o: PoseEstimation.cpp PoseEstimation.h
	gcc -c -o pe.o PoseEstimation.cpp $(CFLAGS)

libpose_estimation.a: pe.o camdrv.o
	ar rvs libpose_estimation.a pe.o camdrv.o

main: main.o camdrv.o pe.o
	gcc -o main main.o camdrv.o pe.o $(LDFLAGS) $(CFLAGS)

all: $(TARGETS)

clean:
	rm -rf *o *d libpose_estimation.a main
