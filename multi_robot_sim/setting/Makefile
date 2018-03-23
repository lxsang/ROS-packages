CC = g++
CFLAGS = -g -Wall
SRCS = skel.cpp
PROG = filter

OPENCV = -I/opt/ros/kinetic/include/opencv-3.2.0-dev -L/opt/ros/kinetic/lib  \
			-lopencv_cvv3 -lopencv_highgui3 \
			-lopencv_core3 -lopencv_imgproc3 -lopencv_features2d3\
			-lopencv_objdetect3 -lopencv_imgcodecs3
LIBS = $(OPENCV)

$(PROG):$(SRCS)
	$(CC) $(CFLAGS) -o $(PROG) $(SRCS) $(LIBS)