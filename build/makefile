MJ_PATH = /home/theia/.mujoco/mujoco200_linux
MUJOCO = -I$(MJ_PATH)/include -L$(MJ_PATH)/bin
LDFLAGS = -lmujoco200 -lGL -lglew -lpthread
LIBGLFW = $(MJ_PATH)/bin/libglfw.so.3
GLOVE_PATH = /home/theia/Libraries/puppet/cyberglove
CGLOVE = -I$(GLOVE_PATH)/source $(GLOVE_PATH)/source/CyberGlove_utils.cpp
MJVIVE = -I../vive/sdk/openVR -I../vive/sdk -DGLEW_STATIC ../vive/sdk/GL/glew.c 
LIBVR = ../vive/sdk/openVR/libopenvr_api.so

CC = g++
CFLAGS = -Wall -std=c++11
all:	
	@echo Building ==============================
	$(CC) $(CFLAGS) $(MUJOCO) ../vive/source/playlog.cpp $(LDFLAGS) $(LIBGLFW) -o ../build/playlog
	$(CC) $(CFLAGS) $(MUJOCO) $(MJVIVE) ../vive/source/viveGlove.cpp $(LDFLAGS) $(LIBGLFW) $(LIBVR) -o ../build/puppet

clean:
	@echo Cleaning ==============================
	rm ../build/playlog*
	rm ../build/puppet*
