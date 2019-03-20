# MJ_PATH = /home/theia/.mujoco/mjpro150
# GLOVE_PATH = /home/theia/Libraries/puppet/cyberglove
# MUJOCO = -I$(MJ_PATH)/include -L$(MJ_PATH)/bin $(MJ_PATH)/bin/libglfw.so.3 
# LDFLAGS = -lGL 
# MJVIVE = -I../vive/sdk/openVR -I../vive/sdk -DGLEW_STATIC -I../vive/sdk/GL/glew.c -I../vive/sdk/openVR/openvr_capi.h 
# CGLOVE = -I$(GLOVE_PATH)/source -I$(GLOVE_PATH)/source/CyberGlove.cpp -I$(GLOVE_PATH)/source/CyberGlove_utils.cpp
# CC = g++
# CFLAGS  = -g -Wall

# all:	
# 	@echo Building ==============================
# 	$(CC) $(CFLAGS) ../vive/source/playlog.cpp $(MUJOCO) $(MJVIVE) $(LDFLAGS) -o ../build/playlog 
# 	$(CC) $(CFLAGS) ../vive/source/viveGlove.cpp $(MUJOCO) $(MJVIVE) $(GLOVE_PATH) $(LDFLAGS) -o ../build/puppet  
# 	@echo Installing ==============================


MJ_PATH = /home/theia/.mujoco/mjpro150
MUJOCO = -I$(MJ_PATH)/include -L$(MJ_PATH)/bin
LDFLAGS = -lmujoco150 -lGL -lglew
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
