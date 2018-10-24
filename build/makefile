MJ_PATH=$(MUJOCOPATH)/mjpro150
GLOVE_PATH=../cyberglove

COMMON=/O2 /MT /EHsc
MUJOCO=/I$(MJ_PATH)/include $(MJ_PATH)/bin/mujoco150.lib $(MJ_PATH)/bin/glfw3.lib opengl32.lib
MJVIVE=/I../vive/sdk/openVR /I../vive/sdk -DGLEW_STATIC ../vive/sdk/GL/glew.c ../vive/sdk/openVR/openvr_api.lib
CGLOVE=/I$(GLOVE_PATH)/source $(GLOVE_PATH)/source/CyberGlove.cpp $(GLOVE_PATH)/source/CyberGlove_utils.cpp
CGLOVE_CAL=/I../cyberglove_calibration \
		   /I../cyberglove_calibration/external_libs/eigen \
		   /I../cyberglove_calibration/external_libs/cxxopts \
		   /I../cyberglove/utils \
		   ../cyberglove_calibration/source/viz.cpp

all:
	@echo  Building ==============================
	cl $(COMMON) ../cyberglove_calibration/source/cyberglove_calibration.cpp $(CGLOVE_CAL) $(CGLOVE) $(MUJOCO) /Fe../build/cyberglove_calibration
	cl $(COMMON) ../vive/source/playlog.cpp $(MUJOCO) $(MJVIVE) /Fe../build/playlog
	cl $(COMMON) ../vive/source/viveGlove.cpp $(MUJOCO) $(MJVIVE) $(CGLOVE) /Fe../build/puppet
	@echo  Installing ==============================
	copy "$(MJ_PATH)\bin\mujoco150.dll" "..\build\mujoco150.dll"
	copy "$(MJ_PATH)\bin\glfw3.dll" "..\build\glfw3.dll"
	copy "..\vive\sdk\openVR\openvr_api.dll" "..\build\openvr_api.dll"
	del *.obj

clean:	
	@echo  Cleaning ==============================
	del ..\biuld\cyberglove_calibration*
	del ..\build\puppet*
	del ..\build\playlog*
	del ..\build\mujoco*
	del ..\build\glfw3.dll
	del ..\build\openvr_api.dll