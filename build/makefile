MJ_PATH=$(MUJOCOPATH)/mjpro150
GLOVE_PATH=../cyberglove

COMMON=/O2 /MT /EHsc /Fe../build/
MUJOCO=/I$(MJ_PATH)/include $(MJ_PATH)/bin/mujoco150.lib $(MJ_PATH)/bin/glfw3.lib opengl32.lib
MJVIVE=/I../vive/sdk/openVR /I../vive/sdk -DGLEW_STATIC ../vive/sdk/GL/glew.c ../vive/sdk/openVR/openvr_api.lib
CGLOVE=/I$(GLOVE_PATH)/source $(GLOVE_PATH)/source/CyberGlove.cpp $(GLOVE_PATH)/source/CyberGlove_utils.cpp

all:
	@echo  Building ==============================
	cl $(COMMON) ../vive/source/mjVive.cpp $(MUJOCO) $(MJVIVE)
	cl $(COMMON) ../vive/source/playlog.cpp $(MUJOCO) $(MJVIVE)
	cl $(COMMON) ../vive/source/viveGlove.cpp $(MUJOCO) $(MJVIVE) $(CGLOVE)
	@echo  Installing ==============================
	copy "$(MJ_PATH)\bin\mujoco150.dll" "..\build\mujoco150.dll"
	copy "$(MJ_PATH)\bin\glfw3.dll" "..\build\glfw3.dll"
	copy "..\vive\sdk\openVR\openvr_api.dll" "..\build\openvr_api.dll"
	del *.obj

clean:	
	@echo  Cleaning ==============================
	del ..\build\mjVive*
	del ..\build\viveGlove*
	del ..\build\playlog*
	del ..\build\mujoco*
	del ..\build\glfw3.dll
	del ..\build\openvr_api.dll