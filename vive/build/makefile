MJ_PATH=$(MUJOCOPATH)/mjpro150
GLOVE_PATH=../../cyberglove
PLOT_PATH=../../../plot

COMMON=/O2 /MT /EHsc /Fe../bin/
MUJOCO=/I$(MJ_PATH)/include $(MJ_PATH)/bin/mujoco150.lib $(MJ_PATH)/bin/glfw3.lib opengl32.lib
MJVIVE=/I../sdk/openVR /I../sdk -DGLEW_STATIC ../sdk/GL/glew.c ../sdk/openVR/openvr_api.lib
CGLOVE=/I$(GLOVE_PATH)/source $(GLOVE_PATH)/source/CyberGlove.cpp $(GLOVE_PATH)/source/CyberGlove_utils.cpp
MATPLOT=/I$(PLOT_PATH)/include $(PLOT_PATH)/x64/Release/MatPlot_static.lib $(PLOT_PATH)/x64/Release/freeglut_static.lib $(GLOVE_PATH)/source/Graphics.cpp

all:
	@echo  Building ==============================
	cl $(COMMON) ../source/mjVive.cpp $(MUJOCO) $(MJVIVE)
	cl $(COMMON) ../source/playlog.cpp $(MUJOCO) $(MJVIVE)
	cl $(COMMON) ../source/viveGlove.cpp $(MUJOCO) $(MJVIVE) $(CGLOVE) $(MATPLOT)
	@echo  Installing ==============================
	copy "$(MJ_PATH)\bin\mujoco150.dll" "..\bin\mujoco150.dll"
	copy "$(MJ_PATH)\bin\glfw3.dll" "..\bin\glfw3.dll"
	copy "..\sdk\openVR\openvr_api.dll" "..\bin\openvr_api.dll"
	del *.obj

clean:	
	@echo  Cleaning ==============================
	del ..\bin\mjVive*
	del ..\bin\viveGlove*
	del ..\bin\playlog*
	del ..\bin\mujoco*
	del ..\bin\glfw3.dll
	del ..\bin\openvr_api.dll