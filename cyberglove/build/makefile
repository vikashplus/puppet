MJ_PATH=$(MUJOCOPATH)/mjpro140
VIVE_PATH=..\..\vive
PLOT_PATH=../../../plot

COMMON=/O2 /MT /EHsc /Fe$(VIVE_PATH)/bin/
mujoco=/I$(MJ_PATH)/include $(MJ_PATH)/bin/mujoco140.lib $(MJ_PATH)/bin/glfw3.lib opengl32.lib
mjVive=/I$(VIVE_PATH)/sdk/openVR /I$(VIVE_PATH)/sdk -DGLEW_STATIC $(VIVE_PATH)/sdk/GL/glew.c $(VIVE_PATH)/bin/openvr_api.lib
cGlove=/I../source ../source/CyberGlove.cpp ../source/CyberGlove_utils.cpp
matplot=/I$(PLOT_PATH)/include $(PLOT_PATH)/x64/Release/MatPlot_static.lib $(PLOT_PATH)/x64/Release/freeglut_static.lib ../source/Graphics.cpp

all:
	cl ../source/viveGlove_main.cpp $(COMMON) $(mujoco) $(mjVive) $(cGlove) $(matplot)
	copy "$(MJ_PATH)\bin\mujoco140.dll" "$(VIVE_PATH)\bin\mujoco140.dll"
	copy "$(MJ_PATH)\bin\glfw3.dll" "$(VIVE_PATH)\bin\glfw3.dll"
	del *.obj
	
clean:
	del $(VIVE_PATH)\bin\viveGlove_main.exe
	del $(VIVE_PATH)\bin\mujoco*
	del $(VIVE_PATH)\bin\glfw3.dll