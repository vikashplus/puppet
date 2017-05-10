MJ_PATH=$(MUJOCOPATH)/mjpro140

COMMON=/O2 /MT /EHsc /Fe../bin/
MUJOCO=/I$(MJ_PATH)/include $(MJ_PATH)/bin/mujoco140.lib $(MJ_PATH)/bin/glfw3.lib opengl32.lib
MJVIVE=/I../sdk/openVR /I../sdk -DGLEW_STATIC ../sdk/GL/glew.c ../bin/openvr_api.lib

all:
	cl $(COMMON) ../source/mjVive.cpp $(MUJOCO) $(MJVIVE)
	copy "$(MJ_PATH)\bin\mujoco140.dll" "..\bin\mujoco140.dll"
	copy "$(MJ_PATH)\bin\glfw3.dll" "..\bin\glfw3.dll"
	del *.obj

clean:	
	del ..\bin\mjVive.exe
	del ..\bin\mujoco*
	del ..\bin\glfw3.dll