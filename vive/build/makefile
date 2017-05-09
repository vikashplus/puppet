MJ_PATH=$(MUJOCOPATH)/mjpro140

COMMON=/O2 /MT /EHsc /I$(MJ_PATH)/include $(MJ_PATH)/bin/mujoco140.lib $(MJ_PATH)/bin/glfw3.lib opengl32.lib /Fe../bin/

all:
	cl $(COMMON) /I../include/openVR /I../include -DGLEW_STATIC ../source/mjViveFetch.cpp ../source/glew.c ../bin/openvr_api.lib
	copy "$(MJ_PATH)\bin\mujoco140.dll" "..\bin\mujoco140.dll"
	copy "$(MJ_PATH)\bin\glfw3.dll" "..\bin\glfw3.dll"
	del *.obj

clean:
	del ..\bin\mjViveFetch.exe
	del ..\bin\mujoco*
	del ..\bin\glfw3.dll