# Vive C driver

## Build 
1. Open VS2015 x64 Native Tools Command Prompt
2. Make sure environment variable "MUJOCOPATH" exists, and is correct, using the command "SET MUJOCOPATH"
3. Use "nmake" to build the project and "nmake clean" to clean the project

## usage
Use "mjvive.exe <model_xml_withPath> <log_filename>" to run the compiled executable. <log_filename> is optional for dumping logs. Logs are dumped in mujoco's .mjl format. Refer [Mujoco documenation](http://www.mujoco.org/book/haptix.html#uiRecord) for details.  

## Requirement
Vive headset and a minimum of one active controller

## Controller bindings
<p align="center"><img src="https://github.com/openai/raas/blob/cyberGlove/vive/controller_bindings.jpg" alt="Controller bindings" height="500"/></p>

## Special cases 

### Controller configuration
Am xml with a mocap body called "mocap0/mocap1" get gets associated with controller0/controller1 and places the controller0 in the "vTOOL_PULL" mode.

### Fetch
While the vive driver is general purpose, There are fetch-gripper specific utilities that are provided if the xml loaded has two actuators with following names: "r_gripper_finger_joint" and "l_gripper_finger_joint"
