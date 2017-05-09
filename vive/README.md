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

### Controller/Mocap configuration
Am xml with a mocap body called "mocap0/mocap1" gets associated with controller0/controller1 and places the controller in the "vTOOL_PULL" mode.

### Fetch
While the vive driver is general purpose, There are fetch-gripper specific utilities that are provided if the xml loaded has two actuators with following names: "r_gripper_finger_joint" and "l_gripper_finger_joint"

## Trouble shooting 
1. Vive tracker not detected - you need to add Vive Tracker into SteamVR. Rightclick on one of the existing controller’s icon and click “Pair Controller” in the pop-up menu. Press the Power button on Vive Tracker for 2 seconds, and then release it to enter the paring mode. Note that if you have two Vive controllers already, then you will need to plug the dongle into the dongle cradle to PC’s USB port. More details can be found here [Vive Tracker developer guide](https://dl.vive.com/Tracker/Guideline/HTC_Vive_Tracker_Developer_Guidelines_v1.3.pdf)
