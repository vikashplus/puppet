# Vive C driver

## usage
From terminal type "mjvive.exe <model_xml_withPath> <log_filename>" to run the compiled executable. <log_filename> is optional for dumping logs. Logs are dumped in mujoco's .mjl format. Refer [Mujoco documenation](http://www.mujoco.org/book/haptix.html#uiRecord) for details.  

## Requirement
Vive headset and a minimum of one active controller

## Controller bindings
<p align="center"><img src="https://github.com/openai/raas/blob/cyberGlove/vive/controller_bindings.jpg" alt="Controller bindings" height="500"/></p>

NOTE: While the vive driver is general purpose, There are fetch-gripper specific utilities that are provided if the xml loaded has two actuators with following names: "r_gripper_finger_joint" and "l_gripper_finger_joint"
