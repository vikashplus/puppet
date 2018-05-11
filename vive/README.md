# Vive C driver

## Build 
1. Open VS2015 x64 Native Tools Command Prompt
2. Navigate to vive/build/ folder
3. Make sure environment variable "MUJOCOPATH" exists/ [add environment variable "MUJOCOPATH"](https://github.com/openai/mujoco-bin#windows), and is correct, using the command "SET MUJOCOPATH".
4. Use "nmake" to build and install the project. It will compile two programs mjVive.exe and playlog.exe
5. Use "nmake clean" to clean the project installation. Note it doesn't clear recorded logs.

## Usage
Navigate to vive/bin/ folder. Type mjvive.exe/ playlog.exe without any arguments for usage instructions. 
1. mjVive.exe is used for emersive visualization and interaction with the mujoco worlds.
2. playlog.exe is can be used to replay recorded logs and dump raw video (Key F9) (pixel_format rgb24).


**Note1**: Logs are dumped in mujoco's .mjl format. Refer [Mujoco documenation](http://www.mujoco.org/book/haptix.html#uiRecord) for details.  

**Note2**: You can use [ffmpeg](https://ffmpeg.org/) to convert the raw video. Ensure that the video resolution and fps matches with the settings used while dumping raw video.
```
ffmpeg -f rawvideo -pixel_format rgb24 -video_size 800x800 -framerate 60 -i rgb.out -vf "vflip" video.mp4
```

## Requirements
Vive headset and a minimum of one active controller.

## Controller bindings
<p align="center"><img src="controller_bindings.jpg" alt="Controller bindings" height="500"/></p>

## Special cases 

### Controller/Mocap configuration
Am xml with a mocap body called "mocap0/mocap1" gets associated with controller0/controller1 and places the controller in the "vTOOL_PULL" mode.

### Fetch
While the vive driver is general purpose, There are fetch-gripper specific utilities that are provided if the xml loaded has two actuators with following names: "r_gripper_finger_joint" and "l_gripper_finger_joint"

## Trouble shooting 
1. Vive tracker not detected? - you need to add Vive Tracker into SteamVR. Rightclick on one of the existing controller’s icon and click “Pair Controller” in the pop-up menu. Press the Power button on Vive Tracker for 2 seconds, and then release it to enter the paring mode. Note that if you have two Vive controllers already, then you will need to plug the dongle into the dongle cradle to PC’s USB port. More details can be found here [Vive Tracker developer guide](https://dl.vive.com/Tracker/Guideline/HTC_Vive_Tracker_Developer_Guidelines_v1.3.pdf)
