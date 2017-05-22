# Cyber Glove Driver 
Cyber glove is 'officially' supported only on windows. The current driver is developed for windows with a planned (probable) upgrade to Unix.
	
## Getting started 
1. Power up cyber glove. Connect cyber glove to the mini USB cable hanging out the HTC headset. Wait for 5 seconds for the green boot up blinks.
2. Every user needs to go through a 10 mins calibration process to get their personalized cyberglove calibration. Talk to Vikash if you need to get your calibration done. (It's done offline and is not part of the driver. Driver won't work without it)

## Build 
1. Open VS2015 x64 Native Tools Command Prompt
2. Navigate to cyberglove/build/ folder
3. Make sure environment variable "MUJOCOPATH" exists/ [add environment variable "MUJOCOPATH"](https://github.com/openai/mujoco-bin#windows), and is correct, using the command "SET MUJOCOPATH".
4. Use "nmake" to build and install the project. It will compile and install viveGlove.exe in teleop/vive/bin folder
5. Use "nmake clean" to clean the project installation. Note it doesn't clear recorded logs.

## Usage
Navigate to vive/bin/ folder. viveGlove.exe is used for emersive visualization and interaction with the mujoco worlds using vive controller, vive tracker and cyberglove. Type viveGlove.exe without any arguments for usage instructions. Recoreded logs can be played-back/ video-recorded using playlog.exe. [Please refer here for more instructions](https://github.com/vikashplus/teleOp/tree/master/vive#usage). 

**Note1**: Carefully read cyberGlove_teleOp.config to understand various configuration modes.

**Note2**: To know the COM port for your cyberglove -- open `Device Manager>Ports(COM & LPT)`. Locate the COM Port connected to the cyber glove (Most likely labeled as `USB Serial Port`) 

## Visualization options
1. HTC Vive (Virtual Reality immersive visualization): Needs vive headset, one active controller, and cyber glove
2. mjHaptix (Stereoscopic visualization): Needs active mjHaptix for rendering. Load d3xter_hand.xml model (form the d3xter folder) in mjHaptix. Turn bool STREAM_2_VIZ = true in the cyberGlove_teleOp.config to use visualization

## Plotting
CyberGlove raw values are rendered using Vikash's plotting library available in plot folder

## Road Map
1. It might be possible to get the driver working on Unix. Glove follows normal serial protocol. If we can establish port communication, rest of the driver should be portable. 
2. Move raw cyber glove plotting from plot to pangolin for uniformity.
3. ~~Integrate mjVive for visualization and remove mjHaptix dependencies.~~
4. Integration with ShadowHand is planned and is on its way.
5. Integration with Kuka is on the long term map once the hardware is here.
6. Calibration process presently is a project in Matlab. If you are interested in working on porting that into a C-code (so that we can make it accessible to everyone), please talk to Vikash.
