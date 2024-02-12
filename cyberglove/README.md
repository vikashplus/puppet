# Cyber Glove Driver 
CyberGlove 'official' supported is limited to windows. The current CyberGlove driver for PUPPET is developed for windows. (This driver is platform agnostic. There are evidence that CyberGlove API works on unix. In principle, PUPPET's CyberGlove support can be extened to unix. This is in the wishlist)
	
## Getting started 
1. Power up cyber glove. Connect cyber glove to the mini USB cable hanging out the HTC headset. Wait for 5 seconds for the green boot up blinks.
2. This repo contains a default calibration that works quite well for most individuals. (brain adapts fast to the provided calibration). Its possible to have your custom calibration. It takes about 10 mins to get a personalized cyberglove calibration. (WIP: there is an unmerged branch on the repo. Get in touch with Vikash if you want to attempt this.)

## Usage
Navigate to `build/` folder. `puppet.exe <config_file>` is used for emersive visualization and interaction with the mujoco worlds using vive controller, vive tracker and cyberglove. Recoreded logs can be played-back/ video-recorded using playlog.exe. [Please refer here for more instructions](https://github.com/vikashplus/teleOp/tree/master/build#usage). 

**Note1**: Carefully read `cyberGlove_teleOp.config` to understand various configuration modes.

**Note2**: To know the COM port for your cyberglove -- open `Device Manager>Ports(COM & LPT)`. Locate the COM Port connected to the cyber glove (Most likely labeled as `USB Serial Port`) 

## Visualization options
1. HTC Vive (Virtual Reality immersive visualization): Needs vive headset, one active controller, and cyber glove
2. (depricated) mjHaptix (Stereoscopic visualization): Needs active mjHaptix for rendering. Load `humanoid.xml` in mjHaptix. Turn `bool STREAM_2_VIZ = true` in the `cyberGlove_teleOp.config` to use visualization


## Road Map
1. It might be possible to get the driver working on Unix. Glove follows normal serial protocol. If we can establish port communication, rest of the driver should be portable. 
2. Calibration process presently is a project in Matlab. If you are interested in working on porting that into a C-code (so that we can make it accessible to everyone), please talk to Vikash.
