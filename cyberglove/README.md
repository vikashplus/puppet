# Cyber Glove Driver 
Cyber glove is 'officially' supported only on windows. The current driver is developed for windows with a planned (probable) upgrade to Unix.
	
## Getting started 
1. Connect cyber glove. Wait for 5 seconds for the green boot up blinks.
2. Every user needs to go through a 10 mins calibration process to get their personalized calibration. Talk to Vikash if you need to get your calibration done. (It's done offline and is not part of the driver. Driver won't work without it)
3. Carefully read cyberGlove_teleOp.config to understand various configuration modes.

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
