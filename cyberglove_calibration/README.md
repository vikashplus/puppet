# CyberGlove III Calibration

This tool generates puppet calibration files for the cyberglove.

## Usage 
To run the cyberglove calibration tool, you must have the following dependencies.


* Adroit MuJoCo Model - `files/adroit/Adroit_hand.xml`
* Adroit Calibration Poses - `files/Adroitcalib_actuatorPoses.csv`

```
C:\>cyberglove_calibration.exe --help
 - calibration Utility for CyberGlove III
Usage:
  cyberglove_calibration.exe [OPTION...]

      --help           Print help
      --mj_path arg    MUJOCO_PATH (default: c:\mujoco)
      --pose_file arg  Pose file (CSV) (default:
                       Adroitcalib_actuatorPoses.csv)
  -p, --port arg       Serial port (eg. "COM3") (default: COM3)
      --prefix arg     Output calibration file prefixes (default: user)
  -v, --viz_only       Visualize a calibration
  -x, --xml arg        Adroit MuJoCo XML model (default:
                       adroit\Adroit_hand.xml)
```

As soon as the script is launched, you will be prompted to move your hand around for 10 seconds. Data captured during this period will be used to normalize the raw signals from the glove, so be sure to capture some extreme joint movements.

After the initial normalization procedure, you will then be instructred to mimic a number of poses, which will be displayed in MuJoCo. The data vectors captured from these poses will then be used to generate the following calibration files:

* `user.calib` - Calibration matrix, mapping from raw sensor space to MuJoCo joint space.
* `user.handRange` - Joint ranges associated with the Adroit hand, used for normalization.
* `user.userRange` - Raw glove sensor ranges, used for normalization


## Compiling cyberglove_calibration
Coming soon!

## Calibration Math
Coming soon!
