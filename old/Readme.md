# The Riptide Hardware Package
This package handles the hardware-software interface with all of our AUVs sensors. Our AUV is equipped with:
* 1x [LordMicrostrain IMU 3DM-GX4](https://www.microstrain.com/inertial/3dm-gx4-25)
* 1x [BlueRobotics Bar30](https://www.bluerobotics.com/store/sensors-sonars-cameras/sensors/bar30-sensor-r1/) depth sensor
* 2x [Point Grey Blackfly USB 3.0](https://www.ptgrey.com/blackfly-13-mp-color-usb3-vision-sony-icx445) cameras (one forward-facing, one downward-facing)
* 4x [Aquarian Audio H2C Hydrophones](http://www.aquarianaudio.com/h2c-hydrophone.html), creating a single acoustics system

# Config Files (cfg/)
This folder contains .yaml files with parameters needed by some of our ROS nodes. The main files are calibration parameters to undistort the images from our cameras, as well as geospatial parameters for our IMU.

# Source Code (src/)
## Active ROS Nodes
### depth_processor
This node reads in raw data from the depth sensor and applies an infinite-impulse response (IIR) low pass filter (LPF) to smooth the data.
* Input Topics: /depth/raw
* Output Topics: /state/depth
### undistort_camera
This node reads in the raw image from a camera and uses the loaded camera parameters and distortion coefficients to undistort the image.
* Input Topics: /<camera_name>/image_raw
* Output Topics: /<camera_name>/image_undistorted
### acoustics
This node uses a triangulation algorithm to calculate the direction and elevation of the vehicle relative to the source of the ping.
* Input Topics: /test/acoustics, /command/acoustics, /status/controls/angular
* Output Topics: /state/acoustics, /command/attitude, /controls/reset
## Utility ROS Nodes
### calibrate_camera
This node will determine the intrinsic camera parameters as well as the distortion coefficients for the camera-lense combination. These parameters are then (manually) written to a file and are loaded automatically when launching the each camera.
* Input Topics: /<camera_name>/image_raw

# Python Scripts (scripts/)
## Active ROS Nodes
### coprocessor_serial
* Input Topics: /command/pwm
* Output Topics: /depth/raw, /state/switches
### pneumatics.py
This node listens to the high-level pneumatics commands from the state machine and sends the appropriate signals to the Arduino (via serial) in the pneumatics system to perform the desired action.
* Input Topics: /command/pneumatics
## Utilities
### riptide_rules
This file contains necessary udev rules needed to interface your computer with our sensors. Most (if not all) of our sensors are referenced using symlinks (symbolic links) to avoid headaches when sensor ports constantly change.
### add_rule
Running this script as "sudo" will add your user to the "uwrt" group containing all of the riptide_rules. This script is automatically ran if you installed ROS via "setup_uwrt_env.sh"
