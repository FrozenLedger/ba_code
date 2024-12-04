# ba_code

## RealSense installation:
https://github.com/IntelRealSense/realsense-ros/tree/ros1-legacy

## [Deprecated] Transformation fix: base -> robotarm
rosrun tf static_transform_publisher 0.12 0 -0.07 1.5708 0 1.5708 base_link world 10

# Jetson
* loads urdf description for camera joints and frames.
* publishes camera_robot_state via robot_state_publisher node

* usb connection -> camera
* usb -> lidar (usually: /dev/ttyUSB0)

## Docker Jetson-Containers
git clone https://github.com/dusty-nv/jetson-containers
bash jetson-containers/install.sh


## Required Packages:
sudo apt-get install ros-noetic-realsense2-camera
sudo apt-get install ros-noetic-realsense2-description
sudo apt-get install ros-noetic-amcl
sudo apt-get install ros-noetic-move-base
sudo apt-get install ros-noetic-slam-toolbox

## Required specific Pytorch installation for Jetson Devices:
Follow instructions at:
    https://docs.nvidia.com/deeplearning/frameworks/install-pytorch-jetson-platform/index.html

## Required Python Packages
pip3 install numpy-quaternion
pip install -U yolov5 # source: https://huggingface.co/turhancan97/yolov5-detect-trash-classification

## CUDA installation for Jetson NX Xavier:
https://www.seeedstudio.com/blog/2020/07/29/install-cuda-11-on-jetson-nano-and-xavier-nx/?srsltid=AfmBOoqUfbfqZSXzyQKox-F6KxN1O3cNPoEd-Hyv_gwkOmrrB5d_6Wcq

## Jetson startup commands
cd /Documents/github/catkin_ba/
source devel/setup.bash
roslaunch ba_code jetson.launch

# RPi4
* loads urdf descriptions for wheel and lidar joints and frames.
* starts the motor node
* publishes /odom data
* runs the master node

* controls the robotarm (usually: /dev/ttyUSB0)
* communication flow: USB -> TTL -> U2D2 -> robotarm

## RPi4 startup commands
roslaunch ba_code raspi.launch

### The problem with multiple robot descriptions in ROS1
https://answers.ros.org/question/318086/using-two-robots-two-robot_description-in-ros/

## Fixes
### The jetson launch file was edited:
static transform from base_link to world
static transform from base_footprint to base_link

### clock synchronization:
use the /clock topic: http://wiki.ros.org/Clock
--- or ---
synchronize using chrony: http://wiki.ros.org/ROS/NetworkSetup

# TODOs
[ ] Stable WLAN/Wifi Setup.
[ ] Containerization using Docker or Podman
[ ] Improvement of the Trash-Detection/Object-Detection
[ ] Improvement of the Statemachine -> needs higher abstraction