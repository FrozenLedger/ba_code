# ba_code

## RealSense installation:
https://github.com/IntelRealSense/realsense-ros/tree/ros1-legacy

## Transformation fix: base -> robotarm
rosrun tf static_transform_publisher 0.12 0 -0.07 1.5708 0 1.5708 base_link world 10

# Jetson
* loads urdf description for camera joints and frames.
* publishes camera_robot_state via robot_state_publisher node

* usb connection -> camera
* usb -> lidar (usually: /dev/ttyUSB0)

## Jetson startup commands
cd /Documents/github/catkin_ba/
source devel/setup.bash
roslaunch ldlidar ld19.launch
roslaunch ba_code jetson.launch

# RPi4
* loads urdf descriptions for wheel and lidar joints and frames.
* starts the motor node
* publishes /odom data
* runs the master node

* controls the robotarm (usually: /dev/ttyUSB0)
* communication flow: USB -> TTL -> U2D2 -> robotarm

## RPi4 startup commands
roslaunch ba_code joint_trajectory_controller.launch

### The problem with multiple robot descriptions in ROS1
https://answers.ros.org/question/318086/using-two-robots-two-robot_description-in-ros/

## Fixes
static transform from base_link to world
static transform from base_footprint to base_link