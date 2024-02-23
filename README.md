# ba_code

## RealSense installation:
https://github.com/IntelRealSense/realsense-ros/tree/ros1-legacy

rosrun tf static_transform_publisher 0.12 0 -0.07 1.5708 0 1.5708 base_link world 10

# Jetson
cd /Documents/github/catkin_ba/
source devel/setup.bash
roslaunch ldlidar ld19.launch
roslaunch ba_code jetson.launch

# Raspi
roslaunch ba_code joint_trajectory_controller.launch