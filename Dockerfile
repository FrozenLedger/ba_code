FROM osrf/ros:noetic-desktop-full

RUN apt update && apt upgrade -y

RUN apt install git -y

RUN mkdir -p /home/catkin_ws/src

RUN cd /home/catkin_ws && /bin/bash -c "source /opt/ros/noetic/setup.bash;catkin_make"

RUN cd /home/catkin_ws/src \
&& git clone "https://github.com/ros-simulation/gazebo_ros_pkgs.git" \
&& git clone "https://github.com/UbiquityRobotics/magni_robot.git" \
&& git clone "https://github.com/UbiquityRobotics/oled_display_node.git"

# Gazebo Installation
RUN curl -sSL http://get.gazebosim.org | sh
RUN apt-get install ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control

RUN cd /home/catkin_ws && /bin/bash -c "source /opt/ros/noetic/setup.bash;catkin_make"
RUN cd /home/catkin_ws && /bin/bash -c "rosdep install --from-paths src --ignore-src --rosdistro=$ROS_DISTRO -y"
RUN cd /home/catkin_ws && /bin/bash -c "source /opt/ros/noetic/setup.bash;catkin_make"

RUN apt update && apt install ros-noetic-navigation ros-noetic-slam-toolbox -y
RUN echo "source /opt/ros/noetic/setup.bash\nsource /home/catkin_ws/devel/setup.bash" >> ~/.bashrc