FROM osrf/ros:noetic-desktop-full

RUN apt update && apt upgrade -y

RUN mkdir -p /catkin_ws
RUN mkdir -p /catkin_ws/src

RUN cd /catkin_ws && /bin/bash -c "source /opt/ros/noetic/setup.bash;catkin_make"

COPY gazebo_ros_pkgs /catkin_ws/src/gazebo_ros_pkgs
COPY magni_robot /catkin_ws/src/magni_robot
COPY oled_display_node /catkin_ws/src/oled_display_node

RUN cd /catkin_ws && /bin/bash -c "source /opt/ros/noetic/setup.bash;catkin_make"
RUN cd /catkin_ws && /bin/bash -c "rosdep install --from-paths src --ignore-src --rosdistro=$ROS_DISTRO -y"

RUN cd /catkin_ws && /bin/bash -c "source /opt/ros/noetic/setup.bash;catkin_make"

RUN apt update && apt install ros-noetic-navigation ros-noetic-slam-gmapping

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics
