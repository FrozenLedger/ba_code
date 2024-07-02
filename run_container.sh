#docker run -it \
#    --name=magnisim_noetic \
#    --net=ros \
#    -v $PWD:/home/catkin_ws/src/ba_code \
#    --env="DISPLAY=$DISPLAY" \
#    --env="QT_X11_NO_MITSHM=1" \
#    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
#    --runtime=nvidia \
#    --gpus all \
#    --env="NVIDIA_DRIVER_CAPABILITIES=all" \
#    magnisim_noetic \
#    bash

docker run -it \
    --name=magnisim_noetic \
    --net=ros \
    -v $PWD:/home/catkin_ws/src/ba_code \
    --gpus all \
    --env="NVIDIA_DRIVER_CAPABILITIES=all" \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    magnisim_noetic \
    bash