xhost local:root
XAUTH=/tmp/.docker.xauth
DIR=$(pwd)
echo $DIR
docker run -it --rm \
    --name=me5413_final_project_ros_container \
    --gpus all \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_NITSHM=1" \
    --volume="/tmp/.X11-unix/:/tmp/.X11-unix:rw" \
    --volume="$DIR:/home/ros/ME5413_Final_Project" \
    --env="XAUTHORITY=$XAUTH"\
    --volume="$XAUTH:$XAUTH" \
    --net=host \
    --privileged \
    me5413_final_project_ros

# Install https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html#installing-with-yum-or-dnf