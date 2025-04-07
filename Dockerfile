FROM siddux/ros:noetic-cuda-gazebo-nvidia

ARG ROS_DISTRO=noetic

ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt upgrade -y\
    && apt-get install -y sudo git \
    ros-noetic-vision-msgs ros-noetic-cv-bridge ros-noetic-sensor-msgs ros-noetic-vision-msgs python3-pip python3-tk\
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

RUN apt-get update && apt-get install -y  \
    ros-noetic-tf2-ros \
    ros-noetic-tf2-geometry-msgs \
    ros-noetic-tf2-sensor-msgs \
    ros-noetic-jsk-rviz-plugins \
    ros-noetic-jackal-gazebo \
    ros-noetic-jackal-navigation \
    ros-noetic-velodyne-simulator \
    ros-noetic-teleop-twist-keyboard \
    ros-noetic-gazebo-ros-pkgs \
    ros-noetic-gazebo-ros-control \
    ros-noetic-explore-lite \
    ros-noetic-jsk-visualization

RUN pip3 install numpy easyocr
RUN pip3 install Pillow==9.3.0

RUN mkdir -p /home/$USERNAME/ME5413_Final_Project
RUN chmod 777 /home/$USERNAME/ME5413_Final_Project
COPY ./src /home/$USERNAME/ME5413_Final_Project/src

RUN mkdir -p /home/$USERNAME/.ros
RUN chmod 777 /home/$USERNAME/.ros
RUN mkdir -p /home/$USERNAME/.ignition
RUN chmod 777 /home/$USERNAME/.ignition
RUN mkdir -p /home/$USERNAME/.gazebo
RUN chmod 777 /home/$USERNAME/.gazebo
RUN mkdir -p /home/$USERNAME/.cache
RUN chmod 777 /home/$USERNAME/.cache
RUN mkdir -p /home/$USERNAME/.rviz
RUN chmod 777 /home/$USERNAME/.rviz

RUN mkdir -p /home/$USERNAME/.gazebo/models
RUN chown -R ros:ros /home/$USERNAME/.gazebo

COPY ./src/me5413_world/models/ /home/$USERNAME/.gazebo/models

RUN git clone https://github.com/osrf/gazebo_models.git /home/$USERNAME/gazebo_models

RUN cp -r /home/$USERNAME/gazebo_models/* /home/$USERNAME/.gazebo/models

# To avoid using boxes without inertial and collision from github gazebo_models
COPY ./src/me5413_world/models/number1 /home/$USERNAME/.gazebo/models/number1
COPY ./src/me5413_world/models/number2 /home/$USERNAME/.gazebo/models/number2
COPY ./src/me5413_world/models/number3 /home/$USERNAME/.gazebo/models/number3
COPY ./src/me5413_world/models/number4 /home/$USERNAME/.gazebo/models/number4
COPY ./src/me5413_world/models/number5 /home/$USERNAME/.gazebo/models/number5
COPY ./src/me5413_world/models/number6 /home/$USERNAME/.gazebo/models/number6
COPY ./src/me5413_world/models/number7 /home/$USERNAME/.gazebo/models/number7
COPY ./src/me5413_world/models/number8 /home/$USERNAME/.gazebo/models/number8
COPY ./src/me5413_world/models/number9 /home/$USERNAME/.gazebo/models/number9

RUN chown -R ros:ros /home/$USERNAME/.ros
RUN chown -R ros:ros /home/$USERNAME/.ignition
RUN chown -R ros:ros /home/$USERNAME/.cache
RUN chown -R ros:ros /home/$USERNAME/.rviz

USER $USERNAME

RUN rosdep update && rosdep install --from-paths /home/$USERNAME/ME5413_Final_Project/src --ignore-src -r -y
RUN cd /home/$USERNAME/ME5413_Final_Project && /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && catkin_make"

RUN echo "if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then source /opt/ros/${ROS_DISTRO}/setup.bash; fi" >> /home/$USERNAME/.bashrc
RUN echo "if [ -f /home/$USERNAME/ME5413_Final_Project/devel/setup.bash ]; then source /home/$USERNAME/ME5413_Final_Project/devel/setup.bash; fi" >> /home/$USERNAME/.bashrc

WORKDIR /home/$USERNAME/ME5413_Final_Project