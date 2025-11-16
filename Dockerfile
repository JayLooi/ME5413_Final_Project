FROM ros:jazzy

ARG ROS_DISTRO=jazzy
ARG USERNAME=ros
ARG USER_ID=1000

ENV HOME=/home/${USERNAME}
ENV DEBIAN_FRONTEND=noninteractive

RUN _OLD_USER=$(getent passwd ${USER_ID}) && _OLD_USERNAME=$(echo ${_OLD_USER} | cut -d : -f1) && deluser ${_OLD_USERNAME}
RUN useradd -m -s /bin/bash -u ${USER_ID} ${USERNAME}
# RUN echo ${USERNAME} ALL = NOPASSWD: /usr/bin/apt, /usr/bin/apt-get, /usr/bin/curl, /usr/bin/dpkg > /etc/sudoers.d/${USERNAME} && chmod 0440 /etc/sudoers.d/${USERNAME}
RUN echo ${USERNAME} ALL = NOPASSWD: ALL > /etc/sudoers.d/${USERNAME} && \
    chmod 0440 /etc/sudoers.d/${USERNAME}

RUN apt update && apt upgrade -y

RUN apt install -y \
    curl lsb-release gnupg git wget \
    ros-$ROS_DISTRO-vision-msgs \
    ros-$ROS_DISTRO-cv-bridge \
    ros-$ROS_DISTRO-sensor-msgs \
    ros-$ROS_DISTRO-tf2-ros \
    ros-$ROS_DISTRO-tf2-geometry-msgs \
    ros-$ROS_DISTRO-tf2-sensor-msgs \
    ros-$ROS_DISTRO-ros-gz \
    ros-$ROS_DISTRO-gz-ros2-control \
    ros-$ROS_DISTRO-teleop-twist-keyboard \
    ros-$ROS_DISTRO-clearpath-simulator \
    ros-$ROS_DISTRO-sick-scan-xd
    ## Not in Jazzy
    # ros-$ROS_DISTRO-jsk-rviz-plugins \
    # ros-$ROS_DISTRO-jackal-gazebo \
    # ros-$ROS_DISTRO-jackal-navigation \
    # ros-$ROS_DISTRO-velodyne-simulator \
    # ros-$ROS_DISTRO-explore-lite \
    # ros-$ROS_DISTRO-jsk-visualization

# Install Clearpath-related dependencies
RUN mkdir -p ~/clearpath_ws/src && cd ~/clearpath_ws && \
    wget https://raw.githubusercontent.com/clearpathrobotics/clearpath_simulator/jazzy/dependencies.repos && \
    vcs import src < dependencies.repos && \
    rosdep update && rosdep install -r --from-paths src -i -y

# # Install Gazebo Harmonic
# RUN curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
# RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
# RUN apt update -y && apt-get install -y gz-harmonic

RUN chown -R ros:ros /home/${USERNAME}/.ros

WORKDIR ${HOME}
USER ${USERNAME}

# RUN pip3 install numpy easyocr
# RUN pip3 install Pillow==9.3.0

# RUN mkdir -p /home/$USERNAME/ME5413_Final_Project
# RUN chmod 777 /home/$USERNAME/ME5413_Final_Project
# COPY ./src /home/$USERNAME/ME5413_Final_Project/src

# RUN mkdir -p /home/$USERNAME/.ros
# RUN chmod 777 /home/$USERNAME/.ros
# RUN mkdir -p /home/$USERNAME/.ignition
# RUN chmod 777 /home/$USERNAME/.ignition
# RUN mkdir -p /home/$USERNAME/.gazebo
# RUN chmod 777 /home/$USERNAME/.gazebo
# RUN mkdir -p /home/$USERNAME/.cache
# RUN chmod 777 /home/$USERNAME/.cache
# RUN mkdir -p /home/$USERNAME/.rviz
# RUN chmod 777 /home/$USERNAME/.rviz

# RUN mkdir -p /home/$USERNAME/.gazebo/models
# RUN chown -R ros:ros /home/$USERNAME/.gazebo

# COPY ./src/me5413_world/models/ /home/$USERNAME/.gazebo/models

# RUN git clone https://github.com/osrf/gazebo_models.git /home/$USERNAME/gazebo_models

# RUN cp -r /home/$USERNAME/gazebo_models/* /home/$USERNAME/.gazebo/models

# # To avoid using boxes without inertial and collision from github gazebo_models
# COPY ./src/me5413_world/models/number1 /home/$USERNAME/.gazebo/models/number1
# COPY ./src/me5413_world/models/number2 /home/$USERNAME/.gazebo/models/number2
# COPY ./src/me5413_world/models/number3 /home/$USERNAME/.gazebo/models/number3
# COPY ./src/me5413_world/models/number4 /home/$USERNAME/.gazebo/models/number4
# COPY ./src/me5413_world/models/number5 /home/$USERNAME/.gazebo/models/number5
# COPY ./src/me5413_world/models/number6 /home/$USERNAME/.gazebo/models/number6
# COPY ./src/me5413_world/models/number7 /home/$USERNAME/.gazebo/models/number7
# COPY ./src/me5413_world/models/number8 /home/$USERNAME/.gazebo/models/number8
# COPY ./src/me5413_world/models/number9 /home/$USERNAME/.gazebo/models/number9

# RUN rosdep update && rosdep install --from-paths /home/$USERNAME/ME5413_Final_Project/src --ignore-src -r -y
# # RUN cd /home/$USERNAME/ME5413_Final_Project && /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && catkin_make"

# RUN echo "if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then source /opt/ros/${ROS_DISTRO}/setup.bash; fi" >> /home/$USERNAME/.bashrc
# RUN echo "if [ -f /home/$USERNAME/ME5413_Final_Project/devel/setup.bash ]; then source /home/$USERNAME/ME5413_Final_Project/devel/setup.bash; fi" >> /home/$USERNAME/.bashrc

# WORKDIR /home/$USERNAME/ME5413_Final_Project
