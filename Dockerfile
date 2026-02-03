FROM osrf/ros:humble-desktop

# Install essential tools and Gazebo keys
USER root
RUN apt-get update && apt-get install -y \
    lsb-release \
    gnupg \
    curl \
    git \
    python3-colcon-common-extensions \
    python3-vcstool \
    sudo

RUN curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
RUN apt-get update && apt-get install -y ignition-fortress

# Environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV DISPLAY=:0
ENV HOME=/home/user
ENV ROS_DISTRO=humble
ENV GZ_SIM_RESOURCE_PATH=${HOME}/ros2_ws/install/final_project/share

# Add non-root user
ARG USER_ID
ARG GROUP_ID
RUN addgroup --gid $GROUP_ID user && \
    adduser --disabled-password --gecos '' --uid $USER_ID --gid $GROUP_ID user && \
    echo "user:user" | chpasswd && \
    echo "user ALL=(ALL:ALL) ALL" >> /etc/sudoers
    
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV GZ_VERSION=fortress
ENV IGN_GAZEBO_SYSTEM_PLUGIN_PATH=/opt/ros/humble/lib

USER user
RUN mkdir -p ${HOME}/ros2_ws/src
WORKDIR ${HOME}/ros2_ws/
RUN rosdep update

# ROS2 packages installation
USER root
RUN apt-get update && apt-get upgrade -y && apt-get install -y \
    # Bridge e Simulazione (PULITO: rimosso gazebo-ros-pkgs e aggiunto ignition-fortress)
    ros-humble-ros-ign-bridge \
    ros-humble-ros-ign-gazebo \
    ros-humble-ros-ign-interfaces \
    ros-humble-ign-ros2-control \
    ignition-fortress \
    # Controllo
    ros-humble-controller-manager \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-joint-state-publisher \
    ros-humble-joint-trajectory-controller \
    ros-humble-ros2-control-test-assets \
    # MoveIt 2
    ros-humble-moveit \
    ros-humble-moveit-ros-planning \
    ros-humble-moveit-planners-ompl \
    ros-humble-moveit-ros-move-group \
    ros-humble-moveit-servo \
    ros-humble-moveit-msgs \
    ros-humble-moveit-visual-tools \
    ros-humble-pilz-industrial-motion-planner \
    # Navigazione e SLAM
    ros-humble-slam-toolbox \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-nav2-amcl \
    # Cinematica e Utility
    ros-humble-kdl-parser \
    liborocos-kdl-dev \
    ros-humble-tf2-eigen \
    ros-humble-rqt-tf-tree \
    ros-humble-xacro && \
    rm -rf /var/lib/apt/lists/*
# Configurazione Bash
USER user
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash;" >> ${HOME}/.bashrc && \
    echo "source ${HOME}/ros2_ws/install/local_setup.bash;" >> ${HOME}/.bashrc && \
    echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> ${HOME}/.bashrc && \
    echo "export _colcon_cd_root=/opt/ros/${ROS_DISTRO}/" >> ${HOME}/.bashrc && \
    echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ${HOME}/.bashrc && \
    echo "export GAZEBO_AUDIO=0" >> ${HOME}/.bashrc 
    

# Final cleanup
USER root
RUN rm -rf /var/lib/apt/lists/*
USER user
