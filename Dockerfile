FROM nvidia/cuda:11.0-cudnn8-runtime-ubuntu18.04

RUN echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections
RUN apt-get update && apt-get install -y lsb-release && apt-get clean all

# Install ROS and other required dependencies for RMUA Simulator
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-get install -y curl && curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
RUN apt-get update && apt-get install -y \
    ros-melodic-desktop-full \
    protobuf-compiler \
    libprotoc-dev \
    ros-melodic-libg2o \
    ros-melodic-controller-manager \
    ros-melodic-realtime-tools  \
    ros-melodic-rosserial-server \
    ros-melodic-serial \
    ros-melodic-robot-localization \
    libgoogle-glog0v5 libgoogle-glog-dev \
    ros-melodic-move-base \
    ros-melodic-interactive-marker-twist-server \
    ros-melodic-gazebo-ros-control \
    ros-melodic-hector-gazebo-plugins \
    ros-melodic-joint-state-controller  \
    ros-melodic-joint-trajectory-controller \
    ros-melodic-lms1xx \
    libarmadillo-dev \
    ros-melodic-map-server

RUN echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
RUN apt-get install -y python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
RUN rosdep init && rosdep update

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

# Useful utilities
RUN apt-get install -y nano wget git tree screen htop
