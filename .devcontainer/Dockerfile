# Base image with Ubuntu 22.04 (commonly used for robotics dev)
FROM ubuntu:22.04

# Avoid user interaction during package installs
ENV DEBIAN_FRONTEND=noninteractive

# Install basic dev tools
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    wget \
    curl \
    sudo \
    gdb \
    python3 \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Install ROS2 Humble (can be adjusted if you need ROS1/ROS2)
RUN apt-get update && apt-get install -y locales \
 && locale-gen en_US en_US.UTF-8 \
 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
 && export LANG=en_US.UTF-8 \
 && apt-get install -y software-properties-common \
 && add-apt-repository universe \
 && apt-get update && apt-get install -y \
    ros-humble-desktop \
    python3-rosdep \
    python3-colcon-common-extensions \
    python3-argcomplete \
    && rm -rf /var/lib/apt/lists/*

# Setup ROS environment
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

# Default working directory
WORKDIR /workspace
