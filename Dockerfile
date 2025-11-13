##########################################################
# ROS 2 HUMBLE – MULTI ARCH BUILDER
# x86_64  -> PC / Simulation
# arm64   -> Jetson Xavier – Real Robot
##########################################################

ARG TARGET_ARCH
ARG ROS_DISTRO=humble


# --------------------------------------------------------
# 1) PLATFORM SEÇİMİ
# --------------------------------------------------------
FROM --platform=linux/amd64 osrf/ros:${ROS_DISTRO}-desktop-full AS base_x86
FROM --platform=linux/arm64 dustynv/ros:${ROS_DISTRO}-desktop-l4t-r36.3.0 AS base_arm

# --------------------------------------------------------
# 2) HEDEF PLATFORMU SEÇ
# --------------------------------------------------------
FROM base_${TARGET_ARCH}

# --------------------------------------------------------
# ORTAK ENV
# --------------------------------------------------------
ARG DEBIAN_FRONTEND=noninteractive
ARG USERNAME=teknolus
ARG USER_UID=1000
ARG USER_GID=1000
ARG WORKSPACE=/home/${USERNAME}/ros2_ws

ENV SHELL=/bin/bash \
    ROS_DISTRO=${ROS_DISTRO} \
    ROS_DOMAIN_ID=50 \
    QT_X11_NO_MITSHM=1 \
    DEBIAN_FRONTEND=noninteractive

# --------------------------------------------------------
# 3) ORTAK PAKETLER (PC + JETSON)
# --------------------------------------------------------
RUN apt-get update && apt-get install -y \
    python3-pip python3-colcon-common-extensions \
    python3-rosdep python3-vcstool \
    git nano curl wget build-essential \
    && rosdep init || true \
    && rosdep update \
    && rm -rf /var/lib/apt/lists/*

# Ortak Python paketleri
RUN pip3 install \
    numpy scipy pyserial \
    opencv-python==4.8.0.76 \
    transforms3d \
    setuptools==68.0.0

# --------------------------------------------------------
# 4) PC SPESİFİK (x86_64) — Gazebo + RViz2
# --------------------------------------------------------
RUN if [ "$(uname -m)" = "x86_64" ]; then \
    sudo apt-get update && \
    sudo apt-get install -y lsb-release wget gnupg && \
    wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/gazebo-archive-keyring.gpg && \
    echo "deb [signed-by=/usr/share/keyrings/gazebo-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
      > /etc/apt/sources.list.d/gazebo-stable.list && \
    apt-get update && \
    apt-get install -y \
      ignition-fortress \
      ros-humble-ros-ign \
      ros-humble-ros-ign-gazebo \
      ros-humble-rviz2 \
    && rm -rf /var/lib/apt/lists/*; \
    fi
# --------------------------------------------------------
# 5) JETSON SPESİFİK (arm64) — CUDA + TensorRT
# --------------------------------------------------------
RUN if [ "$(uname -m)" = "aarch64" ]; then \
    apt-get update && apt-get install -y \
    nvidia-cuda-toolkit \
    libnvinfer8 libnvinfer-plugin8 libnvinfer-dev \
    libgstreamer1.0-dev gstreamer1.0-tools \
    && rm -rf /var/lib/apt/lists/*; \
    fi

# --------------------------------------------------------
# Kullanıcı oluşturma
# --------------------------------------------------------
RUN groupadd --gid ${USER_GID} ${USERNAME} \
    && useradd --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME}

USER ${USERNAME}
WORKDIR /home/${USERNAME}

# --------------------------------------------------------
# Ortak workspace
# --------------------------------------------------------
RUN mkdir -p ${WORKSPACE}

CMD ["bash"]

