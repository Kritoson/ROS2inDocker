##########################################################
# ROS 2 HUMBLE – MULTI ARCH BUILDER
# x86_64  -> PC / Simulation
# arm64   -> Jetson Xavier – Real Robot
##########################################################

# Ortak build arg'lar
ARG TARGET_ARCH                    # x86 veya arm
ARG ROS_DISTRO=humble

# Jetson L4T Tag (JetPack 5.1.2 -> L4T r35.4.1)
ARG L4T_TAG=r35.4.1

##########################################################
# 1) PLATFORM SEÇİMİ
##########################################################
# PC / x86_64 base
FROM --platform=linux/amd64 osrf/ros:${ROS_DISTRO}-desktop-full AS base_x86

# Jetson / arm64 base (JetPack 5.1.2 → L4T r35.4.1)
# İstersen "desktop" yerine "ros-base" ile daha hafif image kullanabilirsin.
FROM --platform=linux/arm64 dustynv/ros:${ROS_DISTRO}-ros-base-l4t-${L4T_TAG} AS base_arm

##########################################################
# 2) HEDEF PLATFORM SEÇİMİ
##########################################################
# docker build arg olarak TARGET_ARCH=x86 veya arm gelecek
FROM base_${TARGET_ARCH} AS runtime

RUN rm -f /etc/apt/sources.list.d/ros* \
 && rm -f /etc/apt/sources.list.d/ros2* \
 && sed -i '/ros2/d' /etc/apt/sources.list \
 && sed -i '/packages\.ros\.org/d' /etc/apt/sources.list

# Kullanıcı ve workspace konfigürasyonu
ARG USERNAME=rosuser
ARG USER_UID=1000
ARG USER_GID=1000
ARG WORKSPACE=/workspace/ros2_ws

# Ortak env'ler
ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=${ROS_DISTRO} \
    ROS_DOMAIN_ID=50 \
    RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    WORKSPACE=${WORKSPACE} \
    FASTDDS_DEFAULT_PROFILES_FILE=/dds/fastdds.xml

##########################################################
# 3) TEMEL ARAÇLAR & BAĞIMLILIKLAR
##########################################################
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    git \
    vim \
    nano \
    locales \
    python3-pip \
    #python3-colcon-common-extensions \
    #python3-rosdep \
    #python3-vcstool \
    net-tools \
    iputils-ping \
    mesa-utils\
    && rm -rf /var/lib/apt/lists/*

# Locale
RUN locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

ENV LANG=en_US.UTF-8 \
    LC_ALL=en_US.UTF-8

##########################################################
# 4) USER & WORKSPACE
##########################################################
ARG USERNAME
ARG USER_UID
ARG USER_GID
ARG WORKSPACE

RUN groupadd --gid ${USER_GID} ${USERNAME} \
    && useradd --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME} \
    && mkdir -p ${WORKSPACE} /dds \
    && chown -R ${USERNAME}:${USER_GID} ${WORKSPACE} /dds

RUN echo 'if [ -f "/opt/ros/$ROS_DISTRO/setup.bash" ]; then \
  source /opt/ros/$ROS_DISTRO/setup.bash; \
fi' >> /home/${USERNAME}/.bashrc

USER ${USERNAME}
WORKDIR ${WORKSPACE}

##########################################################
# 5) DDS / FAST-DDS PROFİL ALTYAPISI
##########################################################
# Buraya minimal bir fastdds.xml placeholder bırakıyoruz.
# Gerçek tuning için host'tan bind-mount ile de üzerine yazabilirsin.
RUN echo '<?xml version="1.0" encoding="UTF-8"?> \
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles"> \
  <participant profile_name="default_participant" is_default_profile="true"> \
    <rtps> \
      <use_builtin_transports>true</use_builtin_transports> \
      <default_unicast_locator_list> \
        <!-- Gerekirse buraya unicast IP/port tuning ekleyebilirsin --> \
      </default_unicast_locator_list> \
      <builtin> \
        <domainId>${ROS_DOMAIN_ID}</domainId> \
        <leaseDuration> \
          <sec>10</sec><nanosec>0</nanosec> \
        </leaseDuration> \
      </builtin> \
    </rtps> \
  </participant> \
</profiles>' > /dds/fastdds.xml

##########################################################
# 6) ENTRYPOINT / LIFECYCLE
##########################################################
# ROS ortamını ve workspace'i source eden standart entrypoint
USER root

RUN printf '%s\n' \
'#!/bin/bash' \
'set -e' \
'if [ -f "/opt/ros/$ROS_DISTRO/install/setup.bash" ]; then' \
'  source /opt/ros/$ROS_DISTRO/install/setup.bash' \
'fi' \
'if [ -f "$WORKSPACE/install/setup.bash" ]; then' \
'  source "$WORKSPACE/install/setup.bash"' \
'fi' \
'exec "$@"' \
> /ros_entrypoint.sh \
 && chmod +x /ros_entrypoint.sh
RUN echo "source /opt/ros/\$ROS_DISTRO/install/setup.bash" >> /home/${USERNAME}/.bashrc

# -----------------------
# Tekrar kullanıcıya dön
# -----------------------
USER ${USERNAME}

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
