#!/bin/bash

echo "===================================================="
echo "   GES ROS2 – Automatic Platform Launcher"
echo "===================================================="

if [ -f /etc/nv_tegra_release ]; then
    echo "[INFO] Jetson detected"
    export ROS_WORKSPACE_JETSON=/home/dev/ROS2inDocker2/ros2_ws
    docker compose --profile robot up --build
else
    echo "[INFO] PC detected"
    export ROS_WORKSPACE_PC=/home/$USER/ROS2inDocker2/ros2_ws
    docker compose --profile sim up --build
fi

