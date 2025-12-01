#!/usr/bin/env bash
set -e

# ------------------------------
#  PLATFORM AUTO-DETECT
# ------------------------------
ARCH=$(uname -m)

if [ "$ARCH" = "x86_64" ]; then
    TARGET="pc"
    DOCKERFILE="dockerfile.pc"
elif [[ "$ARCH" == "aarch64" || "$ARCH" == "arm64" ]]; then
    TARGET="jetson"
    DOCKERFILE="dockerfile.jetson"
else
    echo "Unsupported architecture: $ARCH"
    exit 1
fi

echo "[INFO] Building GES ROS2 image for: $TARGET"
echo "[INFO] Using Dockerfile: $DOCKERFILE"

# ------------------------------
#  BUILD IMAGE
# ------------------------------
docker build \
    -f $DOCKERFILE \
    -t ros2_$TARGET \
    .

echo "[INFO] Build completed: ros2_$TARGET"
