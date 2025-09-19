#!/bin/bash

LIB_DIR="/opt"
OPENCV_VERSION=4.7.0
LIB_INSTALL_DIR="/usr/local"
WS="/home/robot_ws"

# Subtract 2 from total cores
CORES=$(( $(nproc) - 4 ))

# Ensure at least 1 core is used
if [ "$CORES" -lt 1 ]; then
  CORES=1
fi

# List of required packages
DEPENDENCIES=(
    build-essential
    cmake
    git
    curl
)

# Ensure script is run as root
if [ "$EUID" -ne 0 ]; then
    echo "Please run as root (e.g., sudo ./setup.sh)"
    exit 1
fi

# treat ros2 install special 
check_and_install "ros-humble-desktop" "install_ros"

# Loop through and install each
for pkg in "${DEPENDENCIES[@]}"; do
    check_and_install "$pkg"
done

# Ensure the directory exists
mkdir -p "$LIB_DIR"
mkdir -p "$WS"/src

# Assumes robot_idl was cloned first and runs this install script
git clone https://github.com/samlovelace/robot_idl.git "$WS/src/robot_idl"
git clone https://github.com/samlovelace/arm_configs.git "$WS/src/arm_configs"

#pcl 
cd "$LIB_DIR"
git clone https://github.com/PointCloudLibrary/pcl.git
cd pcl
git checkout pcl-1.15.0
mkdir build && cd build 
cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_SHARED_LIBS=ON \
  -DBUILD_tools=OFF \
  -DBUILD_examples=OFF \
  -DWITH_VTK=ON

make -j"$CORES" && make install && ldconfig

cd "$WS"
source /opt/ros/humble/setup.bash
colcon build --packages-select robot_idl arm_configs arm 





