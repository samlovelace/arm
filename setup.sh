#!/bin/bash

# Function to check and install a package 
check_and_install() {
    PKG="$1"
    CUSTOM_INSTALL_FUNC="$2"

    if dpkg -s "$PKG" >/dev/null 2>&1; then
        echo "[✔] $PKG is already installed."
    else
        echo "[✘] $PKG is not installed."

        if [ -n "$CUSTOM_INSTALL_FUNC" ] && declare -f "$CUSTOM_INSTALL_FUNC" > /dev/null; then
            echo "[↪] Using custom install function: $CUSTOM_INSTALL_FUNC"
            "$CUSTOM_INSTALL_FUNC"
        else
            echo "[↪] Installing $PKG via apt..."
            apt update && apt install -y "$PKG"
        fi
    fi
}

install_ros() {
    echo "[ROS] Starting installation of ROS 2 Humble..."

    # --- Configure UTF-8 Locale ---
    if ! locale | grep -q "UTF-8"; then
        echo "[ROS] Configuring UTF-8 locale..."
        apt install -y locales
        locale-gen en_US en_US.UTF-8
        update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
        export LANG=en_US.UTF-8
    fi

    # --- Set Timezone Non-Interactively ---
    echo "[ROS] Setting timezone to America/New_York..."
    ln -fs /usr/share/zoneinfo/America/New_York /etc/localtime
    DEBIAN_FRONTEND=noninteractive apt install -y tzdata
    dpkg-reconfigure -f noninteractive tzdata

    # --- Add Required Tools ---
    apt install -y software-properties-common curl gnupg lsb-release

    # --- Add Universe Repo ---
    add-apt-repository universe

    # --- Add ROS 2 GPG Key ---
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg

    # --- Add ROS 2 Repository ---
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
        http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
        > /etc/apt/sources.list.d/ros2.list

    # --- Install ROS 2 Humble and Tools ---
    apt update
    apt install -y ros-humble-desktop \
                   python3-colcon-common-extensions \
                   python3-rosdep \
                   python3-vcstool

    # --- Source ROS Environment ---
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
    source /opt/ros/humble/setup.bash

    echo "[✔] ROS 2 Humble installed and environment sourced."
}


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





