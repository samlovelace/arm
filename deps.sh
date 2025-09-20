#!/bin/bash

# get workspace path from caller of this script 
WS=$1

LIB_DIR="/opt"
LIB_INSTALL_DIR="/usr/local"

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
	libeigen3-dev
	libyaml-cpp-dev
)

install_pcl()
{
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

	make -j"$CORES" && $SUDO make install && ldconfig
}

function run_custom_build_steps(){

  install_pcl
}
