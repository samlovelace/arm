# Agnostic Robot Manipulation (ARM)

![Ubuntu 22.04](https://github.com/samlovelace/arm/actions/workflows/ci-ubuntu.yaml/badge.svg?branch=main)

## Overview

The **Agnostic Robot Manipulation (ARM)** module is a general-purpose planning and control system for robotic manipulators.

## Dependencies

The list of dependencies are in deps.sh. This file will be used by ptera_msgs to install the required dependencies.

## Install

This module depends on custom ROS2 msgs defined in the ptera_msgs repo. Clone ptera_msgs into a workspace and run the setup script for the vehicle as shown below.

```bash
$ mkdir -p ~/robot_ws/src
$ git clone https://github.com/samlovelace/ptera_msgs.git
$ cd ptera_msgs/scripts
$ chmod +x setup.sh
$ sudo ./setup manipulation
```

## Run

To run the arm module, from the root of the workspace

```bash
source install/setup.bash
ros2 run arm arm
```
