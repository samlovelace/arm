# Agnostic Robot Manipulation (ARM)

![Ubuntu 22.04](https://github.com/samlovelace/arm/actions/workflows/ci-ubuntu.yaml/badge.svg?branch=main)

## Overview

The **Agnostic Robot Manipulation (ARM)** module is a general-purpose planning and control system for robotic manipulators.

## Dependencies

- Ubuntu 22.04 (developed on)
- ROS2 Humble
- Gazebo Ignition-Fortress (arm_simulator)
- yaml-cpp
- eigen

## Install

To install and set up the ARM module, first create a workspace

```sh
mkdir -p arm_ws/src
cd arm_ws/src
```

Then clone this repo

```sh
git clone https://github.com/samlovelace/arm.git
cd arm
```

To clone other repos needed for compiling this module as well as simulator different manipulators and sending commands

```sh
chmod +x clone.sh
./clone.sh
```

## Build

Navigate to the root of the workspace

```sh
cd arm_ws
```

and build

```sh
colcon build
```

## Run

To run the ARM module

```sh
ros2 run arm arm
```
