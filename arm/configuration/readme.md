# arm_configs

The **arm_configs** module is a centralized location for manipulation configurations.

## Dependencies

The list of dependencies are in deps.sh. This file will be used by robot_idl to install the required dependencies.

## Install

This module depends on custom ROS2 msgs defined in the robot_idl repo. Clone robot_idl into a workspace and run the setup script for the vehicle as shown below.

```bash
$ mkdir -p ~/robot_ws/src
$ git clone https://github.com/samlovelace/robot_idl.git
$ cd robot_idl/scripts
$ chmod +x setup.sh
$ sudo ./setup manipulation
```
