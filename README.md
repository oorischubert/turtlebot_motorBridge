# TurtleBot Motor Bridge

A ROS 2 hardware interface package for controlling TurtleBot motors through ESP32 communication.

## Overview

This package provides a hardware interface for ROS 2 control that enables communication between ROS 2 and TurtleBot motors via an ESP32 microcontroller. It implements the `ros2_control` hardware interface to provide a standardized way to control the robot's motors and read their states.

## Features

- Hardware interface for TurtleBot motors
- ESP32 communication integration
- Support for differential drive control
- Real-time motor state monitoring
- ROS 2 control integration

## Dependencies

- ROS 2 (tested with Humble)
- hardware_interface
- rclcpp
- rclcpp_lifecycle
- Boost
- controller_manager
- diff_drive_controller
- joint_state_broadcaster
- robot_state_publisher

## Installation

1. Clone this repository into your ROS 2 workspace:

```bash
cd ~/ros2_ws/src
git clone https://github.com/oorischubert/turtlebot_motorBridge.git
```

2. Install dependencies:

```bash
rosdep install --from-paths src --ignore-src -r -y
```

3. Build the package:

```bash
cd ~/ros2_ws
colcon build --packages-select turtlebot_motorBridge
```

## Usage

1. Source your workspace:

```bash
source ~/ros2_ws/install/setup.bash
```

2. Launch the hardware interface:

```bash
ros2 launch turtlebot_motorBridge diffbot.launch.py
```

## Configuration

The hardware interface can be configured through ROS 2 parameters. Key parameters include:

- Motor communication settings
- ESP32 connection parameters
- Control loop frequency
- Motor limits and safety parameters

## Architecture

The package consists of several key components:

- `DiffBotSystemHardware`: Main hardware interface class implementing the ROS 2 control interface
- `EspComms`: Handles communication with the ESP32 microcontroller
- `Vehicle`: Manages the vehicle state and motor control


## License

This project is licensed under the Apache License 2.0

