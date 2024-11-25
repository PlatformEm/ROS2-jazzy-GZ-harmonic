# 🤖✨ Emergence 3 Platform Project: Controlling Three Robotic Arms with ROS and Raspberry Pi! 🌟

Welcome to the **EM3 Platform Project** repository! 🚀 This repo is all about an exciting adventure in robotics—getting **three robotic arms** to move using software written for **ROS Jazzy** on a **Raspberry Pi 5** running **Ubuntu 24**. This repo was brought to life by forking an existing bridge between ROS2 Jazzy and Gazebo Harmonic.

## 🌈 Project Overview

This project contains code for controlling 3 robotic arms simultaneously, focusing on:
- Setting up the arms with **ROS Jazzy**.
- Writing custom software to define movements and control each arm.
- Simulating the arms with **Gazebo**
- Testing and troubleshooting using **Ubuntu 24** on a **Raspberry Pi 5**.


# Building this workspace is must. use colcon build to build the workspace.

This repository demonstrates how to integrate ROS2 Jazzy Jalisco with Gazebo GZ Harmonic, showcasing a differential drive robot in a warehouse environment. This project addresses the transition from Gazebo Classic (now deprecated) to Gazebo Harmonic, which is natively supported in ROS2 Jazzy.

## Features

- Complete URDF/Xacro robot definition with differential drive capabilities
- Integration with Gazebo GZ Harmonic simulation environment
- Custom warehouse environment (modified Tugbot depot)
- Bridge configuration between ROS2 Jazzy and GZ Harmonic
- Launch files for coordinated startup of all components
- RViz2 visualization support

## 🌳 Project Structure

```
.
├── robot_description/ # 🎨 Resources related to the robotic arm setup (configs, URDF, models)
│   ├── urdf/
│   │   └── robot.urdf.xacro
│   ├── params/
│   │   └── my_robot_bridge.yaml
│   └── ...
├── robot_bringup/
│   ├── launch/
│   │   └── robot_launch.py  # 🐍 Script for setting up the Python-based ROS package
│   └── ...
└── worlds/
    └── tugbot_depot.sdf
├── LICENSE           # ⚖️ License information
└── README.md         # 📖 You’re reading it now!
```

## Key Components

### 1. Robot Description
- URDF/Xacro files defining robot structure and properties
- Differential drive plugin adapted from Turtlebot3
- Sensor configurations and transforms

### 2. Gazebo Integration
- Modified Tugbot depot world file (removed original robot)
- GZ Harmonic-compatible SDF world definition
- Plugin configurations for physics and sensors

### 3. ROS2-Gazebo Bridge Configuration
Located in `my_robot_description/params/my_robot_bridge.yaml`, defines:
- Topic mappings between ROS2 and Gazebo
- Transform configurations
- Sensor data bridges
- Control interfaces

### 4. Launch System
The launch file coordinates:
- Robot state publisher
- Gazebo simulation
- ROS2-Gazebo bridges
- RViz2 visualization
- Parameter loading

## Prerequisites

- ROS2 Jazzy Jalisco
- Gazebo GZ Harmonic
- Required ROS2 packages:
  - ros2_control
  - ros_gz_sim
  - rviz2

## Installation

## 🛠️ Getting Started

To get the robotic arms moving, follow these steps to set up ROS2 on your Raspberry Pi 5:

### Step 1: Flash Ubuntu

1. **Find a compatible Ubuntu distribution** for ROS on your Pi 5. To find a suitable version, Refer to the [ROS 2 REP 2000](https://ros.org/reps/rep-2000.html). For instance, if you choose ROS Jazzy Jalisco, you will use Ubuntu Noble (24.04).
   
2. **Download the Ubuntu desktop image** compatible with your chosen ROS version.

### Step 2: Boot and Set Up Ubuntu

1. **Boot from the SD** by plugging it into the Pi and turning it on.

2. **Get a Coffee** the system will be installing things so yeah you have to wait until it is done. take a break.

3. **Update the Linux system** by running the following commands in the terminal:

```bash
   sudo apt update
   sudo apt upgrade
```
### Step 3: Install ROS Jazzy

Install ROS by following the normal binary installation procedure. You can find detailed instructions in the ROS 2 Documentation.

Source the setup files:

```bash
source /opt/ros/jazzy/setup.bash
```

Set the ROS domain ID:

```bash
export ROS_DOMAIN_ID=1
```

Check the need to change the `ROS_AUTOMATIC_DISCOVERY_RANGE` based on your network configuration.

Follow the tutorial from the ROS2 Documentation for Jazzy to further configure your environment.

### Step 4: Install the bridge between ROS2 Jazzy and Gazebo Harmonic

1. Update and upgrade your Distro and its current applications
```bash
sudo apt update
sudo apt upgrade
```

```bash
2. Clone this repository
git clone https://github.com/ThiwakarS/ROS2-jazzy-GZ-harmonic.git

3. Install dependencies
cd ~/ros2_ws_urdf
rosdep install --from-paths src --ignore-src -r -y

4. Build the workspace using colcon
colcon build
```

## Usage

0. Go up one level
```bash
cd ..
```


1. Source your ROS2 workspace:
```bash
source ~/ros2_ws_urdf/install/setup.bash
```

2. Launch the simulation:
```bash
ros2 launch my_robot_bringup my_robot_gz.launch.py
```

3. Control the robot using geometry messages:
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.3}}"
```

## Common Issues and Solutions

1. **Bridge Communication**: Ensure bridge configurations match exact topic names
2. **Transform Trees**: Check for missing transforms in URDF
3. **Plugin Compatibility**: Use Harmonic-compatible plugins
4. **Resource Paths**: Update model and mesh paths for GZ Harmonic

## References

- [Forked Repository that serves as the basis for this project](https://github.com/ThiwakarS/ROS2-jazzy-GZ-harmonic)
- [Turtlebot3 ROS2 Implementation](https://github.com/ROBOTIS-GIT/turtlebot3/tree/ros2)
- [Gazebo GZ Harmonic Documentation](https://gazebosim.org/docs/latest/getstarted/)
- [ROS2 Jazzy Documentation](https://docs.ros.org/en/jazzy/)

## License

This project is licensed under the MIT License - see the LICENSE file for details.
