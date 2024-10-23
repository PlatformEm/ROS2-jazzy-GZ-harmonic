# ROS2 Jazzy with Gazebo Harmonic Integration Example

# Building this workspace is must. use colcon build to build the workspace.

This repository demonstrates how to integrate ROS2 Jazzy Jalisco with Gazebo GZ Harmonic, showcasing a differential drive robot in a warehouse environment. This project addresses the transition from Gazebo Classic (now deprecated) to Gazebo Harmonic, which is natively supported in ROS2 Jazzy.

## Background

While following Edouard Renard's excellent ROS2 Udemy course (which uses ROS2 Humble with Gazebo Classic), I encountered challenges porting the examples to work with GZ Harmonic. This repository serves as a reference for others making this transition, documenting solutions to common issues and providing working examples.

## Features

- Complete URDF/Xacro robot definition with differential drive capabilities
- Integration with Gazebo GZ Harmonic simulation environment
- Custom warehouse environment (modified Tugbot depot)
- Bridge configuration between ROS2 Jazzy and GZ Harmonic
- Launch files for coordinated startup of all components
- RViz2 visualization support

## Project Structure

```
.
├── robot_description/
│   ├── urdf/
│   │   └── robot.urdf.xacro
│   ├── params/
│   │   └── my_robot_bridge.yaml
│   └── ...
├── robot_bringup/
│   ├── launch/
│   │   └── robot_launch.py
│   └── ...
└── worlds/
    └── tugbot_depot.sdf
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

```bash
# Clone this repository
git clone https://github.com/ThiwakarS/ROS2-jazzy-GZ-harmonic.git

# Install dependencies
cd ~/ros2_ws_urdf
rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
colcon build
```

## Usage

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

## Key Differences from Gazebo Classic

1. **Plugin System**: GZ Harmonic uses a different plugin architecture
2. **Bridge Configuration**: Requires explicit bridge setup between ROS2 and Gazebo
3. **Launch System**: Updated launch file syntax and component organization
4. **World Format**: Uses SDF format with updated syntax
5. **Topic Names**: Different default topic naming conventions

## Common Issues and Solutions

1. **Bridge Communication**: Ensure bridge configurations match exact topic names
2. **Transform Trees**: Check for missing transforms in URDF
3. **Plugin Compatibility**: Use Harmonic-compatible plugins
4. **Resource Paths**: Update model and mesh paths for GZ Harmonic

## References

- [Turtlebot3 ROS2 Implementation](https://github.com/ROBOTIS-GIT/turtlebot3/tree/ros2)
- [Gazebo GZ Harmonic Documentation](https://gazebosim.org/docs/latest/getstarted/)
- ROS2 Jazzy Documentation
- Special thanks to Claude.ai for debugging assistance

## Contributing

Contributions are welcome! Please feel free to submit pull requests or create issues for bugs and feature requests.

## Support

If you encounter any difficulties or have questions, please:
1. Check the [Issues](https://github.com/ThiwakarS/ROS2-jazzy-GZ-harmonic/issues) section
2. Create a new issue if your problem isn't already documented
3. Contact me at thiwakarprivate1230@gmail.com

## License

This project is licensed under the MIT License - see the LICENSE file for details.
