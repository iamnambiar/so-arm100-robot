# SO-ARM-100 ROS2 Packages

This repository contains ROS2 packages for the SO-ARM-100 robot, providing simulation, control, and motion planning capabilities.

## Packages

- **soarm100_control**: Controllers and hardware interfaces for the SO-ARM-100 robot
- **soarm100_description**: URDF models, meshes, and configuration files that describe the robot
- **soarm100_gazebo**: Simulation environments and launch files for Gazebo
- **soarm100_moveit_config**: MoveIt2 configuration for motion planning and manipulation tasks

## Installation

### Prerequisites

- ROS2 (tested with Jazzy)
- Gazebo
- MoveIt2

### Building from Source

```bash
# Create a workspace
mkdir -p ~/soarm100_ws/src
cd ~/soarm100_ws/src

# Clone the repository
git clone https://github.com/iamnambiar/so-arm100-robot.git

# Install dependencies
cd ..
rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
colcon build --symlink-install
source install/setup.bash
```

## Usage

### Simulation

Launch the robot in Gazebo:

```bash
ros2 launch soarm100_gazebo soarm100_gazebo.launch.py
```
<!-- 
### Motion Planning with MoveIt2

```bash
ros2 launch soarm100_moveit_config demo.launch.py
``` -->

<!-- ## Contributing

Contributions are welcome! Please feel free to submit a Pull Request. -->

## Credits

The robot definition is based on the [SO-ARM100 repository](https://github.com/TheRobotStudio/SO-ARM100) by The Robot Studio.

## License

This project is licensed under the Apache License 2.0 - see the [LICENSE](LICENSE) file for details.