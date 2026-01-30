# DOBOT Magician ROS 2

[![ROS 2](https://img.shields.io/badge/ROS_2-Jazzy-22314E?style=for-the-badge&logo=ros&logoColor=white)](https://docs.ros.org/en/jazzy/)
[![Python](https://img.shields.io/badge/Python-3.10+-3776AB?style=for-the-badge&logo=python&logoColor=white)](https://www.python.org/)
[![MoveIt 2](https://img.shields.io/badge/MoveIt_2-Motion_Planning-orange?style=for-the-badge)](https://moveit.ros.org/)
[![Gazebo](https://img.shields.io/badge/Gazebo-Simulation-green?style=for-the-badge)](https://gazebosim.org/)
[![License](https://img.shields.io/badge/License-Apache_2.0-blue?style=for-the-badge)](LICENSE)

A ROS 2 workspace for controlling the **DOBOT Magician** robotic arm, featuring complete simulation support with Gazebo, motion planning with MoveIt 2, and a task-based action server interface.

## Overview

This project provides a full ROS 2 stack for the DOBOT Magician 3-DOF articulated robotic arm, developed as part of research into automated agricultural processing systems. The current implementation focuses on simulation and motion planning infrastructure, with planned extensions for computer vision integration and specialized end-effector control.

### Key Features

- **Complete URDF/Xacro Model** - Accurate kinematic and dynamic model with collision meshes
- **MoveIt 2 Integration** - Full motion planning pipeline with predefined configurations
- **Gazebo Simulation** - Physics-based simulation using `ros2_control` and `gz_ros2_control`
- **Task Action Server** - High-level task execution interface using ROS 2 actions
- **Multiple End-Effectors** - Support for gripper, suction cup, pen, and custom tools

## Project Structure

```
DOBOT_Magician_ROS/
└── src/
    ├── my_robot_description/    # Robot model and visualization
    ├── my_robot_controller/     # ros2_control configuration
    ├── my_robot_moveit/         # MoveIt 2 motion planning
    ├── my_robot_commander/      # Task server and MoveIt interface
    └── my_robot_msgs/           # Custom action definitions
```

### Package Descriptions

| Package | Description |
|---------|-------------|
| **my_robot_description** | URDF/Xacro robot model with 3D meshes (DAE), launch files for RViz visualization and Gazebo simulation |
| **my_robot_controller** | ROS 2 Control configuration with `JointTrajectoryController` for the 3-joint arm |
| **my_robot_moveit** | MoveIt 2 configuration including SRDF, kinematics, joint limits, and planning pipelines |
| **my_robot_commander** | Python-based task server implementing the `Task` action for predefined motion sequences |
| **my_robot_msgs** | Custom ROS 2 action definition (`Task.action`) for task execution with feedback |

### Robot Specifications
<<<<<<< HEAD
=======

| Parameter | Value |
|-----------|-------|
| **Degrees of Freedom** | 3 (+ 2 mimic joints for end-effector orientation) |
| **Joint 1 (Base)** | Revolute, range: -120° to +120° |
| **Joint 2 (Shoulder)** | Revolute, range: -5° to +90° |
| **Joint 3 (Elbow)** | Revolute, range: -15° to +90° |
| **Control Interface** | Position control via `JointTrajectoryController` |

### Predefined Configurations

The MoveIt configuration includes named states for common arm positions:

- `home` - All joints at zero position
- `extended` - Arm extended forward
- `upright` - Arm pointing upward
- `rest` - Compact resting position

## Installation & Usage

For detailed installation instructions, dependencies, and usage guides, see the **[Wiki](https://github.com/ecuador-robotics/DOBOT_Magician_ROS/wiki)**.

### Quick Start

```bash
# Clone and build
mkdir dobot && cd dobot
git clone https://github.com/ecuador-robotics/DOBOT_Magician_ROS.git
cd DOBOT_Magician_ROS
colcon build
source install/setup.bash

# Launch RViz visualization
ros2 launch my_robot_description display.launch.xml

# Or launch Gazebo simulation
ros2 launch my_robot_description gazebo.launch.py
```

> See the [Wiki](https://github.com/ecuador-robotics/DOBOT_Magician_ROS/wiki) for MoveIt integration and Task Server setup (requires multiple terminals)

## Architecture

```
                    ┌─────────────────────┐
                    │    Task Client      │
                    └──────────┬──────────┘
                               │ Action Goal
                               ▼
                    ┌─────────────────────┐
                    │    Task Server      │
                    │  (my_robot_commander)│
                    └──────────┬──────────┘
                               │ MoveIt Py
                               ▼
                    ┌─────────────────────┐
                    │     MoveIt 2        │
                    │  Motion Planning    │
                    └──────────┬──────────┘
                               │ Trajectory
                               ▼
┌──────────────┐    ┌─────────────────────┐    ┌──────────────┐
│    RViz      │◄───│   ros2_control      │───►│   Gazebo     │
│ Visualization│    │ JointTrajectory     │    │  Simulation  │
└──────────────┘    │    Controller       │    └──────────────┘
                    └─────────────────────┘
```

## Future Development

This project is part of ongoing research into automated agricultural processing. Planned features include:

- **Computer Vision Integration** - Object detection for identifying cutting points
- **Custom End-Effector** - Combined gripping and cutting mechanism
- **Path Optimization** - Collision-aware trajectory planning for complex workspaces

## Contributing

Contributions are welcome! Please feel free to submit issues or pull requests.

For detailed setup instructions and troubleshooting, see the [Wiki](https://github.com/ecuador-robotics/DOBOT_Magician_ROS/wiki).

## License

This project is licensed under the Apache 2.0 License - see the [LICENSE](LICENSE) file for details.

## Acknowledgments

Developed by [Ecuador Robotics](https://github.com/ecuador-robotics)
>>>>>>> 736d84168740f8813ae5d4712b8b26b9090fe246

| Parameter | Value |
|-----------|-------|
| **Degrees of Freedom** | 3 (+ 2 mimic joints for end-effector orientation) |
| **Joint 1 (Base)** | Revolute, range: -120° to +120° |
| **Joint 2 (Shoulder)** | Revolute, range: -5° to +90° |
| **Joint 3 (Elbow)** | Revolute, range: -15° to +90° |
| **Control Interface** | Position control via `JointTrajectoryController` |

### Predefined Configurations

The MoveIt configuration includes named states for common arm positions:

- `home` - All joints at zero position
- `extended` - Arm extended forward
- `upright` - Arm pointing upward
- `rest` - Compact resting position

## Installation & Usage

For detailed installation instructions, dependencies, and usage guides, see the **[Wiki](https://github.com/ecuador-robotics/DOBOT_Magician_ROS/wiki)**.

### Quick Start

```bash
# Clone and build
mkdir dobot && cd dobot
git clone https://github.com/ecuador-robotics/DOBOT_Magician_ROS.git
cd DOBOT_Magician_ROS
colcon build
source install/setup.bash

# Launch RViz visualization
ros2 launch my_robot_description display.launch.xml

# Or launch Gazebo simulation
ros2 launch my_robot_description gazebo.launch.py
```

> See the [Wiki](https://github.com/ecuador-robotics/DOBOT_Magician_ROS/wiki) for MoveIt integration and Task Server setup (requires multiple terminals)

## Architecture

```
                    ┌─────────────────────┐
                    │    Task Client      │
                    └──────────┬──────────┘
                               │ Action Goal
                               ▼
                    ┌─────────────────────┐
                    │    Task Server      │
                    │  (my_robot_commander)│
                    └──────────┬──────────┘
                               │ MoveIt Py
                               ▼
                    ┌─────────────────────┐
                    │     MoveIt 2        │
                    │  Motion Planning    │
                    └──────────┬──────────┘
                               │ Trajectory
                               ▼
┌──────────────┐    ┌─────────────────────┐    ┌──────────────┐
│    RViz      │◄───│   ros2_control      │───►│   Gazebo     │
│ Visualization│    │ JointTrajectory     │    │  Simulation  │
└──────────────┘    │    Controller       │    └──────────────┘
                    └─────────────────────┘
```

## Future Development

This project is part of ongoing research into automated agricultural processing. Planned features include:

- **Computer Vision Integration** - Object detection for identifying cutting points
- **Custom End-Effector** - Combined gripping and cutting mechanism
- **Path Optimization** - Collision-aware trajectory planning for complex workspaces

## Contributing

Contributions are welcome! Please feel free to submit issues or pull requests.

For detailed setup instructions and troubleshooting, see the [Wiki](https://github.com/ecuador-robotics/DOBOT_Magician_ROS/wiki).

## License

This project is licensed under the Apache 2.0 License - see the [LICENSE](LICENSE) file for details.

## Acknowledgments

Developed by [Ecuador Robotics](https://github.com/ecuador-robotics)
