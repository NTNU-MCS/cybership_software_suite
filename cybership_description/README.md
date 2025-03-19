# Cybership Description
> URDF models and coordinate frame transformations for CyberShip vessels

## Overview

The `cybership_description` package provides the 3D models, URDF definitions, and coordinate frame transformations necessary for CyberShip vessels. These descriptions are used throughout the CyberShip software suite for visualization, simulation, and physical operation of the vessels.

## Contents

- **urdf/**: URDF files that define the physical structure and coordinate frames of CyberShip vessels
- **meshes/**: 3D model files for visualization of vessel components
- **launch/**: ROS 2 launch files for visualizing and publishing robot state

## Dependencies

This package depends on:
- `tf2_ros`: For coordinate frame transformations
- `robot_state_publisher`: For publishing the URDF state to TF
- `xacro`: For URDF preprocessing
- `urdf`: For URDF parsing

## Usage

### Visualizing the Model

To launch the robot state publisher with the CyberShip model:

```bash
ros2 launch cybership_description description.launch.py
```

