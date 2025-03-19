# Cybership Dynamic Positioning

This package provides dynamic positioning (DP) control capabilities for Cybership vessels, enabling automated position and velocity control for maritime autonomous systems.

## Overview

The `cybership_dp` package implements control algorithms for dynamic positioning of marine vessels. It provides two primary controllers:

1. **Force Controller**: Allocates thrust commands to vessel actuators based on desired force/torque inputs
2. **Velocity Controller**: Generates force/torque commands based on desired vessel velocity

Both controllers are implemented as ROS 2 nodes and can be configured for different vessel models.

## Structure

```
cybership_dp/
├── CMakeLists.txt            # Build configuration
├── package.xml               # Package metadata and dependencies
├── requirements.txt          # Python dependencies
├── config/                   # Controller configurations
│   ├── force_controller.yaml
│   ├── velocity_controller.yaml
│   └── [vessel_models]/      # Vessel-specific configs
├── cybership_dp/             # Python module
│   ├── __init__.py
│   ├── drillship/            # Drillship-specific implementations
│   ├── enterprise/           # Enterprise-specific implementations
│   └── voyager/              # Voyager-specific implementations
├── launch/                   # Launch files
│   ├── dp.launch.py          # Combined DP system launch
│   ├── force_controller.launch.py
│   └── velocity_controller.launch.py
└── nodes/                    # ROS executable nodes
    ├── force_controller.py
    └── velocity_controller.py
```

## Usage

### Launch Files

The package provides multiple launch files:

- **dp.launch.py**: Launches both force and velocity controllers
- **force_controller.launch.py**: Launches only the force controller
- **velocity_controller.launch.py**: Launches only the velocity controller

### Example Usage

To launch the complete DP system for a specific vessel:

```bash
ros2 launch cybership_dp dp.launch.py vessel_name:=voyager vessel_model:=voyager
```

To launch only the force controller:

```bash
ros2 launch cybership_dp force_controller.launch.py vessel_name:=voyager vessel_model:=voyager
```

### Parameters

Both controllers use configuration parameters specified in YAML files in the `config/` directory. Default configuration files are provided, but vessel-specific configurations can be created in the corresponding subdirectories.

Key parameters include:
- PID gains for velocity controller (surge, sway, yaw)
- Thruster configuration for force allocation
- Control frequency

## Features

### Force Controller

The force controller uses the [Skadi](https://github.com/incebellipipo/skadipy) library for optimal thrust allocation. It:

1. Receives desired forces/torques in 3DOF (surge, sway, yaw)
2. Optimally allocates these demands to available thrusters
3. Publishes thrust commands to actuators

### Velocity Controller

The velocity controller:

1. Receives desired velocities
2. Computes control errors
3. Applies PID control to generate force/torque commands
4. Publishes these commands to the force controller

## Dependencies

- ROS 2
- `geometry_msgs` - For standard ROS geometry message types
- `nav_msgs` - For odometry messages
- `cybership_utilities` - Cybership common utilities
- `skadipy` - Thrust allocation library

## Vessel Models

Currently, the package primarily supports the `voyager` vessel model with implementations for:
- Force controller with thrust allocation
- Velocity controller with PID control

Support for additional vessel models (`enterprise` and `drillship`) is planned but not yet fully implemented.