# Cybership Motion Capture System

This package provides ROS 2 integration for the Cybership with a motion capture system. It subscribes to motion capture data and publishes the ship's pose information for navigation and control purposes.

## Overview

The `cybership_mocap` package connects to an external motion capture system (using the mocap4r2 interface) to track the position and orientation of the Cybership vessel. It processes incoming rigid body data from the motion capture system and republishes it in formats suitable for robotic navigation and visualization.

## Features

- Subscribes to motion capture rigid body data
- Publishes pose information as `geometry_msgs::msg::PoseWithCovarianceStamped`
- Optional TF transform broadcasting
- Configurable frame IDs and topic names

## Installation

### Dependencies

- ROS 2
- tf2 & tf2_ros
- sensor_msgs, std_msgs, geometry_msgs
- mocap4r2_msgs
- nav_msgs

### Building

Clone the repository into your ROS 2 workspace and build:

```bash
cd ~/ros2_ws/src
git clone <repository-url>/cybership_software_suite.git
cd ~/ros2_ws
colcon build --packages-select cybership_mocap
```

## Usage

Launch the node with the provided launch file:

```bash
ros2 launch cybership_mocap cybership_mocap.launch.py
```

## Configuration

The node behavior can be configured via parameters in the `config.yaml` file:

| Parameter | Description | Default |
|-----------|-------------|---------|
| `world_frame` | The name of the world/global reference frame | "mocap" |
| `base_frame` | The name of the robot's base frame | "cybership/base_link" |
| `rigid_body_name` | The name of the rigid body in the mocap system | "cybership" |
| `topic_name` | Topic name to which the pose will be published | "cybership/pose" |
| `publish_tf` | Whether to publish the transform to TF | true |

## Node Details

### Subscribed Topics

- `rigid_bodies` ([mocap4r2_msgs/msg/RigidBodies](https://github.com/MOCAP4ROS2-Project/mocap4r2_msgs)) - Motion capture data from the mocap system

### Published Topics

- `cybership/pose` ([geometry_msgs/msg/PoseWithCovarianceStamped](http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/PoseWithCovarianceStamped.html)) - Ship pose information

### Published Transforms (if enabled)

- `mocap` → `cybership/base_link` - Transform from the world frame to the ship's base frame

