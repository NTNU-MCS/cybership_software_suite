# Cybership Bringup

This package is part of the Cybership Software Suite and provides launch files for bringing up the Cybership system in a ROS 2 environment. It supports various vessel types (e.g., drillship, enterprise, voyager) and includes configurations for localization, motion capture, and hardware control nodes.

## Contents

- **Launch Files** in the `launch/` directory:
- [`drillship.launch.py`](./launch/drillship.launch.py)
- [`enterprise.launch.py`](./launch/enterprise.launch.py)
- [`localization.launch.py`](./launch/localization.launch.py)
- [`voyager.launch.py`](./launch/voyager.launch.py)

## Prerequisites

- ROS 2 Foxy (or later)
- Required dependencies:
  - `rclpy`
  - `robot_localization`
- Other dependencies as specified by the included launch files.

## Usage
To launch the bringup for a specific vessel, use one of the provided launch files. For example, to launch the voyager bringup:
```bash
ros2 launch cybership_bringup [voyager.launch.py](./launch/voyager.launch.py)
```

For enterprise
```bash
ros2 launch cybership_bringup [enterprise.launch.py](./launch/enterprise.launch.py)
```
For drillship
```bash
ros2 launch cybership_bringup [drillship.launch.py](./launch/drillship.launch.py)
```

This will start the necessary nodes and configurations for the vessels.

## Launch Includes

- [azimuth_controller.launch.py`](./launch/include/azimuth_controller.launch.py): Launches the azimuth controller node.
  This node is responsible for controlling the azimuth angles of the vessel.
- [imu_bno055.launch.py`](./launch/include/imu_bno055.launch.py): Launches the BNO055 IMU node.
    This node handles the data from the BNO055 IMU sensor.
- [motion_capture_system_connector.launch.py`](./launch/include/motion_capture_system_connector.launch.py): Launches the motion capture system connector node.
    This node connects to the motion capture system and retrieves position data.
- [motion_capture_system_transformer.launch.py`](./launch/include/motion_capture_system_transformer.launch.py): Launches the motion capture system transformer node.
    This node transforms the motion capture data into the appropriate coordinate frame.
- [robot_localization.launch.py`](./launch/include/robot_localization.launch.py): Launches the robot localization node.
    This node fuses data from various sensors to provide an accurate estimate of the vessel's position and orientation.
- [servo_driver.launch.py`](./launch/include/servo_driver.launch.py): Launches the servo driver node.
    This node controls the servos on the vessel.
- [thruster_control.launch.py`](./launch/include/thruster_control.launch.py): Launches the thruster control node.
    This node controls the thrusters on the vessel.
- [urdf_description.launch.py`](./launch/include/urdf_description.launch.py): Launches the URDF description node.
    This node provides the URDF model of the vessel to other nodes.
- [localization.launch.py`](./launch/localization.launch.py): Launches the localization system for the vessel.
    This includes the robot localization node and any other necessary components for localization.

