# External Dependencies for CyberShip Vessels

This directory contains external dependencies used by the CyberShip software suite. These dependencies are maintained as Git submodules for easier version control and updates.

## Included Dependencies

### mocap4ros2_qualisys
- **Description**: ROS2 driver for Qualisys motion capture systems
- **Repository**: https://github.com/incebellipipo/mocap4ros2_qualisys

### dynamixel_azimuth_driver
- **Description**: ROS2 driver for Dynamixel motors used for azimuth control
- **Repository**: https://github.com/incebellipipo/dynamixel_azimuth_driver.git

### ros2_pca9685
- **Description**: ROS2 driver for PCA9685 PWM controller board
- **Repository**: https://github.com/incebellipipo/ros2_pca9685.git

### mocap4r2_msgs
- **Description**: ROS2 message definitions for motion capture systems
- **Repository**: https://github.com/MOCAP4ROS2-Project/mocap4r2_msgs.git

### bno055
- **Description**: ROS2 driver for BNO055 IMU sensor
- **Repository**: https://github.com/flynneva/bno055.git

## Updating Submodules

To update all submodules to their latest versions:

```bash
git submodule update --remote
```

To initialize submodules after cloning the main repository:

```bash
git submodule init
git submodule update
```

