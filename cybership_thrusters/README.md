# Cybership Thrusters

This package provides ROS2 nodes for controlling the Cybership's thrusters using different controller types. The implementation supports various thruster models such as Voith Schneider, Fixed, and Azimuth. The package uses the ROS2 parameter system to configure and launch thruster controllers dynamically based on external YAML configuration inputs.

## Overview

The package offers:
- A base class `ThrusterBase` for common functionality.
- Three types of thrusters:
  - **Voith Schneider (vsp):** Provides force control with differential arm outputs and a fixed RPM command.
  - **Fixed:** Provides a single output signal based on the input force.
  - **Azimuth:** Computes rotational angle and RPM from the input force vector.
- A ROS2 node (executable) that reads the thruster configuration parameters and instantiates the corresponding controllers. See `cybership_thrusters_node.cpp`.

## Services

> [!IMPORTANT]
> Thrusters are disabled by default. To enable them, you need to call `~/thruster/enable` service.

Thrusters are enabled using `~/thruster/enable` and disabled using `~/thruster/disable`.
These services are `std_srvs/srv/Empty` services that can be called to enable or disable the thrusters.

To enable
```bash
ros2 service call <vessel_name>/thruster/enable std_srvs/srv/Empty {}
```

To disable
```bash
ros2 service call <vessel_name>/thruster/disable std_srvs/srv/Empty {}
```


## Parameters and Their Definitions

The ROS2 parameters for configuring each thruster are defined under the `thrusters` namespace. Each thruster is represented by a unique name with its parameters specified as follows:

### Common Parameters (for all thruster types)
- **`type`**:
  Defines the thruster controller type. Valid options are:
  - `"vsp"` (Voith Schneider)
  - `"fixed"`
  - `"azimuth"`

- **`force_topic`**:
  The topic name on which the thruster receives the input force as a `geometry_msgs/msg/Wrench`.

- **`force_max`** and **`force_min`**:
  The maximum and minimum limits for the input force. These values are used to clamp the received force values so that the output remains within the expected range.

### Additional Parameters for Voith Schneider (`vsp`)
- **`arm_x_topic`** and **`arm_y_topic`**:
  The topics on which the computed arm positions (or signals) are published. These outputs are derived from the clamped force values.

- **`rpm_topic`**:
  The topic on which the fixed RPM command is published.

- **`rpm_cmd`**:
  The fixed RPM command value that is output when the thruster is active.

- **`arm_x_inverted`** and **`arm_y_inverted`**:
  Boolean flags indicating whether the computed arm output signals should be inverted.

### Additional Parameters for Azimuth
- **`angle_topic`**:
  The topic on which the computed angular command (using `atan2`) is published.

- **`rpm_topic`**:
  The topic on which the computed RPM (as the magnitude of the force vector) is published.

- **`angle_inverted`** and **`rpm_inverted`**:
  Boolean flags to invert the computed angle and RPM outputs respectively, if needed.

### Additional Parameters for Fixed
- **`signal_topic`**:
  The topic on which the computed signal (derived via linear interpolation of the force input) is published.

- **`signal_inverted`**:
  A boolean flag indicating whether the output signal should be inverted.

## Usage

The node will read parameters from the ROS2 parameter server and create instances of thrusters based on their configurations.

### Example YAML Configuration

You can configure the thrusters by setting parameters under the `thrusters` namespace. Here's an example configuration in YAML:

```yaml
thrusters:
  thruster1:
    type: "vsp"                   # Valid options: vsp, fixed, azimuth
    force_topic: "/thruster1/force"
    arm_x_topic: "/thruster1/arm_x"
    arm_y_topic: "/thruster1/arm_y"
    rpm_topic: "/thruster1/rpm"
    force_max: 10.0
    force_min: -10.0
    rpm_cmd: 1000.0
    arm_x_inverted: false
    arm_y_inverted: false

  thruster2:
    type: "azimuth"               # Valid options: vsp, fixed, azimuth
    force_topic: "/thruster2/force"
    angle_topic: "/thruster2/angle"
    rpm_topic: "/thruster2/rpm"
    force_max: 20.0
    force_min: -20.0
    rpm_inverted: false
    angle_inverted: false

  thruster3:
    type: "fixed"                 # Valid options: vsp, fixed, azimuth
    force_topic: "/thruster3/force"
    signal_topic: "/thruster3/signal"
    force_max: 5.0
    force_min: -5.0
    signal_inverted: true
```

You can load this configuration via a YAML file and pass it as an argument to your node or use ROS2’s parameter mechanisms to set these values dynamically.

## Additional Information

- Build configuration is found in `CMakeLists.txt`.
- The package metadata is defined in `package.xml`.
- For further details, refer to the individual thruster implementation files:
  - **VoithSchneider:** `voith_schneider.cpp`
  - **Fixed:** `fixed.cpp`
  - **Azimuth:** `azimuth.cpp`
