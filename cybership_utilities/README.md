# Cybership Utilities

The **Cybership Utilities** package is part of the CyberShip Software Suite. It provides a set of utility functions and common launch arguments to help integrate and run CyberShip applications within a ROS 2 environment.

## Overview

This package offers:

- **Utility constants:** Predefined vessel model names such as `voyager`, `enterprise`, `drillship`. See [`cybership_utilities.utilities`](./cybership_utilities/utilities.py) for details.
- **Helper functions:**
  - `sanitize_hostname_for_ros2`: Sanitizes hostnames to generate valid ROS 2 node names.
  - `anon`: Generates an anonymous ROS 2 node name using the system hostname and a UUID.
- **Common launch arguments:** A predefined list of launch arguments (e.g., `vessel_model`, `vessel_name`, `param_file`, `use_sim_time`) for use in ROS 2 launch files. See [`cybership_utilities.launch.py`](./cybership_utilities/launch.py).

## Usage

You can use the provided utility functions and launch arguments in your ROS 2 launch files. For example, import the functions from the launch script:

```python
from cybership_utilities import launch
```

Then use `sanitize_hostname_for_ros2` or `anon` in your launch configurations, and include `COMMON_ARGUMENTS` to standardize launch argument handling.

### Force Mux GUI

The script `nodes/force_mux_gui.py` provides a small PyQt GUI to inspect and switch the `topic_tools` force mux source for a selected vehicle namespace.

- Lists available input topics via `MuxList`
- Switches active topic via `MuxSelect`
- Namespace can be typed (e.g., `/enterprise`) or set with `VEHICLE_NS` env var.

Run it with your preferred Python, ensuring your ROS 2 environment is sourced so service types are discoverable.