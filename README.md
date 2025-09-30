# cybership_common

This repository is the home of the CyberShip Software Suite - a collection of ROS 2 packages that together provide the tools to simulate, visualize, and control autonomous maritime vessels. The suite covers everything from the digital twin (URDF models and visualization) to real-time sensor integration (IMU, motion capture, etc.) and advanced control (dynamic positioning and thrust allocation).

## What's Included

- **cybership_bringup**: Launch files to initialize all the necessary nodes for bringing up a vessel in a ROS 2 environment. This includes hardware drivers, localization, and sensor integration.
- **cybership_simulator**: Simple Python scripts for vessel simulation using basic physics. Use these scripts to test vessel behavior in simulation.
- **cybership_viz**: Nodes and configurations to visualize the vessel in RViz. This package provides a visual representation of the vessel using its URDF models.
- **cybership_description**: URDF models and coordinate frame definitions for accurate vessel representation.
- **cybership_config**: Centralized configuration files that are shared and used by various components in the suite.
- **cybership_interfaces**: Common message and service definitions to standardize communication across the suite.
- **Other modules**: Additional packages for utilities, thruster control, dynamic positioning, and sensor drivers like BNO055 and motion capture (mocap).

## How to Get Started

### 1. Installation

Follow these steps to set up your ROS 2 workspace with the CyberShip Software Suite:
 > [!IMPORTANT]
 > Make sure that you have sourced the ROS 2 installation
 > ```bash
 > source /opt/ros/<distro>/setup.bash
 > ```
 > Replace `<distro>` with your ROS 2 distribution (e.g., `humble`, `jazzy`, etc.).

1. **Clone the Repository and Submodules**
   ```bash
   cd $ROS_WORKSPACE/src
   git clone https://github.com/NTNU-MCS/cybership_software_suite
   cd cybership_software_suite
   git submodule update --init --recursive

2. Set Up a Python Virtual Environment
    ```bash
    cd $ROS_WORKSPACE
    python3 -m venv venv --system-site-packages --symlinks
    source venv/bin/activate
    touch venv/COLCON_IGNORE
    ```

3.  Install Python Dependencies
    ```bash
    cd $ROS_WORKSPACE
    source venv/bin/activate
    find src/cybership_software_suite -name "requirements*txt" -exec pip install -r {} \;
    ```

4. Install ROS Dependencies and Build
    ```bash
    cd $ROS_WORKSPACE
    rosdep install --from-paths src -i
    colcon build --symlink-install
    ```
 > [!WARNING]
 > Make sure that you have initialized rosdep and sourced the ROS 2 environment before running the above command. You can do this by running:
 > ```bash
 > source /opt/ros/<distro>/setup.bash
 > sudo rosdep init
 > rosdep update
 > ```
 > Replace `<distro>` with your ROS 2 distribution (e.g., `humble`, `jazzy`, etc.).
 > If you encounter any issues with dependencies, please refer to the documentation of the specific package for additional instructions.

5. Source the Environment

    ```bash
    source $ROS_WORKSPACE/venv/bin/activate
    source $ROS_WORKSPACE/install/setup.bash
    ```

### 2. Bringing Up a Vessel

There are two main modes to bring up a vessel: physical and simulation.

For a Physical Deployment:
Use the cybership_bringup package. For example, to bring up the Voyager vessel, run:
```bash
ros2 launch cybership_bringup voyager.launch.py vessel_name:=voyager vessel_model:=voyager
```
You can similarly launch other vessel types like Enterprise or Drillship by choosing the corresponding launch file in cybership_bringup.

For Simulation:
Use the cybership_simulator package. This runs simplified physics scripts:

### 3. Visualization
Visualize your vessel using the RViz visualization tools provided in the suite. The cybership_viz package sets up RViz with a preconfigured scene showing your vessel's URDF model and sensor data overlays. To launch visualization:

Additionally, the cybership_description package can publish the URDF model via:

### Additional Notes
Dynamic Positioning:
The cybership_dp package provides controllers for dynamic positioning. Launch the complete DP system or individual controllers as needed.

### Sensor Integration & Drivers:
The suite integrates various sensors such as the BNO055 IMU and motion capture systems. Please refer to corresponding packages (e.g., cybership_external/bno055 and cybership_mocap) for detailed instructions.

### Utilities and Interfaces:
Use the utility functions in cybership_utilities for standardized launch arguments and helper functions across your launch files. Common interfaces are provided in cybership_interfaces.

## Docker

```bash
docker compose --profile voyager up
```
