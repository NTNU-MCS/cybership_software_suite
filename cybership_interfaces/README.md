# Cybership Interfaces

This ROS 2 package provides common interface definitions (messages and services) for Cybership vessels. These interfaces are used across the Cybership software suite to ensure standardized communication between different nodes and components.

## Overview

The `cybership_interfaces` package defines custom messages and services that are specific to maritime autonomous systems, particularly for the Cybership vessels. These interfaces extend standard ROS 2 messages with domain-specific data structures needed for maritime operations.

## Contents

### Services

- **ResetSimulator.srv**: Service to reset the simulator with a specified 2D pose
  - **Request**: `geometry_msgs/Pose2D` - Target pose for reset
  - **Response**: `bool success` - Indicates if reset was successful

### Messages

*Currently no custom messages defined. More will be added as needed.*

## Dependencies

- `geometry_msgs`: Standard ROS 2 geometry message definitions
- `rosidl_default_generators`: ROS 2 interface generation tools
- `rosidl_default_runtime`: Runtime dependencies for ROS 2 interfaces

## Usage

To use these interfaces in your package:

1. Add a dependency to your `package.xml`:
   ```xml
   <depend>cybership_interfaces</depend>
   ```

2. In your CMakeLists.txt, add:
   ```cmake
   find_package(cybership_interfaces REQUIRED)
   ```

3. In your C++ code:
   ```cpp
   #include "cybership_interfaces/srv/reset_simulator.hpp"
   ```

4. Or in Python:
   ```python
   from cybership_interfaces.srv import ResetSimulator
   ```