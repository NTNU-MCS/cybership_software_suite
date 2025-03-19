# Cybership Config Package

The `cybership_config` package is part of the Cybership Software Suite, providing configuration files for various Cybership vessel components. This package centralizes configuration settings to make it easier to manage parameters across the entire system.

## Contents

This package contains:

- Configuration files for Cybership vessels and components
- Installation instructions for making configurations available to other packages

## Dependencies

The package has minimal dependencies:
- `ament_cmake` (build dependency)

## Structure

The package uses a simple structure:
```
cybership_config/
├── CMakeLists.txt       # Build configuration
├── LICENSE              # GPL-3.0 license file
├── package.xml          # Package metadata
├── README.md            # This file
└── config/              # Configuration files directory
    └── [various config files]
```