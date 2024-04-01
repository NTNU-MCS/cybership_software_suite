# cybership_common

## Installation

Clone the repositor under ROS workspace
```bash
cd $ROS_WORKSPACE/src
git clone https://github.com/incebellipipo/cybership_common
```
Get all the submodules
```bash
cd $ROS_WORKSPACE/src/cybership_common
git submodule update --init --recursive
```
Install dependencies
```bash
cd $ROS_WORKSPACE
rosdep install --from-paths src -i
```
Build the workspace
```bash
cd $ROS_WORKSPACE
colcon build --symlink-install
```