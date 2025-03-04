# Set default values for build arguments
ARG ROS_DISTRO=humble
ARG RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Use the specified ROS distribution as the base image
FROM ros:$ROS_DISTRO

SHELL [ "/bin/bash" ]

# Set environment variables for ROS distribution and vessel model
ARG ROS_DISTRO
ENV ROS_DISTRO=${ROS_DISTRO}

ARG RMW_IMPLEMENTATION
ENV RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}

# Create the directory for ROS workspace
RUN mkdir -p /ros/src

# Set the working directory
WORKDIR /ros

# Copy the source code into the container
COPY . src/cybership_common

# Copy the .git directory to handle submodules
COPY .git/ src/cybership_common/.git/

# Install necessary packages and initialize git submodules
RUN \
    cd src && \
    apt-get update && \
    apt-get install -y git ros-dev-tools && \
    cd cybership_common && \
    git submodule update --init --recursive

# Install ROS dependencies
RUN \
    rosdep install --from-paths src --ignore-src -r -y

    # Install all RVM implementations
RUN \
    apt-get install -y \
        ros-${ROS_DISTRO}-rmw-opensplice-cpp && \
        ros-${ROS_DISTRO}-rmw-connext-cpp && \
        ros-${ROS_DISTRO}-rmw-fastrtps-cpp

# Build the ROS workspace
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.sh && colcon build"

# Set the entry point to your launch file
ENTRYPOINT ["/bin/bash", "-c", "source /ros/install/setup.sh"]
