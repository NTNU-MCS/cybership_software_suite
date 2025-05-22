ARG ROS_DISTRO=jazzy
ARG UID=1000
ARG GID=1000

FROM ros:$ROS_DISTRO

ARG ROS_DISTRO
ARG UID
ARG GID

ENV ROS_DISTRO=${ROS_DISTRO}
ENV DEBIAN_FRONTEND=noninteractive
ENV DEBIAN_FRONTEND=teletype

RUN apt-get update && apt-get upgrade -y && apt-get install -y sudo

RUN uid=${UID} && \
    if getent passwd $uid > /dev/null; then \
    username=$(getent passwd $uid | cut -d: -f1); \
    userdel -r $username; \
    fi

RUN groupadd --gid "${GID}" ros_user \
    && useradd --uid "${UID}" --gid "${GID}" -m ros_user \
    && usermod -aG sudo ros_user \
    && echo "ros_user ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

RUN mkdir -p /ros/src

WORKDIR /ros

COPY . src/cybership_common
COPY .git/ src/cybership_common/.git/

RUN chown -R ros_user:ros_user /ros

USER ros_user

RUN sudo apt-get update && \
    sudo apt-get install -y git ros-dev-tools && \
    sudo apt-cache search ${ROS_DISTRO}-rmw \
    | grep -v -E "dbgsym|connextdds" \
    | awk '{print $1}' \
    | xargs sudo apt-get install -y

RUN cd src/cybership_common && git submodule update --init --recursive

RUN rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y

RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.sh && colcon build"

SHELL ["/bin/bash"]

ENTRYPOINT ["/bin/bash", "-c", "source /ros/install/setup.sh && exec \"${@}\"", "--"]
CMD ["bash"]
