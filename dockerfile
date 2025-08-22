ARG ROS_DISTRO=jazzy
ARG UID=1000
ARG GID=1000

FROM ros:${ROS_DISTRO}-ros-base

ARG ROS_DISTRO
ARG UID
ARG GID

ENV ROS_DISTRO=${ROS_DISTRO}
ENV DEBIAN_FRONTEND=noninteractive
ENV DEBIAN_FRONTEND=teletype

RUN sudo apt-get update && \
    sudo apt-get install -y git ros-dev-tools python3-venv python3-pip && \
    rosdep update

RUN \
    # Remove existing user with the specified UID if it exists
    uid=${UID} && \
    if getent passwd $uid > /dev/null; then \
    username=$(getent passwd $uid | cut -d: -f1); \
    userdel -r $username; \
    fi \
    && \
    # Add a new user with the specified UID and GID
    groupadd --gid "${GID}" ros_user \
    && useradd --uid "${UID}" --gid "${GID}" -m ros_user \
    && usermod -aG sudo ros_user \
    && echo "ros_user ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

RUN mkdir -p /ros/src && chown -R ros_user:ros_user /ros

COPY . /ros/src/cybership
COPY .git/ /ros/src/cybership/.git/

USER ros_user

RUN sudo chown -R ros_user:ros_user /ros && \
    git -C /ros/src/cybership submodule update --init --recursive  && \
    rosdep update && \
    rosdep install --from-paths /ros/src --ignore-src -r -y

RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    python3 -m venv /ros/venv --system-site-packages --symlinks && \
    touch /ros/venv/COLCON_IGNORE && \
    . /ros/venv/bin/activate && \
    find /ros/src -name "requirements*txt" -exec pip install -r {} \;

RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.sh && cd /ros && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    echo 'source /ros/install/setup.sh' >> /home/ros_user/.bashrc && \
    echo 'source /ros/venv/bin/activate' >> /home/ros_user/.bashrc"

SHELL ["/bin/bash"]
ENTRYPOINT ["/bin/bash", "-c", "source /ros/install/setup.sh && source /ros/venv/bin/activate && exec \"${@}\"", "--"]
CMD ["bash"]
