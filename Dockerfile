# Container for running COMPAS RRC Driver
#
# Build:
#  docker build --rm -f Dockerfile -t compasrrc/compas_rrc_driver .
#
# Usage:
#  docker pull compasrrc/compas_rrc_driver

FROM ros:jazzy-ros-core
LABEL maintainer="RRC Team <rrc@arch.ethz.ch>"

SHELL ["/bin/bash","-c"]

# Install packages
RUN apt-get update && apt-get install -y \
    iputils-ping \
    ros-${ROS_DISTRO}-rosbridge-server \
    --no-install-recommends \
    # Clear apt-cache to reduce image size
    && rm -rf /var/lib/apt/lists/*

# Build number
ENV RRC_BUILD=1

# Create local catkin workspace
ENV WS=/root/ws
# Add COMPAS RRC Driver package
ADD . $WS/src/compas_rrc_driver
WORKDIR $WS/src

RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    # Update apt-get because its cache is always cleared after installs to keep image size down
    && apt-get update \
    && apt-get install ros-dev-tools -y  \
    # Reconfigure rosdep
    && rm -rf /etc/ros/rosdep/sources.list.d/* \
    && rosdep init && rosdep update \
    # Install dependencies
    && cd $WS \
    && rosdep install -y --from-paths . --ignore-src --rosdistro ${ROS_DISTRO} \
    # Build catkin workspace
    && colcon build \
    # Clear apt-cache to reduce image size
    && rm -rf /var/lib/apt/lists/*

COPY ./.docker/ros_entrypoint.sh /
RUN chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
