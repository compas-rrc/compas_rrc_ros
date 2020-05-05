# Container for running COMPAS RRC Driver
#
# Build:
#  docker build --rm -f Dockerfile -t gramaziokohler/compas_rrc_driver .
#
# Usage outside DockerHub:
#  docker save -o compas_rrc_driver-latest.tar gramaziokohler/compas_rrc_driver:latest
#  docker load -i compas_rrc_driver-latest.tar
#
# Usage:
#  docker pull gramaziokohler/compas_rrc_driver

FROM ros:kinetic-ros-core
LABEL maintainer "Gonzalo Casas <casas@arch.ethz.ch>"

SHELL ["/bin/bash","-c"]

RUN apt-key adv --keyserver keyserver.ubuntu.com --recv-keys F42ED6FBAB17C654

# Install packages
RUN apt-get update && apt-get install -y \
    # Basic utilities
    iputils-ping \
    # ROS bridge server and related packages
    ros-${ROS_DISTRO}-rosbridge-server \
    ros-${ROS_DISTRO}-tf2-web-republisher \
    --no-install-recommends \
    # Clear apt-cache to reduce image size
    && rm -rf /var/lib/apt/lists/*

# Create local catkin workspace
ENV CATKIN_WS=/root/catkin_ws
# Add COMPAS RRC Driver package
ADD . $CATKIN_WS/src/compas_rrc_driver
WORKDIR $CATKIN_WS/src

RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    # Update apt-get because its cache is always cleared after installs to keep image size down
    && apt-get update \
    # Install dependencies
    && cd $CATKIN_WS \
    && rosdep install -y --from-paths . --ignore-src --rosdistro ${ROS_DISTRO} \
    # Build catkin workspace
    && ROS_LANG_DISABLE=geneus:genlisp:gennodejs catkin_make

COPY ./.docker/ros_catkin_entrypoint.sh /

ENTRYPOINT ["/ros_catkin_entrypoint.sh"]
CMD ["bash"]
