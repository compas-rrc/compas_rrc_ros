# COMPAS RRC: ROS driver

> ROS package for the COMPAS RRC driver for ABB robots.

## Usage

The easiest option to use the ROS package is via Docker:

* Download the image file and `docker-compose.yml` files to your computer
* Load the image in docker:

      docker load -i compas_rrc_driver-latest.tar

* To use it with a virtual controller (ABB RobotStudio) on the same computer, no further changes are needed.
* To use it with a real robot, update the robot's IP address in `docker-compose.yml` file (e.g. `robot_ip:=192.168.0.100`)
* Run docker compose up:

      docker-compose up

* Start the robot controller.
