# COMPAS RRC: ROS driver

![COMPAS RRC](images/compas_rrc.png)

> ROS package for the COMPAS RRC driver for ABB robots.

## Usage

### Docker

The easiest option to use the ROS package is via Docker:

* Download the image file and `docker-compose.yml` files to your computer
* To use it with a virtual controller (ABB RobotStudio) on the same computer, no further changes are needed.
* To use it with a real robot, update the robot's IP address in `docker-compose.yml` file (e.g. `robot_ip:=192.168.0.100`)
* To use it in other scenarios, check the [examples](examples) folder.
* Run docker compose up:

      $ docker-compose up

* Start the robot controller(s).

### Linux

If you prefer to use a ROS installation on Linux:

* On the terminal, change to your catkin workspace source folder:

      $ cd ~/catkin_ws/src/

* Clone this repository:

      $ git clone https://github.com/compas-rrc/compas_rrc_ros.git

* Build your workspace (e.g. using `catkin_make`):

      $ cd ~/catkin_ws
      $ catkin_make

* Source your workspace:

      $ source ~/catkin_ws/devel/setup.bash

* Launch the driver using one of the provided launch files, e.g.:

      $ roslaunch compas_rrc_driver bringup.launch robot_ip:=127.0.0.1 robot_streaming_port:=30101 robot_state_port:=30201

## Launch file parameters

The launch files have the following parameters available:

* `robot_ip`: [*mandatory*] IP address of the robot.
* `robot_streaming_port`: [*optional*, `default=30101`] TCP port of the streaming interface of the robot.
* `robot_state_port`: [*optional*, `default=30201`] TCP port of the state interface of the robot.
* `sequence_check_mode`: [*optional*, `default=none`] Sequence check mode, valid options are: `none`, `all`, `incoming`, `outgoing`
* `namespace`: [*optional*, `default='/'`] Allows to define a namespace in order to connect to multiple robots on the same instance. Defaults to root (`/`).

## License

This project is licensed under the terms of the [MIT License](/LICENSE).
