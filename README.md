# Capstone Project

This is the project repo for the final project of the Udacity Self-Driving Car Nanodegree: Programming a Real Self-Driving Car. For more information about the project, see the project introduction [here](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/e1a23b06-329a-4684-a717-ad476f0d8dff/lessons/462c933d-9f24-42d3-8bdc-a08a5fc866e4/concepts/5ab4b122-83e6-436d-850f-9f4d26627fd9).

## Introduction
The team designed an autonomous car that will be tested on the simulator and, then, on Udacity’s real self-driving car (Carla). As introduced in the Udacity walkthrough videos, the project is organized in three parts:
- the Waypoint Updater;
- the Drive-By-Wire (DBW);
- the Traffic Light Detection.

![overview](./imgs/final-project-ros-graph-v2.png "")

## Waypoint Updater
This node is implemented in the [wayppoint_updater.py](/ros/src/waypoint_updater/waypoint_updater.py) file.

![wayppoint_updater](./imgs/waypoint-updater-ros-graph.png "")


## Drive-By-Wire (DBW)
This node is implemented in the [dbw_node.py](/ros/src/twist_controller/dbw_node.py) file.
It's subscribed to the `current_vel`,`twist_cmd` and `dbw_enabled` topics and it publishes the `throttle_cmd`, `brake_cmd` and `steering_cmd` topics.

![dbw](./imgs/dbw-node-ros-graph.png "")

### Steering
Predictive Steering is implemented using the provided `YawController` class ([yaw_controller.py](/ros/src/twist_controller/yaw_controller.py)).

### Throttle
Throttle is controlled by a linear PID by passing in the velocity error(difference between the current velocity and the proposed velocity)

### Brake
When the PID controller returns a negative value for throttle, it means the car needs to decelerate. The amount of deceleration needed is calculated in these two lines of code:
```
decel = max(vel_error, self.decel_limit)
brake = abs(decel) * self.vehicle_mass * self.wheel_radius # Torque N*m
```
`brake` is the amount of torque that is applied to the brake system to decrease the car's speed.

## Traffic Light Detection
This node is implemented in the [tl_detector.py](/ros/src/tl_detection/tl_detector.py) file.

![dbw](./imgs/tl-detector-ros-graph.png "")



## Setup

Please use **one** of the two installation options, either native **or** docker installation.

### Native Installation

* Be sure that your workstation is running Ubuntu 16.04 Xenial Xerus or Ubuntu 14.04 Trusty Tahir. [Ubuntu downloads can be found here](https://www.ubuntu.com/download/desktop).
* If using a Virtual Machine to install Ubuntu, use the following configuration as minimum:
  * 2 CPU
  * 2 GB system memory
  * 25 GB of free hard drive space

  The Udacity provided virtual machine has ROS and Dataspeed DBW already installed, so you can skip the next two steps if you are using this.

* Follow these instructions to install ROS
  * [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) if you have Ubuntu 16.04.
  * [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu) if you have Ubuntu 14.04.
* [Dataspeed DBW](https://bitbucket.org/DataspeedInc/dbw_mkz_ros)
  * Use this option to install the SDK on a workstation that already has ROS installed: [One Line SDK Install (binary)](https://bitbucket.org/DataspeedInc/dbw_mkz_ros/src/81e63fcc335d7b64139d7482017d6a97b405e250/ROS_SETUP.md?fileviewer=file-view-default)
* Download the [Udacity Simulator](https://github.com/udacity/CarND-Capstone/releases).

### Docker Installation
[Install Docker](https://docs.docker.com/engine/installation/)

Build the docker container
```bash
docker build . -t capstone
```

Run the docker file
```bash
docker run -p 4567:4567 -v $PWD:/capstone -v /tmp/log:/root/.ros/ --rm -it capstone
```

### Port Forwarding
To set up port forwarding, please refer to the [instructions from term 2](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/16cf4a78-4fc7-49e1-8621-3450ca938b77)

### Usage

1. Clone the project repository
```bash
git clone https://github.com/udacity/CarND-Capstone.git
```

2. Install python dependencies
```bash
cd CarND-Capstone
pip install -r requirements.txt
```
3. Make and run styx
```bash
cd ros
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
```
4. Run the simulator

### Real world testing
1. Download [training bag](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/traffic_light_bag_file.zip) that was recorded on the Udacity self-driving car.
2. Unzip the file
```bash
unzip traffic_light_bag_file.zip
```
3. Play the bag file
```bash
rosbag play -l traffic_light_bag_file/traffic_light_training.bag
```
4. Launch your project in site mode
```bash
cd CarND-Capstone/ros
roslaunch launch/site.launch
```
5. Confirm that traffic light detection works on real life images
"# First-sketch-last-project-SDC"
