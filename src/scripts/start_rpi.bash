#!/bin/bash

master_ip="10.42.0.49"
hostname=`hostname`

export ROS_MASTER_URI=http://${master_ip}:11311
export ROS_HOSTNAME=${hostname}.local

source ~/edo_ws/devel/setup.bash
#source ~/catkin_ws/devel/setup.bash
roslaunch calib calibrate.launch
