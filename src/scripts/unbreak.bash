#!/bin/bash


master_ip="10.42.0.49"
hostname=`hostname`

export ROS_MASTER_URI=http://${master_ip}:11311
export ROS_HOSTNAME=${hostname}.local






source ~/edo_ws/devel/setup.bash

rostopic pub bridge_jnt_reset edo_core_msgs/JointReset "{joints_mask: 63, disengage_steps: 2000, disengage_offset: 3.5}" -1

