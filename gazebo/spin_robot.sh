#!/bin/sh

rosservice call /gazebo/apply_joint_effort '{joint_name: "back_left_joint", effort: 7.0, start_time: {secs: 0, nsecs: 0}, duration: {secs: 10, nsecs: 0}}'

rosservice call /gazebo/apply_joint_effort '{joint_name: "back_right_joint", effort: -7.0, start_time: {secs: 0, nsecs: 0}, duration: {secs: 10, nsecs: 0}}'

rosservice call /gazebo/apply_joint_effort '{joint_name: "front_left_joint", effort: 7.0, start_time: {secs: 0, nsecs: 0}, duration: {secs: 10, nsecs: 0}}'

rosservice call /gazebo/apply_joint_effort '{joint_name: "front_right_joint", effort: -7.0, start_time: {secs: 0, nsecs: 0}, duration: {secs: 10, nsecs: 0}}'


