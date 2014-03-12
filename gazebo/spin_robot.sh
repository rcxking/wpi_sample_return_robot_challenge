#!/bin/sh

rosservice call /gazebo/apply_joint_effort '{joint_name: "stereo_camera_joint", effort: -0.1, start_time: {secs: 0, nsecs: 0}, duration: {secs: 2, nsecs: 0}}'


