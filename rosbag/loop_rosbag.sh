#!/bin/sh

while [ 1 ]; do 
  rosbag play move_forward.bag; 
  sleep 1;
done
