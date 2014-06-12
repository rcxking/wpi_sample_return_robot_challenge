#!/bin/sh

while [ 1 ]; do 
  rosbag play recording.bag; 
  sleep 1;
done
