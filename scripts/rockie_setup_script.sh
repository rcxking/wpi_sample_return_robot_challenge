#!/bin/bash

######################################################
#
# This is a setup script that is meant to install all
# of the necessary software to simulate and/or run
# rockie, as well as setup the environment (libraries
# etc.). 
#
# Currently, the structure of the script is:
#       Section 1. Build tools, gcc, cmake, etc.
#       Section 2. ROS
#       Section 3. OpenCV
#
# The chosen versions are as follows:
#       Ubuntu 12.04 (assumed you are running)
#       ROS Groovy
# 
# 
######################################################


# Make sure the script is NOT run as root (sudo)
if [ `whoami` == root ]; then
    echo "Please do NOT run as root."
    exit
fi

# Check if running Ubuntu 12.04
. /etc/lsb-release
if [ $DISTRIB_ID == "Ubuntu" ]; then
    if [ $DISTRIB_RELEASE == 12.04 ]; then
        echo "User runnign Ubuntu 12.04"
    else
        echo "WARNING: This script was only tested on Ubuntu 12.04"
    fi
else
    echo "WARNING: This script was only tested on Ubuntu 12.04"
fi



######################################################
# SECTION 1.  BUILD TOOLS
######################################################
sudo apt-get update
echo "------------ Removing any pre-installed ffmpeg and x264"
sudo apt-get -qq remove ffmpeg x264 libx264-dev

echo "------------ Installing build tools..."
sudo apt-get -qq -y install libopencv-dev build-essential checkinstall cmake pkg-config yasm libtiff4-dev libjpeg-dev libjasper-dev libavcodec-dev libavformat-dev libswscale-dev libdc1394-22-dev libxine-dev libgstreamer0.10-dev libgstreamer-plugins-base0.10-dev libv4l-dev python-dev python-numpy libtbb-dev libqt4-dev libgtk2.0-dev libfaac-dev libmp3lame-dev libopencore-amrnb-dev libopencore-amrwb-dev libtheora-dev libvorbis-dev libxvidcore-dev x264 v4l-utils ffmpeg

# Although the user probably already had Git in order to get this file...
sudo apt-get install -y git 



echo "------------ Installing ROS..."
######################################################
# SECTION 2.  ROS
######################################################
# From http://wiki.ros.org/groovy/Installation/Ubuntu

# 1.1
# It is assumed that the user already has configured 
# their Ubuntu repositories to allow "restricted," 
# "universe," and "multiverse."

# 1.2
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu precise main" > /etc/apt/sources.list.d/ros-latest.list'

# 1.3
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -

# 1.4
sudo apt-get update
sudo apt-get install -y ros-groovy-desktop-full

# 1.5
sudo rosdep init
rosdep update

# 1.6
echo "source /opt/ros/groovy/setup.bash" >> ~/.bashrc
source ~/.bashrc



echo "------------ Replacing ROS's OpenCV with newest version..."
######################################################
# SECTION 3.  OPENCV
######################################################

# sudo apt-get install -y libopencv-dev

# This follows: https://sites.google.com/site/rameyarnaud/research/ros/latest-opencv-in-ros
# Also, check out https://help.ubuntu.com/community/OpenCV
# Get the latest version of OpenCV 
ROSVERSION="groovy"

# Get and compile OpenCV
cd ~
wget --content-disposition http://sourceforge.net/projects/opencvlibrary/files/latest
unzip opencv*
cd opencv*
mkdir BUILD
cd BUILD
cmake ..
make 

# Replace ROS OpenCV
sudo chmod a+rw -R /opt/ros/$ROSVERSION/lib/
mkdir /opt/ros/$ROSVERSION/lib/libopencv_backup
mv /opt/ros/$ROSVERSION/lib/libopencv*.so* /opt/ros/$ROSVERSION/lib/libopencv_backup
cp ./lib/libopencv* /opt/ros/$ROSVERSION/lib/
ls -hal /opt/ros/$ROSVERSION/lib/libopencv*











