Running Sample Return Gazebo Sim
================================

This is a brief how-to for getting the test environment up and running with the test world located in this directory.

Prerequisites
-------------

- Non-Virtual machine Ubuntu 12.04 (you can do it on VMware or Virtualbox, but a few things wont render correctly)
- Blender (download from blender.org)
- Full desktop installation of ROS Hydro (http://wiki.ros.org/ROS/Installation)
  - For gazebo in Groovy you need gazebo_ros_pkgs (http://gazebosim.org/wiki/Tutorials/1.9/Installing_gazebo_ros_Packages)
     - It took a few tries, was sluggish running and crashes sometimes, but that could just be my computer.
- Put the following two lines into your .bashrc script
  - export LD_LIBRARY_PATH=your_git_repo_path/gazebo/husky_plugin/lib:$LD_LIBRARY_PATH
  - export GAZEBO_MODEL_PATH=your_git_repo_path/gazebo/models:$GAZEBO_MODEL_PATH


About the test World
--------------------

The test world is a small, 100 meter X 100 meter world, with a few trees and architectural features that are similar to the wpi sample return test grounds. There is also a Clearpath Husky robot, with a stereo camera on top. All of this is located in landscape.world.

Most of the environment is imported from blender as a mesh (COLLADA) file. You can view the blender environment by opening blender/landscape2.blend in blender.


Running the World
----------------
First make sure you have build the husky_plugin package and teleop_husky package. If not, cd to those two packages and type:
```shell
make
```

To run the world in gazebo, start roscore in a terminal:

```shell
roscore
```

and then in a seperate terminal, type this command to run gazebo with ros wrappers:

```shell
cd wpi_sample_return_robot_challenge/gazebo
rosrun gazebo_ros gazebo landscape.world
```

You can also view the camera feeds in gazebo by selecting "Window -> Topic Visualization" and selecting the stereo camera topic.

To teleop husky, start a seperate terminal, type this command:
```shell
rosrun teleop_husky husky_teleop
```

Then you should be able to move husky around in gazebo world using 'A', 'S', 'D', 'F'.

Viewing stereo camera output, disparity map, and point cloud (depth info)
-------------------------------------------------------------

The Husky robot in the simulation is equipped with a stereo camera. We can use the ros package [stereo_image_proc](http://wiki.ros.org/stereo_image_proc) to derive depth information from the stereo cameras.

Ensure that roscore is running. Then, if you haven't already started gazebo, do so (you only need the gazebo server to run)

```shell
cd wpi_sample_return_robot_challenge/gazebo
rosrun gazebo_ros gzserver landscape.world
```

Now start the stereo processing node in a new terminal (TODO: change topic namespace for cameras)

```shell
ROS_NAMESPACE=rrbot/camera1 rosrun stereo_image_proc stereo_image_proc
```

We can now visualize the camera output and the disparity map (color-coded to show depth info). In a new terminal, type

```shell
rosrun image_view stereo_view stereo:=rrbot/camera1 image:=image_rect_color
```

This should pop up three windows: a left and right camera feed, and a disparity map showing depth.

###Visualizing the Point Cloud

To visualize the point cloud, start rviz:

```shell
rosrun rviz rviz
```

When rviz opens, find the "Panels" display (the left-most panel). select "Add" near the bottom of the panel. In the modal dialog that pops-up, select the "By Topic" tab. Select the /rrbot/camera1/points2 topic.

Also, in the Display panel, change the "Fixed Frame" setting from "map" to "camera_link" to give rviz the origin point for the point cloud data. After you do this, you should be able to see the point cloud data visualized in the rviz viewer.

Controlling Robot through ROS (TODO)
--------------------------------------


