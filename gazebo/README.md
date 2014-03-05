Running Sample Return Gazebo Sim
================================

This is a brief how-to for getting Gazebo up and running with the test world located in this directory.

Prerequisites
-------------

- Non-Virtual machine Ubuntu (you can do it on VMware or Virtualbox, but a few things wont render correctly)
- Blender (download from blender.org)
- Full desktop installation of ROS Hydro (http://wiki.ros.org/ROS/Installation)


Running the World
----------------

To run the world in gazebo, start roscore in a terminal:

```shell
roscore
```

and then in a seperate terminal, type this command to run gazebo with ros wrappers:

```shell
rosrun gazebo_ros gazebo landscape.world
```

To visualize the stereo camera output, open a new terminal and type

```shell
rqt
```

On the menu, select "Plugins -> Visualization -> Image View". This will allow you to select and visualize the left and right cameras.

You can also view the images in gazebo by selecting "Window -> Topic Visualization" and selecting the stereo camera topic.:w


About the test World
--------------------

The test world is a small, 100 meter X 100 meter world, with a few trees and architectural features that are similar to the wpi sample return test grounds. There is also a Clearpath Husky robot, with a stereo camera on top. All of this is located in landscape.world.

Most of the environment is imported from blender as a mesh (COLLADA) file. You can view the blender environment by opening blender/landscape2.blend in blender.

Controlling Robot through ROS (TODO)
--------------------------------------


