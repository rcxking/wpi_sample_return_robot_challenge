Running Sample Return Gazebo Sim
================================

This is a brief how-to for getting Gazebo up and running with the test world located in this directory.

Prerequisites
-------------

- Non-Virtual machine Ubuntu (you can do it on VMware or Virtualbox, but a few things wont render correctly)
- Blender (download from blender.org)
- Gazebo (follow install instructions from gazebosim.org)


Running the World
----------------

To run the world in gazebo, type this at the terminal:

```shell
gazebo landscape.world
```

To visualize the stereo camera output, click on "Windows -> Topic Visualization", and select the stereo camera item.

About the test World
--------------------

The test world is a small, 100 meter X 100 meter world, with a few trees and architectural features that are similar to the wpi sample return test grounds. There is also a Clearpath Husky robot, with a stereo camera on top. All of this is located in landscape.world.

Most of the environment is imported from blender as a mesh (COLLADA) file. You can view the blender environment by opening blender/landscape2.blend in blender.

