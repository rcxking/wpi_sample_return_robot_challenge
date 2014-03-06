ROS Packages we're using
------------------------
> TODO: Can we just copy the PR2 code for their stereo camera setup? They have a wide-angle stereo camera that they use for environment mapping/sensing.

- [image_view](http://wiki.ros.org/image_view): for viewing output from cameras
- [stereo_image_proc](http://wiki.ros.org/stereo_image_proc): To generate point cloud data from stereo cameras, and generate a disparity image. TODO: Consider [image_pipeline](http://wiki.ros.org/image_pipeline) as a more complete alternative?
- [viso2_ros](http://wiki.ros.org/viso2_ros): Visual odometry to estimate camera (and robot) motion from mono and stereo cameras.
- [pcl](http://wiki.ros.org/pcl): Extensive library for processing point clouds. 

> Use this for point cloud registration (stitching together point clouds to produce consistent global model, [here](http://pointclouds.org/documentation/tutorials/pairwise_incremental_registration.php#pairwise-incremental-registration) is a tutorial for registering point cloud pairs)

- [stereo_slam](http://www.ros.org/browse/details.php?distro=hydro&name=stereo_slam): SLAM for robots fitted with stereo cameras.

> TODO: is this a reliable package?

Installing Rosberry Pi
----------------------

[Here](http://www.instructables.com/id/Raspberry-Pi-and-ROS-Robotic-Operating-System/step2/Writing-the-image-to-the-SD-card/) is an instructable for installing ROS + Debian OS to the Raspberry Pi. Someone made an [image](http://www.zagrosrobotics.com/files/Raspbian-ROS-full.zip) of Debian with ROS fuerte pre-installed for the Raspberry Pi, so the installation is largely just writing this image to the SD card of the Raspberry Pi.
