#WPI Feature Finder ROS Package

The purpose of this package is to match WPI-supplied feature images on the camera inputs for the purpose of absolute localization. If possible, a homography matching like the one Jed has done shown [here](https://github.com/rcxking/wpi_sample_return_robot_challenge/issues/3) would allow us to match the feature images from any oblique angle.

The general processing outline of this node is:

1. Attempt to perform [homographic feature matching](http://docs.opencv.org/trunk/doc/py_tutorials/py_feature2d/py_feature_homography/py_feature_homography.html) on both camera images, using the [images supplied by wpi](file:///home/will/Downloads/NASA%20Sample%20Return%20Robot%20Field%202014[4].pdf)
2. If we can successfully perform this matching with both cameras, we can [determine the location of this object](http://en.wikipedia.org/wiki/Epipolar_geometry) in our map.
3. If we can determine the location of three features (or two, with an additional assumption), we can triangulate the absolute position and orientation of the robot.
4. With this information, we can perform absolute SLAM in the wpi-supplied frame of reference (we now have a tf from robot_frame --> wpi_map_frame), and leave the rest up to motion planning.
