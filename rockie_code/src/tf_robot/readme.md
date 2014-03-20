#Robot transforms

This package handles all transforms between frames of reference for the robot (map, base_link, camera, etc).

> NOTE TO SELF: To get the joint angle for base_link --> camera, call 
```shell
rosservice call /gazebo/get_joint_properties stereo_camera_joint
```
