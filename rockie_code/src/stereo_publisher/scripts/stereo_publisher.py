#!/usr/bin/env python
import rospy
from cv_bridge import CvBridge, CvBridgeError
import roslib
import sys
import cv2
import Image, cv
from sensor_msgs.msg import Image as ros_image
from std_msgs.msg import String

def publish_stereo_image():

    while not rospy.is_shutdown():

        ret1, img_left = video1.read()
        ret2, img_right = video2.read()

        ros_img_left = bridge.cv2_to_imgmsg(img_left, "bgr8")
        ros_img_right = bridge.cv2_to_imgmsg(img_right, "bgr8")

        pub_left.publish(ros_img_left)
        pub_right.publish(ros_img_right)
        r.sleep()

def config_video_capture(capture):
    capture.set(cv.CV_CAP_PROP_FRAME_WIDTH, 640)
    capture.set(cv.CV_CAP_PROP_FRAME_HEIGHT, 320)

if __name__ == '__main__':
    try:
        rospy.init_node('stereo_publisher')

        pub_left = rospy.Publisher('stereo_image_left', ros_image)
        pub_right = rospy.Publisher('stereo_image_right', ros_image)

        r = rospy.Rate(10) # 10hz

        bridge = CvBridge()
        video2 = cv2.VideoCapture(1)
        video1 = cv2.VideoCapture(0)

        config_video_capture(video1)
        config_video_capture(video2)

        publish_stereo_image()
    except rospy.ROSInterruptException: pass
