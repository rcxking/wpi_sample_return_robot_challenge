import numpy as np
import cv2
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image as ros_image
from cv_bridge import CvBridge, CvBridgeError
import datetime

stereo_ns = 'my_stereo'
image_name = 'image_mono_rect'
path_to_img_store = '~/Code/wpi-sample-return-robot-challenge/rockie_code/src/stereo_historian/images/'

if __name__ == '__main__':
    store_stereo_images()

def save_image(img, time, camera):

    cv2_img = bridge.imgmsg_to_cv2(img, "bgr8")
    cv2_img_gray = cv2.cvtColor(cv2_img, cv2.COLOR_BGR2GRAY)

    cv2.imwrite('images/' + str(currenttime)  + '-' + camera + 'jpg', cv2_img_gray)

def left_callback(left_image):
    currenttime = datetime.datetime.now().time()
    save_image(left_image, currenttime, "left")

def right_callback(right_image):
    currenttime = datetime.datetime.now().time()
    save_image(right_image, currenttime, "right")

def store_stereo_images():

    bridge = CvBridge()

    rospy.init_node("stereo_historian")

    left_img_topic = stereo_ns + '/left/' + image_name
    right_img_topic = stereo_ns + '/right/' + image_name

    rospy.Subscriber(left_img_topic, ros_image, left_callback)
    rospy.Subscriber(right_img_topic, ros_image, right_callback)

    rospy.spin()
