import numpy as np
import cv2
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image as ros_image
from cv_bridge import CvBridge, CvBridgeError
import datetime

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
    rospy.Subscriber("my_stereo/left_image", ros_image, left_callback)
    rospy.Subscriber("my_stereo/right_image", ros_image, right_callback)

    rospy.spin()
