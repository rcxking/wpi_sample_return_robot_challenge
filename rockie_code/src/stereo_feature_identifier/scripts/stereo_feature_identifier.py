'''
This node subscribes to the stereo image historian node, and every time a stereo image
pair is saved it will perform feature matching on it. It will then save features to the
database
'''
import numpy as np
import cv2
import cv
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image as ros_image
from cv_bridge import CvBridge, CvBridgeError
import datetime

stereo_historian_topic = '/my_stereo/stereo_image_saves'
def create_and_store_features_callback(filepath):

    #split_pair = filepath_pair.split("###")

    #img_left_filepath = split_pair[0]
    #img_right_filepath = split_pair[1]

    #img_left = cv2.imread(img_left_filepath)
    #img_right = cv2.imread(split_pair[1])

    img = cv2.imread(filepath, 0)

    #keypoints1 = CreateKeypoints(img_left)
    #keypoints2 = CreateKeypoints(img_right)

    keypoints = CreateKeypoints(img)

    #SaveKeypoints(keypoints1, keypoints2, img_left_filepath, img_right_filepath)

    SaveKeypoints(keypoints, filepath)

    #matches = FindMatches()

    #SaveMatches(matches, left_img=img_left, right_img=img_right)

def find_and_store_features():
'''
    img = cv2.imread("/home/will/Code/wpi-sample-return-robot-challenge/rockie_code/src/stereo_historian/images/10:19:39.596026-left.jpg", 0)
    #img = cv2.imread("/home/will/Downloads/messi5.jpg", 0)

    sift = cv2.SIFT()
    surf = cv2.SURF()
    kp = surf.detect(img, None)

    img2 = cv2.drawKeypoints(img,kp)

    cv2.imshow('img', img2)

    cv2.waitKey(0)
'''
    rospy.init_node("stereo_feature_identifier")
    rospy.Subscriber(stereo_historian_topic, String, create_and_store_features_callback)
    rospy.spin()

if __name__ == '__main__':
    find_and_store_features()

