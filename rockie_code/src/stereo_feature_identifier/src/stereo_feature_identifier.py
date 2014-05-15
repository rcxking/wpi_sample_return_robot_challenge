'''
This node subscribes to the stereo image historian node, and every time a stereo image
pair is saved it will perform feature matching on it. It will then save features to the
database
'''
import numpy as np
import cv2
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image as ros_image
from cv_bridge import CvBridge, CvBridgeError
import datetime

stereo_historian_topic = 'stereo_pair_history'

if __name__ == '__main__':
    find_and_store_features()

def create_and_store_features_callback(filepath_pair):

    split_pair = filepath_pair.split("###")

    img_left_filepath = split_pair[0]
    img_right_filepath = split_pair[1]

    img_left = cv2.imread(img_left_filepath)
    img_right = cv2.imread(split_pair[1])

    keypoints1 = CreateKeypoints(img_left)
    keypoints2 = CreateKeypoints(img_right)

    SaveKeypoints(keypoints1, keypoints2, img_left_filepath, img_right_filepath)

    matches = FindMatches()

    SaveMatches(matches, left_img=img_left, right_img=img_right)

def find_and_store_features():

    rospy.init_node("stereo_feature_identifier")

    rospy.Subscriber(stereo_historian_topic, String, create_and_store_features_callback)

    rospy.spin()
