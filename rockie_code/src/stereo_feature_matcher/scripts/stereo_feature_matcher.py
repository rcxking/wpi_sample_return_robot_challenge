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
    match_and_store_feature_matches()

'''
Determine matches for this keypoint, using FLANN. If
the full search turns out to be too intensive, maybe we can try
to search the strongest descriptors first
'''
def create_and_store_features_callback(keypoint_matches_id):

    new_keypoints = get_keypoints(keypoint_matches_id)
    existing_matches = get_all_keypoints()
    matches = find_matches(new_keypoints, existing_matches)
    store_matches(matches)

'''
Every time our feature matcher saves keypoints to the store, perform keypoint matching
'''
def match_and_store_feature_matches():

    rospy.init_node("stereo_feature_matcher")

    rospy.Subscriber(stereo_feature_identifier_topic, String, match_and_store_feature_matches)

    rospy.spin()
