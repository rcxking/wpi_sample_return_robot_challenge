#!/usr/bin/env python

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
from stereo_feature_identifier_db import Stereo_Pair_Keypoints, Stereo_Pair_Keypoint_Matches
from stereo_historian_db import Stereo_Image_Pair, Base
import datetime
import cPickle as pickle
from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker

#stereo_imagepath_base = '/home/will/Code/wpi-sample-return-robot-challenge/rockie_code/src/stereo_historian/scripts/'
stereo_imagepath_base = '/home/rockie/Code/wpi-sample-return-robot-challenge/rockie_code/src/stereo_historian/scripts'

engine = create_engine('mysql://root@localhost/rockie')
Base.metadata.bind = engine
DBSession = sessionmaker(bind=engine)
session = DBSession()

stereo_historian_topic = '/my_stereo/stereo_image_saves'
stereo_feature_identifier_topic = '/my_stereo/stereo_image_keypoint_saves'
stereo_feature_matcher_topic = '/my_stereo/stereo_image_keypoint_matches'

pub = rospy.Publisher(stereo_feature_matcher_topic, String)

# FLANN parameters
FLANN_INDEX_KDTREE = 0
index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
search_params = dict(checks=50)   # or pass empty dictionary

flann = cv2.FlannBasedMatcher(index_params,search_params)

def triangulate_matches_callback(stereo_pair_keypoint_data_id):
    global session
    global DBSession
    global pub
   
    session = DBSession()

    #get matches
    #get left and right keypoints
    #triangulate matches based on focal length and baseline
    #create 3D point data for each keypoint
    #store to file and db
    
    matches = get_matches()
    left_keypoints = get_left_keypoints()
    right_keypoints = get_right_keypoints()

    3d_points = triangulate(matches, left_keypoints, right_keypoints)
  
    store_3d_points()

    session.close()

class py_match:
  pass

def convert_match(match):
    new_match = py_match()
    new_match.distance = match.distance
    new_match.imgIdx = match.imgIdx
    new_match.queryIdx = match.queryIdx
    new_match.trainIdx = match.trainIdx

    return new_match

def save_keypoint_matches(matches, stereo_pair_keypoint):
    
    #filepath = "{0}{1}.keypoint_matches".format(stereo_imagepath_base, stereo_pair_keypoint.left_keypoints_filepath)
    filepath = "{0}.keypoint_matches".format(stereo_pair_keypoint.left_keypoints_filepath)
    
    matches = [convert_match(match) for match in matches]

    pickle.dump(matches, open(filepath, 'wb'))
    return filepath

def get_matches(kps_descs_1, kps_descs_2):
    global flann

    kpts1 = kps_descs_1[0]
    des1 = kps_descs_1[1]

    kpts2 = kps_descs_2[0]
    des2 = kps_descs_2[1]

    #matches = flann.knnMatch(des1, des2, 3)
    matches = flann.match(des1, des2)

    return matches

def recreate_keypoints(kp):
    return cv2.KeyPoint(x=kp.pt[0], y=kp.pt[1], _size=kp.size, _angle=kp.angle, _response=kp.response, _octave=kp.octave, _class_id=kp.class_id) 

def get_left_keypoints(stereo_pair_keypoint):
    #left_keypoints_filepath = "{0}{1}".format(stereo_imagepath_base, stereo_pair_keypoint.left_keypoints_filepath)
    left_keypoints_filepath = "{0}".format(stereo_pair_keypoint.left_keypoints_filepath)

    kpts_descs = pickle.load(open(left_keypoints_filepath, "rb"))
    kpts_descs[0] = [recreate_keypoints(kp) for kp in kpts_descs[0]]
    return kpts_descs

def get_right_keypoints(stereo_pair_keypoint):
    #right_keypoints_filepath = "{0}{1}".format(stereo_imagepath_base, stereo_pair_keypoint.right_keypoints_filepath)
    right_keypoints_filepath = "{0}".format(stereo_pair_keypoint.right_keypoints_filepath)

    return pickle.load(open(right_keypoints_filepath, "rb"))

def get_stereo_pair_keypoint(stereo_pair_keypoint_id):
    global session
    query = session.query(Stereo_Pair_Keypoints)
    stereo_pair_keypoint = query.filter_by(stereo_pair_keypoint_id = int(stereo_pair_keypoint_id)).first()
    return stereo_pair_keypoint

def triangulate_matches():
    rospy.init_node("stereo_feature_match_triangulator")
    rospy.Subscriber(stereo_feature_matcher_topic, String, triangulate_matches_callback)
    rospy.spin()

class SerializableKeypoint():
    pass

if __name__ == '__main__':
    triangulate_matches()

    '''
    left_filename = "1401033910962120122.jpg"
    right_filename = "1401033910962120122.jpg"
    left_kpts = pickle.load(open("{0}images/left/{1}.keypoints".format(stereo_imagepath_base, left_filename), "rb"))
    right_kpts = pickle.load(open("{0}images/right/{1}.keypoints".format(stereo_imagepath_base, right_filename), "rb"))

    matches = get_matches(left_kpts, right_kpts) 

    gray = cv2.imread("{0}images/left/{1}".format(stereo_imagepath_base, left_filename), 0)
    kpts = left_kpts[0]

    kpts = [recreate_keypoints(kp) for kp in kpts]
    
    img=cv2.drawKeypoints(gray,kpts)
    cv2.imshow('img', img)
    cv2.waitKey(0)
    '''

