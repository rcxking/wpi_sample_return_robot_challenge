#!/usr/bin/env python

'''
This node subscribes to the stereo image historian node, and every time a stereo image
pair is saved it will perform feature matching on it. It will then save features to the
database
'''
import numpy as np
import cv2
import math
import cv
import os
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image as ros_image
from cv_bridge import CvBridge, CvBridgeError
from stereo_feature_identifier_db import Stereo_Pair_Keypoints, Stereo_Pair_Keypoint_Matches, Stereo_3D_Matches
from stereo_historian_db import Stereo_Image_Pair, Base
import datetime
import cPickle as pickle
from sqlalchemy import create_engine
import random
from sqlalchemy.orm import sessionmaker

stereo_imagepath_base = "{0}/Code/wpi-sample-return-robot-challenge/rockie_code/src/stereo_historian/scripts".format(os.getenv("HOME"))

engine = create_engine('mysql://root@localhost/rockie')
Base.metadata.bind = engine
DBSession = sessionmaker(bind=engine)
session = DBSession()

stereo_historian_topic = '/my_stereo/stereo_image_saves'
stereo_feature_identifier_topic = '/my_stereo/stereo_image_keypoint_saves'
stereo_feature_matcher_topic = '/my_stereo/stereo_image_keypoint_matches'
stereo_feature_triangulator_topic = '/my_stereo/stereo_image_3D_points'

pub = rospy.Publisher(stereo_feature_triangulator_topic, String)

#camera distance is 94 mm
camera_dist = .094

#focal length is 579 mm
f = 579

def update_graph_callback(sp_keypoint_matches_data_id):
    global session
    global DBSession
    global pub
    
    #create pose node
    #connect pose node to previous pose node

    #iterate through prior 3d_matches
      #if there is a 3d_match-3d_match (> min num of matches):
        #connect newly created pose node to found 3d_match

        #if we've found a 3d_match-3d_match match that share enough nodes
          #don't add new 3d_matches as new feature node
        #else
          #add new 3d_matches as new feature node to graph
          #connect this new node to new pose node

def get_keypoints_pair(spk_id):
    global session
    query = session.query(Stereo_Pair_Keypoints)
    stereo_pair_keypoints = query.filter_by(stereo_pair_keypoint_id = int(spk_id)).first()
    return stereo_pair_keypoints

def triangulate(matches, kpts1, kpts2):

    _3d_points = []

    for match in matches:    

        query_pt = kpts1[match.queryIdx]
        train_pt = kpts2[match.trainIdx]

        #should check against epipolar line        
        if(math.fabs(query_pt.pt[1] - train_pt.pt[1]) > 10):
            continue

        disparity = math.fabs(query_pt.pt[0] - train_pt.pt[0])

        #consider query image as origin
        x = query_pt.pt[0]
        y = query_pt.pt[1]

        #see pg 371 and 416 of O'REILLY "Learning OpenCV" 2008
        Z = (f*camera_dist)/disparity
        Y = (y*Z)/f
        X = (x*Z)/f

        _3d_points.append([match, X, Y, Z])

    return _3d_points

def store_3d_points(_3d_points, matches_filepath):
    filepath = "{0}.3d_points".format(matches_filepath)
    pickle.dump(_3d_points, open(filepath, 'wb'))
    return filepath

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
    
    filepath = "{0}.keypoint_matches".format(stereo_pair_keypoint.left_keypoints_filepath)
    
    matches = [convert_match(match) for match in matches]

    pickle.dump(matches, open(filepath, 'wb'))
    return filepath

def recreate_keypoints(kp):
    return cv2.KeyPoint(x=kp.pt[0], y=kp.pt[1], _size=kp.size, _angle=kp.angle, _response=kp.response, _octave=kp.octave, _class_id=kp.class_id) 

def get_left_keypoints(stereo_pair_keypoint):
    left_keypoints_filepath = "{0}".format(stereo_pair_keypoint.left_keypoints_filepath)

    kpts_descs = pickle.load(open(left_keypoints_filepath, "rb"))
    kpts_descs[0] = [recreate_keypoints(kp) for kp in kpts_descs[0]]
    return kpts_descs

def get_right_keypoints(stereo_pair_keypoint):
    right_keypoints_filepath = "{0}".format(stereo_pair_keypoint.right_keypoints_filepath)

    return pickle.load(open(right_keypoints_filepath, "rb"))

def get_sp_keypoint_matches(sp_keypoint_matches_id):
    global session
    query = session.query(Stereo_Pair_Keypoint_Matches)
    sp_keypoint_matches = query.filter_by(sp_keypoint_matches_id = int(sp_keypoint_matches_id)).first()
    return sp_keypoint_matches

def update_graph():
    rospy.init_node("stereo_graph_manager")
    rospy.Subscriber(stereo_graph_manager_topic, String, update_graph_callback)
    rospy.spin()

class SerializableKeypoint():
    pass

if __name__ == '__main__':
    update_graph()

