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
from stereo_feature_identifier_db import Stereo_Pair_Keypoints, Stereo_Pair_Keypoint_Matches
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

pub = rospy.Publisher(stereo_feature_matcher_topic, String)

# FLANN parameters
FLANN_INDEX_KDTREE = 0
index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
search_params = dict(checks=50)   # or pass empty dictionary

flann = cv2.FlannBasedMatcher(index_params,search_params)

#camera distance is 94 mm
camera_dist = .094

#focal length is 579 mm
camera_focal_length = 579

def drawMatches(gray1, kpts1, gray2, kpts2, matches):

    h1, w1 = gray1.shape[:2]
    h2, w2 = gray2.shape[:2]
    vis = np.zeros((max(h1, h2), w1+w2), np.uint8)
    vis[:h1, :w1] = gray1
    vis[:h2, w1:w1+w2] = gray2
    vis = cv2.cvtColor(vis, cv2.COLOR_GRAY2BGR)

    for match in matches:
        pt1 = (int(kpts1[match.queryIdx].pt[0]), int(kpts1[match.queryIdx].pt[1]))
        pt2  = (int(kpts2[match.trainIdx].pt[0] + w1), int(kpts2[match.trainIdx].pt[1]))

        #should check against epipolar line
        if math.fabs(pt1[1] - pt2[1]) > 10:
            continue
        
        #cv2.circle(gray1, (int(kpts1[match.queryIdx].pt[0]), int(kpts1[match.queryIdx].pt[1])), 5, (0, 0, 255), -1)
        #cv2.circle(gray2, (int(kpts2[match.trainIdx].pt[0]), int(kpts2[match.trainIdx].pt[1])), 5, (0, 0, 255), -1)

        #cv2.imshow('img2', gray1)
        #cv2.imshow('img3', gray2)

        
        cv2.line(vis, pt1, pt2, (int(255*random.random()), int(255*random.random()), int(255*random.random())), 1) 
        z = triangulate(match, kpts1, kpts2)

        if z > 0:
            cv2.putText(vis, str(z), pt1, cv2.FONT_HERSHEY_SIMPLEX, 2, 255, 2)
            #cv2.putText(vis, str(z), pt1, 
            cv2.circle(vis, pt2, 5, (int(z), int(z), int(z)), -1)
            cv2.circle(vis, pt1, 5, (int(z), int(z), int(z)), -1)

    cv2.imshow('img', vis)

    cv2.waitKey(100)

def triangulate_matches_callback(stereo_pair_keypoint_data_id):
    global session
    global DBSession
    global pub
   
    session = DBSession()
    
    sp_keypoint_matches = get_sp_keypoint_matches(stereo_pair_keypoint_data_id.data)
    matches = pickle.load(open(sp_keypoint_matches.sp_keypoint_matches_filepath, "rb"))

    #get matches
    #get left and right keypoints
    #triangulate matches based on focal length and baseline
    #create 3D point data for each keypoint
    #store to file and db
    
    stereo_pair_keypoints = get_keypoints_pair(sp_keypoint_matches.stereo_pair_keypoint_id)

    left_keypoints = get_left_keypoints(stereo_pair_keypoints)
    right_keypoints = get_right_keypoints(stereo_pair_keypoints)

    _3d_points = triangulate(matches, left_keypoints[0], right_keypoints[0])
  
    #store_3d_points()

    session.close()

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

        Z = (camera_focal_length*camera_dist)/disparity

        _3d_points.append(Z)

    return _3d_points

def store_3d_points():
    pass

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
    #return pickle.load(open(sp_keypoint_matches.sp_keypoint_matches_filepath, "rb"))

def triangulate_matches():
    rospy.init_node("stereo_feature_match_triangulator")
    rospy.Subscriber(stereo_feature_matcher_topic, String, triangulate_matches_callback)
    rospy.spin()

class SerializableKeypoint():
    pass

if __name__ == '__main__':
    triangulate_matches()

    #get cam0
    #get cam1

    #get img1
    #get img2

    #get kpts1
    #get kpts2

    #get matches

    #triangulate matches

    #display on imgs

    '''

    cam0 = cv2.VideoCapture(0)
    cam1 = cv2.VideoCapture(1)

    cam0.set(3, 640)
    cam0.set(4, 360)
    cam0.set(cv2.cv.CV_CAP_PROP_FOURCC, cv2.cv.CV_FOURCC('Y', 'U', 'Y', 'V'))

    cam1.set(3, 640)
    cam1.set(4, 360)
    cam1.set(cv2.cv.CV_CAP_PROP_FOURCC, cv2.cv.CV_FOURCC('Y', 'U', 'Y', 'V'))
    #descriptor = cv2.SIFT(50)
    descriptor = cv2.SIFT(100)

    while True:

        ret1, img1 = cam0.read()
        ret2, img2 = cam1.read()

        gray1 = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)
        gray2 = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)

        kpts1, descs1  = descriptor.detectAndCompute(gray1, None)
        kpts2, descs2  = descriptor.detectAndCompute(gray2, None)
        
        try:
            matches = flann.match(descs1, descs2)
        except:
            continue


        drawMatches(gray1, kpts1, gray2, kpts2, matches)

    '''
    '''
        for match in matches:
            z = triangulate(match, kpts1, kpts2)
            pt = kpts1[match.queryIdx].pt
            
            #cv2.circle(gray1, (int(pt[0]), int(pt[1])), 3, (255, 255, 255), 1) 

            if z > 0:
                cv2.circle(gray1, (int(pt[0]), int(pt[1])), 5, (int(z), int(z), int(z)), -1) 
            else:
                cv2.circle(gray1, (int(pt[0]), int(pt[1])), 5, (0, 0, 255), -1) 


        cv2.imshow('img', gray1)
        cv2.waitKey(100)
    '''
            

