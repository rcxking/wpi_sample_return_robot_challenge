#!/usr/bin/env python

'''
This node subscribes to the stereo image historian node, and every time a stereo image
pair is saved it will perform feature matching on it. It will then save features to the
database
'''
import numpy as np
import cv2
import cv
import os
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image as ros_image
from cv_bridge import CvBridge, CvBridgeError
from stereo_feature_identifier_db import Stereo_Pair_Keypoints, Stereo_Pair_Keypoint_Matches, Stereo_Image_Pair, Base
import datetime
import cPickle as pickle
from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker
import math
import random

stereo_imagepath_base = "{0}/Code/wpi-sample-return-robot-challenge/rockie_code/src/stereo_historian/scripts".format(os.getenv("HOME"))

bridge = CvBridge()

engine = create_engine('mysql://root@localhost/rockie')
Base.metadata.bind = engine
DBSession = sessionmaker(bind=engine)
session = DBSession()

stereo_historian_topic = '/my_stereo/stereo_image_saves'
stereo_feature_identifier_topic = '/my_stereo/stereo_image_keypoint_saves'
stereo_feature_matcher_topic = '/my_stereo/stereo_image_keypoint_matches'
stereo_feature_matcher_debug_topic = '/my_stereo/stereo_feature_matcher/matches_img'

matches_img_pub = rospy.Publisher(stereo_feature_matcher_debug_topic, ros_image)
pub = rospy.Publisher(stereo_feature_matcher_topic, String)

# FLANN parameters
FLANN_INDEX_KDTREE = 0
index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
search_params = dict(checks=50)   # or pass empty dictionary

#flann = cv2.FlannBasedMatcher(index_params,search_params)
flann = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

def match_and_store_features_callback(stereo_pair_keypoint_data_id):
    global session
    global DBSession
    global pub
   
    #get left keypoints filepath
    #get right keypoints filepath

    #perform matches and return matches
    #save matches
    #publish

    session = DBSession()

    stereo_pair_keypoint = get_stereo_pair_keypoint(stereo_pair_keypoint_data_id.data)

    left_keypoints = get_left_keypoints(stereo_pair_keypoint)
    right_keypoints = get_right_keypoints(stereo_pair_keypoint)

    matches = get_matches(left_keypoints, right_keypoints)

    # Sort them in the order of their distance.
    matches = sorted(matches, key = lambda x:x.distance)

    matches_filepath = save_keypoint_matches(matches, stereo_pair_keypoint)

    publish_debug_matches_img(left_keypoints, right_keypoints, matches, stereo_pair_keypoint)

    sp_matches = Stereo_Pair_Keypoint_Matches()
    sp_matches.stereo_pair_keypoint_id = stereo_pair_keypoint_data_id.data
    sp_matches.sp_keypoint_matches_filepath = matches_filepath
   
    session.add(sp_matches)
    session.commit()

    stereo_pair_keypoint_matches_id = sp_matches.sp_keypoint_matches_id
    
    pub.publish(str(stereo_pair_keypoint_matches_id))

    session.close()

def publish_debug_matches_img(left_keypoints, right_keypoints, matches, stereo_pair_keypoint):
    global session
    global DBSession
    global pub
   
    session = DBSession()

    stereo_image_pair_id = stereo_pair_keypoint.stereo_image_pair_id 
    query = session.query(Stereo_Image_Pair)
    stereo_image_pair = query.filter_by(stereo_image_pair_id = int(stereo_image_pair_id)).first()

    img_left_filepath = stereo_image_pair.left_filepath
    img_right_filepath = stereo_image_pair.right_filepath

    img_left = cv2.imread(img_left_filepath, 0)
    img_right = cv2.imread(img_right_filepath, 0)
  
    drawMatches(img_left, left_keypoints[0], img_right, right_keypoints[0], matches)

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

def get_matches(kps_descs_1, kps_descs_2):
    global flann

    kpts1 = kps_descs_1[0]
    des1 = kps_descs_1[1]

    kpts2 = kps_descs_2[0]
    des2 = kps_descs_2[1]

    matches = flann.match(des1, des2)

    return matches

def drawMatches(gray1, kpts1, gray2, kpts2, matches):

  h1, w1 = gray1.shape[:2]
  h2, w2 = gray2.shape[:2]
  vis = np.zeros((max(h1, h2), w1+w2), np.uint8)
  vis[:h1, :w1] = gray1
  vis[:h2, w1:w1+w2] = gray2
  #vis = cv2.cvtColor(vis, cv2.COLOR_GRAY2BGR)

  for match in matches:
    pt1 = (int(kpts1[match.queryIdx].pt[0]), int(kpts1[match.queryIdx].pt[1]))
    pt2  = (int(kpts2[match.trainIdx].pt[0] + w1), int(kpts2[match.trainIdx].pt[1]))

    #should check against epipolar line
    if math.fabs(pt1[1] - pt2[1]) > 10:
      continue
    
    cv2.line(vis, pt1, pt2, (int(255*random.random()), int(255*random.random()), int(255*random.random())), 1) 

  matches_img_pub.publish(ConvertCV2ToROSGrayscale(vis))

def ConvertCV2ToROSGrayscale(img):
  global bridge

  color = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
  ros_img = bridge.cv2_to_imgmsg(color, "bgr8")
  return ros_img

def recreate_keypoints(kp):
    return cv2.KeyPoint(x=kp.pt[0], y=kp.pt[1], _size=kp.size, _angle=kp.angle, _response=kp.response, _octave=kp.octave, _class_id=kp.class_id) 

def get_left_keypoints(stereo_pair_keypoint):
    left_keypoints_filepath = "{0}".format(stereo_pair_keypoint.left_keypoints_filepath)

    kpts_descs = pickle.load(open(left_keypoints_filepath, "rb"))
    kpts_descs[0] = [recreate_keypoints(kp) for kp in kpts_descs[0]]
    #kpts_descs[1] = np.array(kpts_descs[1], np.float32)
    kpts_descs[1] = np.array(kpts_descs[1], np.uint8)

    return kpts_descs

def get_right_keypoints(stereo_pair_keypoint):
    right_keypoints_filepath = "{0}".format(stereo_pair_keypoint.right_keypoints_filepath)

    kpts_descs =  pickle.load(open(right_keypoints_filepath, "rb"))
    kpts_descs[0] = [recreate_keypoints(kp) for kp in kpts_descs[0]]
    #kpts_descs[1] = np.array(kpts_descs[1], np.float32)
    kpts_descs[1] = np.array(kpts_descs[1], np.uint8)

    return kpts_descs

def get_stereo_pair_keypoint(stereo_pair_keypoint_id):
    global session
    query = session.query(Stereo_Pair_Keypoints)
    stereo_pair_keypoint = query.filter_by(stereo_pair_keypoint_id = int(stereo_pair_keypoint_id)).first()
    return stereo_pair_keypoint

def match_features():
    rospy.init_node("stereo_feature_matcher")
    rospy.Subscriber(stereo_feature_identifier_topic, String, match_and_store_features_callback)
    rospy.spin()

class SerializableKeypoint():
    pass

if __name__ == '__main__':
    match_features()

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

