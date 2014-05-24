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
stereo_imagepath_base = '/home/rockie/Code/wpi-sample-return-robot-challenge/rockie_code/src/stereo_historian/scripts/'

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

    sp_matches = Stereo_Pair_Keypoint_Matches()
    sp_matches.sp_keypoint_id_1 = left_keypoints.stereo_pair_keypoint_id
    sp_matches.sp_keypoint_id_2 = right_keypoints.stereo_pair_keypoint_id
    sp_matches.sp_keypoint_matches_filepath = save_keypoint_matches(matches)
   
    #session.commit()
    #stereo_pair_keypoint_matches_id = sp_matches.sp_keypoint_matches_id
    #pub.publish(str(stereo_pair_keypoint_matches_id))

    session.close()

def save_keypoint_matches(matches):
    pass 

def get_matches(kps_descs_1, kps_descs_2):
    global flann

    kpts1 = kps_descs_1[0]
    des1 = kps_descs_1[1]

    kpts2 = kps_descs_2[0]
    des2 = kps_descs_2[1]

    #matches = flann.knnMatch(des1,des2,k=2)
    matches = flann.knnMatch(des1, des2, 3)
    return matches

def get_left_keypoint(stereo_pair_keypoint):
    left_keypoints_filepath = "{0}{1}".format(stereo_imagepath_base, stereo_pair_keypoint.left_filepath)
    return pickle.load(open(left_keypoints_filepath, "rb"))

def get_right_keypoint(stereo_pair_keypoint):
    right_keypoints_filepath = "{0}{1}".format(stereo_imagepath_base, stereo_pair_keypoint.right_filepath)
    return pickle.load(open(right_keypoints_filepath, "rb"))

def get_stereo_pair_keypoint(stereo_pair_keypoint_id):
    global session
    query = session.query(Stereo_Pair_Keypoint)
    stereo_pair_keypoint = query.filter_by(stereo_pair_keypoint_id = int(stereo_pair_keypoint_id)).first()

def match_features():
    rospy.init_node("stereo_feature_matcher")
    rospy.Subscriber(stereo_feature_identifier_topic, String, match_and_store_features_callback)
    rospy.spin()

class SerializableKeypoint():
    pass

def ConvertToSerializableKeypoint(kp):
    new_kp = SerializableKeypoint()
    new_kp.x = kp.pt[0]
    new_kp.y = kp.pt[1]
    new_kp.size = kp.size
    new_kp.angle = kp.angle
    new_kp.response = kp.response
    new_kp.octave = kp.octave
    new_kp.class_id = kp.class_id

    return new_kp

def SaveKeypoints(kp, image_filepath):
    serializable_kps = ConvertToSerializableKeypoints(kp)
    filepath = "{0}.keypoints".format(image_filepath)
    pickle.dump(serializable_kps, open(filepath, 'wb'))
    return filepath

def ConvertToSerializableKeypoints(kp):
    return [ConvertToSerializableKeypoint(keypoint) for keypoint in kp]

if __name__ == '__main__':
    #match_features()
    left_filename = "1400960287109987484.jpg"
    right_filename = "1400960287265802411.jpg"
    left_kpts = pickle.load(open("{0}images/left/{1}.keypoints".format(stereo_imagepath_base, left_filename), "rb"))
    right_kpts = pickle.load(open("{0}images/right/{1}.keypoints".format(stereo_imagepath_base, right_filename), "rb"))

    get_matches(left_kpts, right_kpts) 
