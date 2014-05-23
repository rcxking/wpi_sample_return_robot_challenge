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
from stereo_feature_identifier_db import Stereo_Pair_Keypoints
from stereo_historian_db import Stereo_Image_Pair, Base
import datetime
import cPickle as pickle
from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker


stereo_imagepath_base = '/home/will/Code/wpi-sample-return-robot-challenge/rockie_code/src/stereo_historian/scripts/'

sift = cv2.SIFT()

engine = create_engine('mysql://root@localhost/rockie')
Base.metadata.bind = engine
DBSession = sessionmaker(bind=engine)
session = DBSession()

stereo_historian_topic = '/my_stereo/stereo_image_saves'
stereo_feature_identifier_topic = '/my_stereo/stereo_image_keypoint_saves'
stereo_feature_matcher_topic = '/my_stereo/stereo_image_keypoint_matches'

pub = rospy.Publisher(stereo_feature_matcher_topic, String)

def match_and_store_features_callback(stereo_pair_keypoint_data_id):
    global session
    global DBSession
    global pub
   
    session = DBSession()

    stereo_pair_keypoint_id = stereo_pair_keypoint_data_id.data
    query = session.query(Stereo_Pair_Keypoint)
    stereo_pair_keypoint = query.filter_by(stereo_pair_keypoint_id = int(stereo_pair_keypoint_id)).first()

    img_left_filepath = "{0}{1}".format(stereo_imagepath_base, stereo_pair_keypoint.left_filepath)
    img_right_filepath = "{0}{1}".format(stereo_imagepath_base, stereo_pair_keypoint.right_filepath)

    img_left = cv2.imread(img_left_filepath, 0)
    img_right = cv2.imread(img_right_filepath, 0)

    left_keypoints = CreateKeypoints(img_left)
    right_keypoints = CreateKeypoints(img_right)
    
    left_img_keypoints_filepath = SaveKeypoints(left_keypoints, img_left_filepath)
    right_img_keypoints_filepath = SaveKeypoints(right_keypoints, img_right_filepath)

    stereo_pair_keypoints = Stereo_Pair_Keypoints()

    stereo_pair_keypoints.left_keypoints_filepath
    stereo_pair_keypoints.right_keypoints_filepath
    stereo_pair_keypoints.stereo_pair_keypoint_id = stereo_pair_keypoint_id

    session.commit()
    stereo_pair_keypoint_id = stereo_pair_keypoints.stereo_pair_keypoint_id
    pub.publish(str(stereo_pair_keypoint_id))

    session.close()

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

def CreateKeypoints(img):
    global sift
    kp = sift.detect(img, None)
    return kp

def SaveKeypoints(kp, image_filepath):
    serializable_kps = ConvertToSerializableKeypoints(kp)
    filepath = "{0}.keypoints".format(image_filepath)
    pickle.dump(serializable_kps, open(filepath, 'wb'))
    return filepath

def ConvertToSerializableKeypoints(kp):
    return [ConvertToSerializableKeypoint(keypoint) for keypoint in kp]

if __name__ == '__main__':
    match_features()
