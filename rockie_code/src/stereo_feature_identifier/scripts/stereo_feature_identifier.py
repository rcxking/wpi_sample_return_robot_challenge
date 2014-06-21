#!/usr/bin/env python

'''
This node subscribes to the stereo image historian node, and every time a stereo image
pair is saved it will perform feature matching on it. It will then save features to the
database
'''
import numpy as np
import cv2
import os
import cv
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image as ros_image
from cv_bridge import CvBridge, CvBridgeError
from stereo_feature_identifier_db import Stereo_Pair_Keypoints, Stereo_Image_Pair, Base
import cPickle as pickle
from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker

stereo_imagepath_base = "{0}/Code/wpi-sample-return-robot-challenge/rockie_code/src/stereo_historian/scripts/".format(os.getenv("HOME"))

#sift = cv2.SIFT()
#sift = cv2.SIFT(50)
#sift = cv2.SURF(3000)
sift = cv2.ORB(scaleFactor=1.1)

engine = create_engine('mysql://root@localhost/rockie')
Base.metadata.bind = engine
DBSession = sessionmaker(bind=engine)
session = DBSession()

stereo_historian_topic = '/my_stereo/stereo_image_saves'
stereo_feature_identifier_topic = '/my_stereo/stereo_image_keypoint_saves'
stereo_feature_identifier_log_topic = '/stereo_feature_identifier/log'

stereo_left_keypoints_topic = 'my_stereo/stereo_feature_identifier/left_image_keypoints'
stereo_right_keypoints_topic = 'my_stereo/stereo_feature_identifier/right_image_keypoints'

pub = rospy.Publisher(stereo_feature_identifier_topic, String)
log = rospy.Publisher(stereo_feature_identifier_log_topic, String)
left_keypoints_img_pub = rospy.Publisher(stereo_left_keypoints_topic, ros_image)
right_keypoints_img_pub = rospy.Publisher(stereo_right_keypoints_topic, ros_image)

bridge = CvBridge()

def create_and_store_features_callback(stereo_image_pair_data_id):
    global session
    global DBSession
    global pub
   
    session = DBSession()

    stereo_image_pair_id = stereo_image_pair_data_id.data
    query = session.query(Stereo_Image_Pair)
    stereo_image_pair = query.filter_by(stereo_image_pair_id = int(stereo_image_pair_id)).first()

    img_left_filepath = stereo_image_pair.left_filepath
    img_right_filepath = stereo_image_pair.right_filepath

    img_left = cv2.imread(img_left_filepath, 0)
    img_right = cv2.imread(img_right_filepath, 0)

    left_keypoints = CreateKeypointsAndDescriptors(img_left)
    right_keypoints = CreateKeypointsAndDescriptors(img_right)
    
    debug_keypoint_images_topic(img_left, img_right, left_keypoints, right_keypoints)

    if(left_keypoints[1] != None and right_keypoints[1] != None):

        left_img_keypoints_filepath = SaveKeypoints(left_keypoints, img_left_filepath)
        right_img_keypoints_filepath = SaveKeypoints(right_keypoints, img_right_filepath)

        stereo_pair_keypoints = Stereo_Pair_Keypoints()

        stereo_pair_keypoints.left_keypoints_filepath = left_img_keypoints_filepath
        stereo_pair_keypoints.right_keypoints_filepath = right_img_keypoints_filepath
        stereo_pair_keypoints.stereo_image_pair_id = stereo_image_pair_id

        session.add(stereo_pair_keypoints)
        session.commit()
        stereo_pair_keypoint_id = stereo_pair_keypoints.stereo_pair_keypoint_id
        pub.publish(str(stereo_pair_keypoint_id))

    session.close()

def debug_keypoint_images_topic(img_left, img_right, left_keypoints, right_keypoints):

  left_keypoints_img = cv2.drawKeypoints(img_left, left_keypoints[0])
  right_keypoints_img = cv2.drawKeypoints(img_right, right_keypoints[0])

  log.publish(str(left_keypoints_img.shape)) 

  try:
    pass
    left_keypoints_img_pub.publish(ConvertCV2ToROSGrayscale(left_keypoints_img))
    right_keypoints_img_pub.publish(ConvertCV2ToROSGrayscale(right_keypoints_img))
  except CvBridgeError, e:
    print e


def ConvertCV2ToROSGrayscale(img):
  global bridge

  #color = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
  ros_img = bridge.cv2_to_imgmsg(img, "bgr8")
  return ros_img

  #cv2_img = bridge.imgmsg_to_cv2(img, "bgr8")
  #cv2_img_gray = cv2.cvtColor(cv2_img, cv2.COLOR_BGR2GRAY)
  #return cv2_img_gray

def find_and_store_features():
    rospy.init_node("stereo_feature_identifier")
    rospy.Subscriber(stereo_historian_topic, String, create_and_store_features_callback)
    rospy.spin()

class SerializableKeypoint():
    pt = []

def ConvertToSerializableKeypoint(kp):
    new_kp = SerializableKeypoint()
    new_kp.pt = kp.pt 
    new_kp.size = kp.size
    new_kp.angle = kp.angle
    new_kp.response = kp.response
    new_kp.octave = kp.octave
    new_kp.class_id = kp.class_id

    return new_kp

def CreateKeypointsAndDescriptors(img):
    global sift
    kp, desc = sift.detectAndCompute(img, None)
    return [kp, desc]

def SaveKeypoints(kp, image_filepath):
    serializable_kps = ConvertToSerializableKeypoints(kp)
    filepath = "{0}.keypoints".format(image_filepath)
    pickle.dump(serializable_kps, open(filepath, 'wb'))
    return filepath

def ConvertToSerializableKeypoints(kps_descs):

    log.publish("type of keypoint descs: {0}".format(kps_descs[1].dtype)) 

    kps_descs[0] = [ConvertToSerializableKeypoint(keypoint) for keypoint in kps_descs[0]]
    kps_descs[1] = kps_descs[1].tolist()
    return kps_descs

if __name__ == '__main__':
    try:
        os.makedirs('images/left')
        os.makedirs('images/right')
    except:
        pass

    find_and_store_features()
