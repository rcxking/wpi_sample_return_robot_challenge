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
import datetime
import cPickle as pickle

stereo_imagepath_base = '/home/will/Code/wpi-sample-return-robot-challenge/rockie_code/src/stereo_historian/scripts/'

stereo_historian_topic = '/my_stereo/stereo_image_saves'
def create_and_store_features_callback(filepath):
    img = cv2.imread("{0}{1}".format(stereo_imagepath_base, filepath.data), 0)
    keypoints = CreateKeypoints(img)
    SaveKeypoints(keypoints, filepath)

def find_and_store_features():
    rospy.init_node("stereo_feature_identifier")
    rospy.Subscriber(stereo_historian_topic, String, create_and_store_features_callback)
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
    #return kp

def CreateKeypoints(img):
    kp = sift.detect(img, None)
    return kp

def SaveKeypoints(kp, filepath):
    serializable_kps = ConvertToSerializableKeypoints(kp)
    pickle.dump(serializable_kps, open("{0}.keypoints".format(filepath.data), 'wb'))

def ConvertToSerializableKeypoints(kp):
    return [ConvertToSerializableKeypoint(keypoint) for keypoint in kp]

if __name__ == '__main__':
    sift = cv2.SIFT()
    find_and_store_features()
    #img = cv2.imread('balloons.jpg', 0)
    #kp = sift.detect(img, None)
    #serializable_kp = ConvertToSerializableKeypoints(kp)
    #pickle.dump(serializable_kp, open('testdump.dump', 'wb'))
    #cv2.drawKeypoints(img, kp)
    #cv2.imshow('image', img)
    #cv2.waitKey(0)
