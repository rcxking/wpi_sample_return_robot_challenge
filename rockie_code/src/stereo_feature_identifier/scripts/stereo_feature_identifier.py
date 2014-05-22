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
import Image_Frame_Keypoint

stereo_imagepath_base = '/home/will/Code/wpi-sample-return-robot-challenge/rockie_code/src/stereo_historian/scripts/'

engine = create_engine('mysql://root@localhost/rockie')
Base.metadata.bind = engine
DBSession = sessionmaker(bind=engine)
session = DBSession()

stereo_historian_topic = '/my_stereo/stereo_image_saves'
stereo_feature_identifier_topic = '/my_stereo/stereo_image_keypoint_saves'

def create_and_store_features_callback(image_frame_data_id):
    global session
    
    image_frame_id = image_frame_data_id.data
    image_frame = session.query(Image_Frame.id = image_frame_id).one()

    img_left = cv2.imread("{0}{1}".format(stereo_imagepath_base, image_frame.left_filepath), 0)
    img_right = cv2.imread("{0}{1}".format(stereo_imagepath_base, image_frame.right_filepath), 0)

    left_keypoints = CreateKeypoints(img_left)
    right_keypoints = CreateKeypoints(img_right)
    
    left_img_keypoints_filepath = SaveKeypoints(left_keypoints, 'left')
    right_img_keypoints_filepath = SaveKeypoints(right_keypoints, 'right')

    image_frame_keypoints = Image_Frame_Keypoints()

    image_frame_keypoints.left_keypoints_filepath
    image_frame_keypoints.right_keypoints_filepath
    image_frame_keypoints.image_frame_id = image_frame_id

    session.commit()

    image_frame_keypoints_id = image_frame_keypoints.id

    #publish image_frame_keypoints_id
    
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

def CreateKeypoints(img):
    kp = sift.detect(img, None)
    return kp

def SaveKeypoints(kp, filepath):
    
    serializable_kps = ConvertToSerializableKeypoints(kp)
    pickle.dump(serializable_kps, open("{0}.keypoints".format(filepath.data), 'wb'))

def ConvertToSerializableKeypoints(kp):
    return [ConvertToSerializableKeypoint(keypoint) for keypoint in kp]

if __name__ == '__main__':
    try:
        os.makedirs('images/left')
        os.makedirs('images/right')
    except:
        pass

    sift = cv2.SIFT()
    find_and_store_features()
    #img = cv2.imread('balloons.jpg', 0)
    #kp = sift.detect(img, None)
    #serializable_kp = ConvertToSerializableKeypoints(kp)
    #pickle.dump(serializable_kp, open('testdump.dump', 'wb'))
    #cv2.drawKeypoints(img, kp)
    #cv2.imshow('image', img)
    #cv2.waitKey(0)
