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
import datetime
import cPickle as pickle
from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker

stereo_imagepath_base = "{0}/Code/wpi-sample-return-robot-challenge/rockie_code/src/stereo_historian/scripts/".format(os.getenv("HOME"))

sift = cv2.SIFT()

engine = create_engine('mysql://root@localhost/rockie')
Base.metadata.bind = engine
DBSession = sessionmaker(bind=engine)
session = DBSession()

stereo_historian_topic = '/my_stereo/stereo_image_saves'
stereo_feature_identifier_topic = '/my_stereo/stereo_image_keypoint_saves'

pub = rospy.Publisher(stereo_feature_identifier_topic, String)

def find_and_report_obstacles_callback(stereo_image_pair_data_id):
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

  img_left_blur = cv2.GaussianBlur(img_left, (5, 5), 0)
  img_right_blur = cv2.GaussianBlur(img_right, (5, 5), 0)
  
  [h, w] = img_left.shape

  #cv2.imshow('left', img_left_blur)
  #cv2.imshow('right', img_right_blur)
  #cv2.imshow('img', img_diff)

  max_disparity = 500
  num_erodes_dilates = 1 

  kernel = np.ones((5,5), np.uint8)

  left_obstacles_color = np.zeros((h, w, 3), np.uint8)
  left_obstacles_gray = np.zeros((h, w), np.uint8)

  img_left_blur_color = cv2.cvtColor(img_left_blur, cv2.COLOR_GRAY2RGB)

  for i in range(max_disparity):
    padding = np.empty([h, 1], dtype='float32')

    #img_left_blur = np.hstack((img_left_blur, padding))
    #img_right_blur = np.hstack((padding, img_right_blur))
    #img_diff = np.absolute(img_left_blur - img_right_blur)

    img_left = np.hstack((img_left, padding))
    img_right = np.hstack((padding, img_right))
    img_diff = np.absolute(img_left - img_right)

    ret, thresh = cv2.threshold(img_diff, 1, 255, cv2.THRESH_BINARY_INV)

    eroded = cv2.erode(thresh, kernel, iterations = num_erodes_dilates)

    left_obstacles_gray += eroded[:, :w]
    #img_diff = cv2.dilate(img_diff, kernel, iterations = num_erodes_dilates)
    
    #cv2.imshow('thresh', thresh)
    #cv2.imshow('erode', eroded)
    #cv2.imshow('left_obs', left_obstacles_gray)

    #cv2.waitKey(100)

  ret, left_obstacles_gray = cv2.threshold(left_obstacles_gray, 0, 255, cv2.THRESH_BINARY)

  img_left_blur_color[:, :, 1] = left_obstacles_gray
  cv2.imshow('obs', img_left_blur_color)
  cv2.imshow('gray', left_obstacles_gray)

  cv2.waitKey(0)

  #small gaussian blur on stereo frames

  #determine y mis-alignment by iterating over possible 
  #mis-alignments by sliding stereo frames over each other
  #with different y-alignments
  #and finding cumulative difference
  #minimum difference integral is the correct y calibration

  #now, slide stereo frames against each other
  #for 1:max_disparity, calculate the difference image
    #threshold the difference image to a binary image
    #erode, then dilate to ensure we only select contiguous blobs
    #if a blob still exists, it is an obstacle. mark the x, y and disparity

    #how to filter the ground?


  #re-project into image to confirm
  #publish obstacle location

  #TODO: Can we use x,y,disparity gradient to simplify this?
  #look for places with low x,y gradient, but high values of 2nd disparity derivative?


  session.close()

def find_and_report_obstacles():
  rospy.init_node("stereo_obstacle_detection")
  rospy.Subscriber(stereo_historian_topic, String, find_and_report_obstacles_callback)
  rospy.spin()

class dummy:
  data = 1

if __name__ == '__main__':
  #find_and_report_obstacles()
  
  find_and_report_obstacles_callback(dummy())
