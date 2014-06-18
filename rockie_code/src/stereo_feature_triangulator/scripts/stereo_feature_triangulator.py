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
from stereo_feature_identifier_db import Stereo_Pair_Keypoints, Stereo_Pair_Keypoint_Matches, Stereo_3D_Matches, Stereo_Image_Pair, Base
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
triangulator_visualization_topic = '/my_stereo/3d_points'

pub = rospy.Publisher(stereo_feature_triangulator_topic, String)
pub_vis = rospy.Publisher(triangulator_visualization_topic, ros_image)

bridge = CvBridge()

#camera distance is 94 mm
#camera_dist = .094

###DEBUG gazebo cam dist is .4
camera_dist = .4

#focal length is 579 mm
f = 579

hfov = 1.047
img_height = 720
img_width = 960

f_gazebo = img_width / (2*np.tan(hfov / 2))

#set to f_gazebo for debugging
f = f_gazebo

max_vertical_discrepency = 1

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
    
    #cv2.line(vis, pt1, pt2, (int(255*random.random()), int(255*random.random()), int(255*random.random())), 1) 
    z = triangulate(match, kpts1, kpts2)

    if z > 0 and z != float("inf"):
      z_str = str(z)
      z_str = z_str[:3]
      cv2.putText(gray1, z_str, pt1, cv2.FONT_KERSHEY_SIMPLEX, 2, 255, 2)
      cv2.circle(gray1, pt1, 5, (int(z), int(z), int(z)), -1)

  cv2.imshow('img', gray1)

  cv2.waitKey(100)

def triangulate_matches_callback(sp_keypoint_matches_data_id):
  global session
  global DBSession
  global pub

  #get matches
  #get left and right keypoints
  #triangulate matches based on focal length and baseline
  #create 3D point data for each keypoint
  #store to file and db
  
  session = DBSession()
  
  sp_keypoint_matches = get_sp_keypoint_matches(sp_keypoint_matches_data_id.data)
  matches = pickle.load(open(sp_keypoint_matches.sp_keypoint_matches_filepath, "rb"))

  stereo_pair_keypoints = get_keypoints_pair(sp_keypoint_matches.stereo_pair_keypoint_id)

  left_keypoints = get_left_keypoints(stereo_pair_keypoints)
  right_keypoints = get_right_keypoints(stereo_pair_keypoints)

  ############3####For debugging/visualization#############
  left_image = get_left_image(stereo_pair_keypoints.stereo_image_pair_id)

  _3d_points = triangulate(matches, left_keypoints, right_keypoints, left_image)
  
  filepath = store_3d_points(_3d_points, sp_keypoint_matches.sp_keypoint_matches_filepath)

  _3d_matches = Stereo_3D_Matches()
  _3d_matches.sp_matches_id = sp_keypoint_matches.sp_keypoint_matches_id
  _3d_matches.sp_3d_matches_filepath = filepath

  session.add(_3d_matches)
  session.commit()
  sp_3d_matches_id = _3d_matches.sp_3d_matches_id

  pub.publish(str(sp_3d_matches_id))

  session.close()

def get_left_image(image_pair_id):
  global session

  query = session.query(Stereo_Image_Pair)
  stereo_image_pair = query.filter_by(stereo_image_pair_id = int(image_pair_id)).first()
  img = cv2.imread(stereo_image_pair.left_filepath, 0)
  return img

def get_keypoints_pair(spk_id):
  global session
  query = session.query(Stereo_Pair_Keypoints)
  stereo_pair_keypoints = query.filter_by(stereo_pair_keypoint_id = int(spk_id)).first()
  return stereo_pair_keypoints

def triangulate(matches, kpts_descs1, kpts_descs2, left_image):
  global pub_vis
  global max_vertical_discrepency

  kpts1 = kpts_descs1[0]
  descs1 = kpts_descs1[1]

  kpts2 = kpts_descs2[0]
  descs2 = kpts_descs2[1]

  descriptor_length = descs1.shape[1]
  descriptor_dtype = descs1.dtype

  matched_descs = np.empty([0, descriptor_length], dtype=descriptor_dtype)
  positions = np.empty([0, 3], np.float32)

  _3d_points = []

  for match in matches:  

    query_pt = kpts1[match.queryIdx]
    train_pt = kpts2[match.trainIdx]

    #descriptors is a matrix, where each row corresponds
    #to a descriptor vector, and the row index
    #corresponds to the keypoint index
    query_pt_desc = descs1[match.queryIdx, :]
    train_pt_desc = descs2[match.trainIdx, :]

    #should check against epipolar line    
    if(math.fabs(query_pt.pt[1] - train_pt.pt[1]) > max_vertical_discrepency):
      continue

    disparity = math.fabs(query_pt.pt[0] - train_pt.pt[0])

    #consider query image as origin
    x = -query_pt.pt[0] + img_width/2
    y = -query_pt.pt[1] + img_height/2

    #see pg 371 and 416 of O'REILLY "Learning OpenCV" 2008
    Z = (f*camera_dist)/disparity
    Y = (y*Z)/f
    X = (x*Z)/f

    #######draw and publish 3d points#########
    draw_3d_point(left_image, query_pt.pt, Z)

    #stack matched descriptor onto matched_descs
    matched_descs = np.vstack((matched_descs, query_pt_desc))

    #add new row to position matrix
    positions = np.vstack((positions, np.array([X, Y, Z])))

  _3d_points = [matched_descs.tolist(), positions.tolist()]

  try:
    pub_vis.publish(ConvertCV2ToROSGrayscale(left_image))
  except CvBridgeError, e:
    print e

  return _3d_points

def draw_3d_point(left_image, pt, z):

  if z != float("inf"):
    cv2.circle(left_image, (int(pt[0]), int(pt[1])), 5, int(z))
    z_str = str(z)
    z_str = z_str[:4]
    cv2.putText(left_image, z_str, (int(pt[0]), int(pt[1])), cv2.FONT_HERSHEY_SIMPLEX, 2, 255, 2)

def ConvertCV2ToROSGrayscale(img):
  global bridge

  color = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
  ros_img = bridge.cv2_to_imgmsg(color, "bgr8")
  return ros_img

  #cv2_img = bridge.imgmsg_to_cv2(img, "bgr8")
  #cv2_img_gray = cv2.cvtColor(cv2_img, cv2.COLOR_BGR2GRAY)
  #return cv2_img_gray

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
  kpts_descs[1] = np.array(kpts_descs[1], np.float32)

  return kpts_descs

def get_right_keypoints(stereo_pair_keypoint):
  right_keypoints_filepath = "{0}".format(stereo_pair_keypoint.right_keypoints_filepath)

  kpts_descs = pickle.load(open(right_keypoints_filepath, "rb"))
  kpts_descs[0] = [recreate_keypoints(kp) for kp in kpts_descs[0]]
  kpts_descs[1] = np.array(kpts_descs[1], np.float32)

  return kpts_descs

def get_sp_keypoint_matches(sp_keypoint_matches_id):
  global session
  query = session.query(Stereo_Pair_Keypoint_Matches)
  sp_keypoint_matches = query.filter_by(sp_keypoint_matches_id = int(sp_keypoint_matches_id)).first()
  return sp_keypoint_matches

def triangulate_matches():
  rospy.init_node("stereo_feature_match_triangulator")
  rospy.Subscriber(stereo_feature_matcher_topic, String, triangulate_matches_callback)
  rospy.spin()

class SerializableKeypoint():
  pass

if __name__ == '__main__':
  triangulate_matches()

  '''

  matches_filepath = '/home/will/Code/wpi-sample-return-robot-challenge/rockie_code/src/stereo_historian/scripts/images/left/1401674393991712601.jpg.keypoints.keypoint_matches'

  left_kpts_filepath = '/home/will/Code/wpi-sample-return-robot-challenge/rockie_code/src/stereo_historian/scripts/images/left/1401674393991712601.jpg.keypoints'

  right_kpts_filepath = '/home/will/Code/wpi-sample-return-robot-challenge/rockie_code/src/stereo_historian/scripts/images/right/1401674393991712601.jpg.keypoints'

  matches = np.load(open(matches_filepath))
  right_ke


  #matches = pickle.load(open(matches_filepath))
  #right_kpts = pickle.load(open(right_kpts_filepath))
  #left_kpts = pickle.load(open(left_kpts_filepath))

  triangulate(matches, left_kpts, right_kpts)
  '''
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
      

