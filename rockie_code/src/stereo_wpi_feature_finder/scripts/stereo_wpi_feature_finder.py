#!/usr/bin/python
'''
This node subscribes to the stereo image historian node, and every time a stereo image
pair is saved it will perform wpi feature matching on it. It will then save features to the
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
from stereo_feature_identifier_db import Stereo_Pair_Keypoints, Stereo_Pair_Keypoint_Matches, Stereo_3D_Matches, Graph_Nodes, Graph_Edges
from stereo_historian_db import Stereo_Image_Pair, Base
import datetime
import cPickle as pickle
from sqlalchemy import create_engine
import random
from sqlalchemy.orm import sessionmaker

engine = create_engine('mysql://root@localhost/rockie')
Base.metadata.bind = engine
DBSession = sessionmaker(bind=engine)
session = DBSession()

stereo_historian_topic = '/my_stereo/stereo_image_saves'
stereo_feature_identifier_topic = '/my_stereo/stereo_image_keypoint_saves'
stereo_feature_matcher_topic = '/my_stereo/stereo_image_keypoint_matches'
stereo_feature_triangulator_topic = '/my_stereo/stereo_image_3D_points'
stereo_graph_manager_topic = '/my_stereo/stereo_graph_node_updates'

#pillar_1 is feature 1020
#pillar_2 is feature 1021
#East is X, North is Y, Z is Z
pillar_1_location = [60.33, 40.72, 0]
pillar_2_location = [70.97, 40.23, 0]

#stage column 1 is feature 1002
#stage column 2 is feature 1003
#stage column 3 is feature 1004
stage_column_1_location = [60.88, -29.33, 0]
stage_column_2_location = [51.12, -16.44, 0]
stage_column_3_location = [42.43, -23.02, 0]

stereo_imagepath_base = "{0}/Code/wpi-sample-return-robot-challenge/rockie_code/src/stereo_historian/scripts/images/left/".format(os.getenv("HOME"))

def add_new_wpi_feature_node(_3d_matches):
  global session
  feature_node = Graph_Nodes()
  feature_node.node_type = 'wpi_feature'
  feature_node.sp_3d_matches_id = _3d_matches.sp_3d_matches_id

  session.add(feature_node)
  session.commit()
  return feature_node

def create_stage_1_node(path):
  pillar_1_top = 250
  pillar_1_bottom = 424
  pillar_1_left = 67
  pillar_1_right = 97

  pillar_2_top = 250 
  pillar_2_bottom = 420
  pillar_2_left = 663
  pillar_2_right = 693

  box_1 = [pillar_1_top, pillar_1_bottom, pillar_1_left, pillar_1_right]
  box_2 = [pillar_2_top, pillar_2_bottom, pillar_2_left, pillar_2_right]
  boxes = [box_1, box_2]
  locations = [pillar_1_location, pillar_2_location]

  create_feature_node_boxes(boxes, locations, path)

def between(arg, num_1, num_2):
  if (num_1 <= arg <= num_2) or (num_1 >= arg >= num_2):
  return True
  else:
  return False

def create_two_pillars_1_node(path):
  pillar_1_top = 96
  pillar_1_bottom = 440
  pillar_1_left = 114
  pillar_1_right = 157

  pillar_2_top = 27
  pillar_2_bottom = 478
  pillar_2_left = 506
  pillar_2_right = 586

  box_1 = [pillar_1_top, pillar_1_bottom, pillar_1_left, pillar_1_right]
  box_2 = [pillar_2_top, pillar_2_bottom, pillar_2_left, pillar_2_right]
  boxes = [box_1, box_2]
  locations = [pillar_1_location, pillar_2_location]

  create_feature_node_boxes(boxes, locations, path)

def create_two_pillars_2_node(path):
  pass

def get_pixel_coords(pixel_pt, boxes, locations):

  #100 pixels/meter
  pixel_scale = .01
  for i, box in boxes:

    [top, bottom, left, right] = box
    [x, y] = pixel_pt

    if between(y, top, bottom) and between(x, left, right):
      coordinates = locations[i]

      #fudge the z
      coordinates[2] = (bottom - y)*pixel_scale

      return coordinates

  return [None, None, None] 

#a box is [top, bottom, left, right]
#a location is [x, y, z]
def create_feature_node_boxes(boxes, locations, path):
   _3d_points = []

  img = cv2.imread(path)
  img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

  all_kpts, all_descs = sift.detectAndCompute(img_gray, None)
  descriptor_length = all_descs.shape[1]
  descriptor_dtype = all_descs.dtype

  _3d_descs = np.empty([0, descriptor_length], dtype=descriptor_dtype)
  positions = np.empty([0, 3])

  for index, kp in all_kpts:
    desc = all_descs[index, :]
    
    #= [None, None, None] if no wpi coordinates in feature
    coordinates = get_pixel_coords(kp.pt, boxes, locations)

    positions = np.vstack((positions, coordinates))
    _3d_descs = np.vstack((_3d_descs, desc))

  _3d_keypoints = [_3d_descs.tolist(), positions.tolist()]

  #store 3d points
  filepath = store_3d_points(_3d_keypoints, path)

  #technically not stereo matches, but pretend it is
  _3d_matches = Stereo_3D_Matches()
  _3d_matches.sp_3d_matches_filepath = filepath
  session.add(_3d_matches)
  session.commit()

  add_new_wpi_feature_node(_3d_matches)

  session.close()

def store_3d_points(_3d_points, filepath):
  filepath = "{0}.3d_points".format(filepath)
  pickle.dump(_3d_points, open(filepath, 'wb'))
  return filepath

def create_stage_2_node(path):
  pillar_1_top = 335
  pillar_1_bottom = 515
  pillar_1_left = 140
  pillar_1_right = 172

  pillar_2_top = 200
  pillar_2_bottom = 545
  pillar_2_left = 220
  pillar_2_right = 290

  pillar_3_top = 250
  pillar_3_bottom = 535
  pillar_3_left = 985
  pillar_3_right = 1042

  box_1 = [pillar_1_top, pillar_1_bottom, pillar_1_left, pillar_1_right]
  box_2 = [pillar_2_top, pillar_2_bottom, pillar_2_left, pillar_2_right]
  box_3 = [pillar_3_top, pillar_3_bottom, pillar_3_left, pillar_3_right]
  boxes = [box_1, box_2, box_3]
  locations = [stage_column_1_location, stage_column_2_location, stage_column_3_location]

  create_feature_node_boxes(boxes, locations, path)

def create_wpi_feature_nodes(folder):
  create_stage_1_node(folder + "stage_1.png")
  create_stage_2_node(folder + "stage_2.png")

  create_two_pillars_1_node(folder + "two_pillars_1.png")
  create_two_pillars_2_node(folder + "two_pillars_2.png")

  #TODO: create a one pillar node in case we can't match both?
  #create_one_pillar_node(folder + "stage_1.png")

if __name__ == '__main__':
  wpi_feature_folder = "{0}/Code/wpi-sample-return-robot-challenge/wpi_features/feature_images/".format(os.getenv("HOME"))
  wpi_starting_platform_folder = "{0}/Code/wpi-sample-return-robot-challenge/wpi_features/platform_images/".format(os.getenv("HOME"))

  if(wpi_feature_nodes_not_created()):
  create_wpi_feature_nodes(wpi_feature_folder)
  create_wpi_platform_node(wpi_starting_platform_folder)

  #check db for wpi feature nodes
  #if none:
  #create 3d keypoints for wpi features
  #create wpi features nodes from these collections
  #generate wpi feature nodes
  #add to db
  #ensure that the wpi feature node flag is set

  #update_graph()

