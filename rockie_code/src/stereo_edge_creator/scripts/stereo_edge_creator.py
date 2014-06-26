#!/usr/bin/python

import numpy as np
import cv2
import math
import cv
import os
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image as ros_image
from cv_bridge import CvBridge, CvBridgeError
from stereo_feature_identifier_db import Stereo_Pair_Keypoints, Stereo_Pair_Keypoint_Matches, Stereo_3D_Matches, Graph_Nodes, Graph_Edges, Stereo_Image_Pair, Base
import datetime
import cPickle as pickle
from sqlalchemy import create_engine
import random
from sqlalchemy.orm import sessionmaker

engine = create_engine('mysql://root@localhost/rockie')
Base.metadata.bind = engine
DBSession = sessionmaker(bind=engine)
session = DBSession()

node_creator_topic = '/my_stereo/new_nodes'
pub = rospy.Publisher('my_stereo/new_edges', String)

matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

#if we can match 70% of the 3D points, don't create a new feature node
new_feature_threshold = .7

#If we have at least 7 matches, make a connection
new_connection_threshold = 1#15 
ransac_sample_size = 6 
ransac_iterations = 400 

stereo_imagepath_base = "{0}/Code/wpi-sample-return-robot-challenge/rockie_code/src/stereo_historian/scripts/images/left/".format(os.getenv("HOME"))

min_wpi_feature_matches = 3
min_error_per_point_threshold = 1 

def create_edges_callback(node_id_data):
  global session
  global DBSession
  global pub

  new_node = get_node_by_node_id(node_id_data.data)
  all_nodes = get_all_nodes()

  for node in all_nodes:
    attempt_create_edge(new_node, all_nodes)

def get_3d_matches_object(_3d_matches):
  obj = pickle.load(open(_3d_matches.sp_3d_matches_filepath))
  return obj

def get_3d_matches_obj_by_node(node):
  global session

  _3d_matches = get_3d_points(node.sp_3d_matches_id)
  return get_3d_matches_object(_3d_matches)

def get_3d_point_matches(descs1, descs2):
  global matcher
  matches = matcher.match(descs1, descs2)
  return matches

def get_3d_points(_3d_matches_id):
  global session

  session.commit()

  query = session.query(Stereo_3D_Matches)
  _3d_matches = query.filter_by(sp_3d_matches_id = _3d_matches_id).first()

  return _3d_matches

def match_node_points(node_1, node_2):
  [node_1_descs, node_1_points] = get_3d_matches_obj_by_node(node_1)
  [node_2_descs, node_2_points] = get_3d_matches_obj_by_node(node_2)

  node_1_descs = np.matrix(node_1_descs, np.uint8)
  node_2_descs = np.matrix(node_2_descs, np.uint8)
  node_1_points = np.matrix(node_1_points, np.float32)
  node_2_points = np.matrix(node_2_points, np.float32)

  matches = get_3d_point_matches(node_1_descs, node_2_descs)
  matches = sorted(matches, key = lambda x:x.distance) 
  
  if len(matches) > ransac_sample_size:
    return [matches, node_1_points, node_2_points]
  else:
    return None

def ransac_matches(matches, positions_1, positions_2):

  #following http://en.wikipedia.org/wiki/Kabsch_algorithm
  #also check out https://github.com/charnley/rmsd
  ransac_matches = rand_sampled_matches(matches, ransac_sample_size)

  [rand_positions_1, rand_positions_2] = get_rand_positions(ransac_matches, 
      positions_1, 
      positions_2)

  rand_positions_1 = np.asmatrix(rand_positions_1)  
  rand_positions_2 = np.asmatrix(rand_positions_2)

  return [rand_positions_1, rand_positions_2]

def get_centroid(positions):
  return np.mean(positions, axis=0).transpose()

def subtract_centroid(P, centroid):
  #subtract centroid from each position
  P[:, 0] -= centroid[0]
  P[:, 1] -= centroid[1]
  P[:, 2] -= centroid[2]

  return P

def calculate_transform_error(R, t, positions_1, positions_2):

  #add error to R if it is very oblique
  #(since large angles of R are unlikely)
  [axis, theta] = get_axis_angle(R)
  angle_likelihood = theta**2

  cum_error = 0
  num_points = positions_1.shape[0]

  for i in range(num_points):
    pt_1 = np.asmatrix(positions_1[i, :])
    pt_2 = np.asmatrix(positions_2[i, :])
    
    pt_1 = pt_1.transpose()
    pt_2 = pt_2.transpose()

    transformed_pt_1 = (np.dot(R, pt_1)) + t
    cum_error += np.linalg.norm((transformed_pt_1 - pt_2))**2

  return cum_error*angle_likelihood
  #return cum_error

def kabsch(P, Q):

  #A is the covariance matrix
  A = np.dot(np.transpose(P), Q)
  
  try:
    [V, S, W_t] = np.linalg.svd(A)
  except:
    return

  ident = np.identity(3)

  W = np.transpose(W_t)
  V_t = np.transpose(V)

  d = np.sign(np.linalg.det(np.dot(W, V_t)))
  ident[2, 2] = d

  return np.dot(W, np.dot(ident, V_t))

def calculate_3d_transform(matches, positions_1, positions_2):

  min_error = float("inf")

  centroid_1 = np.zeros([3, 1])
  centroid_2 = np.zeros([3, 1])

  opt_t = np.zeros([3, 1])
  opt_R = np.zeros([3, 3])

  for i in range(ransac_iterations):
    [rand_positions_1, rand_positions_2] = ransac_matches(matches, positions_1, positions_2)

    orig_rand_positions_1 = np.copy(rand_positions_1)
    orig_rand_positions_2 = np.copy(rand_positions_2)

    centroid_1 = get_centroid(rand_positions_1)
    centroid_2 = get_centroid(rand_positions_2)

    #subtract centroid from each position
    rand_positions_1 = subtract_centroid(rand_positions_1, centroid_1)
    rand_positions_2 = subtract_centroid(rand_positions_2, centroid_2)

    '''

    #TODO: Points further away will have a lot of error.
    # to compensate, define a new metric = centroid_norm/variance
    # the higher this number, the higher the expected error in the 
    # rotation matrix. 
    # (high centroid norm means the points are far away and less accurate, but high
    # variance means the points are very spread out which increases accuracy)
    centroid_norm_1 = np.norm(centroid_1)
    centroid_norm_2 = np.norm(centroid_2)
    max_norm = np.max(centroid_norm_1, centroid_norm_2) 
    std_dev = np.std(A, axis=0)
    expected_R_error = max_norm/std_dev
    
    #TODO: Ensure that ransac points aren't colinear 
    #TODO: Ensure that if ransac points are co-planar, the flipped rotation
    #matrix is not selected

    '''

    R = kabsch(rand_positions_1, rand_positions_2)
    #R = np.identity(3)

    t = -R*centroid_1 + centroid_2
    
    error = calculate_transform_error(R, t, orig_rand_positions_1, orig_rand_positions_2)

    if(error < min_error):
      opt_t = t
      opt_R = R
      opt_centroid_1 = centroid_1
      opt_centroid_2 = centroid_2
      min_error = error
      opt_rand_positions_1 = orig_rand_positions_1
      opt_rand_positions_2 = orig_rand_positions_2
      
 
  if(min_error/ransac_sample_size < min_error_per_point_threshold):
    pass
    #log.publish("rand_positions_1: {0}".format(opt_rand_positions_1))
    #log.publish("rand_positions_2: {0}".format(opt_rand_positions_2))

    #cum_t += (opt_centroid_2 - opt_centroid_1 )
    #log.publish("cumulative t = {0}".format(cum_t))
    #log.publish("opt R = {0}".format(opt_R))
    #log.publish("opt t = {0}".format(opt_t))
    #log.publish("opt error/point = {0}".format(min_error/ransac_sample_size))
    #log.publish("centroid 1 = {0}".format(opt_centroid_1))
    #log.publish("centroid 2 = {0}".format(opt_centroid_2))
    #log.publish("centroid 2 - 1 = {0}".format(opt_centroid_2 - opt_centroid_1))


  #print("rand_positions_1: {0}".format(opt_rand_positions_1))
  #print("rand_positions_2: {0}".format(opt_rand_positions_2))


  return [opt_R.tolist(), opt_t.tolist(), min_error/ransac_sample_size]

def save_transform(transform, node_1, node_2):
  filepath = "{0}node_{1}_to_node_{2}_edge.transform".format(stereo_imagepath_base, node_1.node_id, node_2.node_id)
  pickle.dump(transform, open(filepath, 'wb'))
  return filepath

def create_edge(node_1, node_2, R, t)
  global session

  edge = Graph_Edges()
  edge.node_1_id = node_1.node_id
  edge.node_2_id = node_2.node_id
  
  transform = [R, t]
  transform_filepath = save_transform(transform, node_1, node_2)
  edge.opt_transform_1_to_2_filepath = transform_filepath

  #TODO: do we need to save these?
  '''
  _3d_matches_filepath = save_3d_matches(point_matches,
      pose_3d_points, 
      feature_3d_points, 
      new_pose_node.node_id, 
      feature_node.node_id, 
      transform)

  edge._3d_matches_filepath = _3d_matches_filepath
  '''

  session.add(edge)
  session.commit()
  return edge

def attempt_create_edge(node_1, node_2):

  matched_points = match_node_points(node_1, node_2)

  if matched_points is not None:
    [matches, node_1_points, node_2_points] = matched_points
    [R, t, err] = calculate_3d_transform(matches, node_1_points, node_2_points)
    edge = create_edge(node_1, node_2, R, t)
    return edge

  return None

def get_all_nodes():
  global session

  session.commit()
  query = session.query(Graph_Nodes)
  return query.first()

def get_node_by_node_id(node_id):
  global session

  session.commit()
  query = session.query(Graph_Nodes).filter(Graph_Nodes.node_id == node_id)
  return query.first()

def create_edges():
  rospy.init_node("stereo_edge_creator")
  rospy.Subscriber(node_creator_callback, String, create_edges_callback)
  rospy.spin()

if __name__ == '__main__':
  create_edges()
