#!/usr/bin/python
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
from stereo_feature_identifier_db import Stereo_Pair_Keypoints, Stereo_Pair_Keypoint_Matches, Stereo_3D_Matches, Graph_Nodes, Graph_Edges, Stereo_Image_Pair, Base
import datetime
import cPickle as pickle
from sqlalchemy import create_engine, and_
import random
from sqlalchemy.orm import sessionmaker
#import calculate_rmsd

cum_t = np.zeros([3, 1])

engine = create_engine('mysql://root@localhost/rockie')
Base.metadata.bind = engine
DBSession = sessionmaker(bind=engine)
session = DBSession()

stereo_historian_topic = '/my_stereo/stereo_image_saves'
stereo_feature_identifier_topic = '/my_stereo/stereo_image_keypoint_saves'
stereo_feature_matcher_topic = '/my_stereo/stereo_image_keypoint_matches'
stereo_feature_triangulator_topic = '/my_stereo/stereo_image_3D_points'
stereo_graph_manager_topic = '/my_stereo/stereo_graph_node_updates'

#flann = cv2.FlannBasedMatcher(index_params,search_params)
flann = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

#if we can match 70% of the 3D points, don't create a new feature node
new_feature_threshold = .7

#If we have at least 7 matches, make a connection
new_connection_threshold = 1#15 
ransac_sample_size = 6 
ransac_iterations = 400 

stereo_imagepath_base = "{0}/Code/wpi-sample-return-robot-challenge/rockie_code/src/stereo_historian/scripts/images/left/".format(os.getenv("HOME"))

log = rospy.Publisher("/stereo_graph_manager/log", String)

min_wpi_feature_matches = 3
min_error_per_point_threshold = 1 

##DEBUG

is_debug = True
initial_node_set = False

def update_graph_callback(_3d_matches_data_id):
  global session
  global DBSession
  global pub

  _3d_matches = get_3d_points(_3d_matches_data_id.data)

  if(_3d_matches != None):

    stereo_keypoint_matches = get_sp_keypoint_matches(_3d_matches.sp_matches_id)
    stereo_keypoints = get_stereo_pair_keypoint(stereo_keypoint_matches.stereo_pair_keypoint_id)
    stereo_image_pair = get_stereo_image_pair(stereo_keypoints.stereo_image_pair_id)

    update_slam_graph(_3d_matches, stereo_image_pair)

  #create pose node
  #connect pose node to previous pose node
  #use wheel odometry to get edge transform for this new edge

  #iterate through prior 3d_matches
  #if there is a 3d_match-3d_match (> min num of matches):
    #connect newly created pose node to found 3d_match

    #if we've found a 3d_match-3d_match match that share enough nodes
    #don't add new 3d_matches as new feature node
    #else
    #add new 3d_matches as new feature node to graph
    #connect this new node to new pose node

  ################For more info, look at Thrun's graphslam paper http://robots.stanford.edu/papers/thrun.graphslam.pdf############

  #Every pose-feature-pose edgewise connection can be reduced to a pose-pose edge (constraint)
  #NOTE: Don't reduce a pose-wpi-feature-node-pose edges to a pose-pose edge, this will
  #discard global info

  #We can then reduce our pose-pose/pose-feature graph to just a pose-pose graph
  #A rigid body transformation (rotation followed by translation) defines this constraint

  #The constraint error is defined as the square distance between each 3d point match after transformation

  #Once this is done we can at least recover odometry (pose-pose constraints describe motion between poses)

  #To recover absolute position, we first optimize our pose-pose constraints (least-squares/libg20)
  #then, we find a wpi feature node in the graph. all pose-nodes that have an edge to this feature
  #node(i.e. have seen the wpi feature) are given an absolute/global location.

  #From these nodes, we propogate out the optimized transformations between pose-pose-nodes
  #we should be able to just find a path through the graph to our current position, and apply the
  #transformation along the path to recover the global position

def get_stereo_image_pair(stereo_image_pair_id):
  global session

  query = session.query(Stereo_Image_Pair)
  stereo_image_pair = query.filter_by(stereo_image_pair_id = int(stereo_image_pair_id)).first()
  return stereo_image_pair

def get_stereo_pair_keypoint(stereo_pair_keypoint_id):
  global session
  query = session.query(Stereo_Pair_Keypoints)
  stereo_pair_keypoint = query.filter_by(stereo_pair_keypoint_id = int(stereo_pair_keypoint_id)).first()
  return stereo_pair_keypoint

def get_sp_keypoint_matches(sp_keypoint_matches_id):
  global session
  query = session.query(Stereo_Pair_Keypoint_Matches)
  sp_keypoint_matches = query.filter_by(sp_keypoint_matches_id = int(sp_keypoint_matches_id)).first()
  return sp_keypoint_matches

def get_3d_matches_object(_3d_matches):
  obj = pickle.load(open(_3d_matches.sp_3d_matches_filepath))
  return obj

def try_connect_nodes_wpi_feature(feature_node, new_point_descs, new_point_positions, new_pose_node):
  global min_wpi_feature_matches

  feature_node_3d_points = get_3d_points(feature_node.sp_3d_matches_id)
  feature_node_3d_points_obj = get_3d_matches_object(feature_node_3d_points)
  [fn_descs, fn_positions] = feature_node_3d_points_obj

  insert_new_feature_node = True

  if(len(fn_positions) > ransac_sample_size):

    feature_point_descs = np.array(fn_descs, np.float32)
    feature_point_positions = np.array(fn_positions, np.float32)

    point_matches = get_3d_point_matches(new_point_descs, feature_point_descs)

    
    num_3d_matches = len(point_matches)
    num_feature_node_points = feature_point_positions.shape[0]

    #Returns true if we have enough matches to connect new pose to existing feature, false otherwise
    if(num_3d_matches > min_wpi_feature_matches):

      transform = calculate_wpi_3d_transform(point_matches, 
          new_point_positions, 
          feature_point_positions)

      connect_pose_to_feature(new_pose_node, 
          feature_node, 
          transform, 
          point_matches, 
          new_point_positions, 
          feature_point_positions)

def try_connect_nodes(feature_node, new_point_descs, new_point_positions, new_pose_node):

  feature_node_3d_points = get_3d_points(feature_node.sp_3d_matches_id)
  feature_node_3d_points_obj = get_3d_matches_object(feature_node_3d_points)
  [fn_descs, fn_positions] = feature_node_3d_points_obj

  insert_new_feature_node = True

  if(len(fn_positions) > ransac_sample_size):

    #feature_point_descs = np.array(fn_descs, np.float32)
    feature_point_descs = np.array(fn_descs, np.uint8)
    feature_point_positions = np.array(fn_positions, np.float32)

    point_matches = get_3d_point_matches(new_point_descs, feature_point_descs)

    log.publish("attempting to connect node {0} to node {1}".format(new_pose_node.node_id, feature_node.node_id))
    log.publish ("num point matches = {0}".format(len(point_matches)))
    #log.publish("point_matches: {0}".format(point_matches))

    num_3d_matches = len(point_matches)
    num_feature_node_points = feature_point_positions.shape[0]

    #Returns true if we have enough matches to connect new pose to existing feature, false otherwise
    if(num_3d_matches > new_connection_threshold):

      #TODO: If the error for this is higher than we would
      #expect, we should disregard it and not add it to the
      #graph
      result = calculate_3d_transform(point_matches, new_point_positions, feature_point_positions)

      if result != None:
        [R, t, min_error_per_point] = result
      else:
        return

      transform = [R, t]  

      if(min_error_per_point < min_error_per_point_threshold):
        connect_pose_to_feature(new_pose_node, 
            feature_node, 
            transform, 
            point_matches, 
            new_point_positions, 
            feature_point_positions)

      #don't add new feature node if num of matches is relatively large
      if(num_3d_matches > new_feature_threshold*num_feature_node_points):
          insert_new_feature_node = False

  return insert_new_feature_node

def update_slam_graph(_3d_matches, stereo_image_pair):

  #1. get new 3d keypoints node
  #2. get all existing nodes
  #3. attempt to connect new node to all existing nodes
  #4. if we've connected to a node with many similar points, 
  # dont add new feature node, otherwise, add new feature node

  #new_node = create_pose_node(_3d_matches)
  #existing_nodes = get_all_feature_nodes()
  #insert_new_feature_nodes = try_connect_nodes(new_node, existing_nodes)
  #if insert_new_feature_node then insert new feature node

  _3d_matches_object = get_3d_matches_object(_3d_matches)
  [new_point_descs, new_point_positions] = _3d_matches_object

  #new_point_descs = np.array(new_point_descs, np.float32)
  new_point_descs = np.array(new_point_descs, np.uint8)
  new_point_positions = np.array(new_point_positions, np.float32)

  #TODO: Connect poses with wheel odometry
  new_pose_node = create_pose_node(stereo_image_pair.stereo_image_pair_id)  
  #connect_poses(new_pose_node.node_id, new_pose_node.node_id - 1)

  #TODO: Check most recent frame against ALL prior frames.
  #for now, just check against last frame (odometry mode)

  all_non_wpi_feature_nodes = get_all_feature_nodes(is_wpi_feature_node=False)
  #all_wpi_feature_nodes = get_all_feature_nodes(is_wpi_feature_node=True)

  log.publish("number of feature nodes: {0}".format(len(all_non_wpi_feature_nodes)))

  insert_new_feature_node = True
  '''

  last_feature_node = get_last_feature_node(new_pose_node)

  if last_feature_node is not None:
    #log.publish("found previous node, attempting to connect to current node")
    if(try_connect_nodes(last_feature_node, new_point_descs, new_point_positions, new_pose_node)):
      insert_new_feature_node = False

  '''

  #TODO: Attempt to match against wpi feature nodes for global coords,
  #and other nodes for soft edges
  '''
  for feature_node in all_wpi_feature_nodes:
    if(try_connect_nodes_wpi_feature(feature_node, new_point_descs, new_point_positions, new_pose_node)):
      #if we successfully match a wpi feature, connect and return
      return
  '''
  #Attempt to match against a previous non-wpi node
  for feature_node in all_non_wpi_feature_nodes:
    if(try_connect_nodes(feature_node, new_point_descs, new_point_positions, new_pose_node)):
      insert_new_feature_node = False

  #Nothing matched our incoming keypoints completely,
  #so add this as a new node for future matching
  if(insert_new_feature_node):
    new_feature_node = add_new_feature_node(_3d_matches)
    identity_transform = get_identity_transform()
    connect_pose_to_feature(new_pose_node, 
        new_feature_node, 
        identity_transform, 
        None, 
        new_point_positions, 
        new_point_positions)

def get_identity_transform():
  return [np.identity(3).tolist(), np.zeros([1, 3]).tolist()]

def add_new_feature_node(_3d_matches):
  global session
  feature_node = Graph_Nodes()
  feature_node.node_type = 'feature'
  feature_node.sp_3d_matches_id = _3d_matches.sp_3d_matches_id

  session.add(feature_node)
  session.commit()
  return feature_node

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

def subtract_centroid(P, centroid):
  #subtract centroid from each position
  P[:, 0] -= centroid[0]
  P[:, 1] -= centroid[1]
  P[:, 2] -= centroid[2]

  return P
  
#Look at github wiki  
def calculate_3d_transform(matches, positions_1, positions_2):
  global cum_t

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


  print("rand_positions_1: {0}".format(opt_rand_positions_1))
  print("rand_positions_2: {0}".format(opt_rand_positions_2))


  return [opt_R.tolist(), opt_t.tolist(), min_error/ransac_sample_size]

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

def is_none_vector(v):

  if v[0] == None and v[1] == None and v[2] == None:
    return True
  else:
    return False

def matches_with_wpi_coordinates(matches, ransac_sample_size, positions_query, positions_train):

  matches_with_coords = []

  for match in matches:
    if not is_none_vector(positions_query[match.queryIdx]) and not is_none_vector(positions_train[match.trainIdx]):
      matches_with_coords.append(match)

  random_indexes = random.sample(range(len(matches_with_coords)), ransac_sample_size)
  return [matches_with_coords[i] for i in random_indexes]

def rand_sampled_matches(matches, ransac_sample_size):

  random_indexes = random.sample(range(len(matches)), ransac_sample_size)
  return [matches[i] for i in random_indexes]

def get_rand_positions(ransac_matches, positions_query, positions_train):
  
  sampled_train_positions = np.zeros([0, 3])
  sampled_query_positions = np.zeros([0, 3])

  for match in ransac_matches:
    query_pt = positions_query[match.queryIdx]
    train_pt = positions_train[match.trainIdx]

    sampled_train_positions = np.vstack((sampled_train_positions, train_pt))
    sampled_query_positions = np.vstack((sampled_query_positions, query_pt))

  return [sampled_query_positions, sampled_train_positions]

def get_centroid(positions):
  return np.mean(positions, axis=0).transpose()

def get_covariance_matrix(positions_1, positions_2, centroid_1, centroid_2):
  H = np.asmatrix(np.zeros([3, 3]))

  for i in range(len(positions_1)):
    point_1 = np.asmatrix(positions_1[i, :])
    point_2 = np.asmatrix(positions_2[i, :])

    point_1 = point_1.transpose()
    point_2 = point_2.transpose()

    '''

    print("point 1 = {0}".format(point_1))
    print("point 2 = {0}".format(point_2))
    print("centroid 1 = {0}".format(centroid_1))
    print("centroid 2 = {0}".format(centroid_2))
    
    '''

    H += (point_1 - centroid_1)*((point_2 - centroid_2).transpose())

  return H

def save_transform(transform, filepath, new_pose_node, feature_node):
  filepath = "{0}node_{1}_to_node_{2}_edge.transform".format(stereo_imagepath_base, new_pose_node.node_id, feature_node.node_id)
  pickle.dump(transform, open(filepath, 'wb'))
  return filepath

def save_global_transform(transform, node):
  filepath = "{0}node_{1}_global_transform.transform".format(stereo_imagepath_base, node.node_id)
  pickle.dump(transform, open(filepath, 'wb'))
  return filepath


def connect_pose_to_feature(new_pose_node, feature_node, transform, point_matches, pose_3d_points, feature_3d_points):
  global session

  edge = Graph_Edges()
  edge.node_1_id = new_pose_node.node_id
  edge.node_1_type = 'pose'
  edge.node_2_id = feature_node.node_id
  edge.node_2_type = 'feature'
  
  transform_filepath = save_transform(transform, None, new_pose_node, feature_node)
  edge.opt_transform_1_to_2_filepath = transform_filepath

  _3d_matches_filepath = save_3d_matches(point_matches,
      pose_3d_points, 
      feature_3d_points, 
      new_pose_node.node_id, 
      feature_node.node_id, 
      transform)

  edge._3d_matches_filepath = _3d_matches_filepath

  session.add(edge)
  session.commit()

class py_match:
  pass

def convert_match(match):
  new_match = py_match()
  new_match.distance = match.distance
  new_match.imgIdx = match.imgIdx
  new_match.queryIdx = match.queryIdx
  new_match.trainIdx = match.trainIdx

  return new_match

def save_3d_matches(point_matches, pose_3d_points, feature_3d_points, pose_id, feature_id, transform):
  if(point_matches != None):
    matches = [convert_match(match) for match in point_matches]
  else:
    matches = None

  _3d_matches = [matches, pose_3d_points.tolist(), feature_3d_points.tolist(), transform]

  filepath = "{0}node_{1}_to_node_{2}_edge.edge".format(stereo_imagepath_base, pose_id, feature_id)
  pickle.dump(_3d_matches, open(filepath, 'wb'))
  return filepath

def create_pose_node(stereo_image_pair_id):
  global session
  global initial_node_set
  global is_debug

  node = Graph_Nodes()
  node.stereo_image_pair_id = stereo_image_pair_id
  node.node_type = 'pose'

  session.add(node)
  session.commit()

  #if we're debugging, set the first node to global coordinates
  if(is_debug and not initial_node_set):
    initial_node_set = True
    T = get_identity_transform()
    filepath = save_global_transform(T, node)
    node.global_transformation_filepath = filepath

  session.commit()
  return node

def get_keypoints_pair(spk_id):
  global session

  session.commit()

  query = session.query(Stereo_Pair_Keypoints)
  stereo_pair_keypoints = query.filter_by(stereo_pair_keypoint_id = int(spk_id)).first()
  return stereo_pair_keypoints

def get_3d_points(_3d_matches_id):
  global session

  session.commit()

  query = session.query(Stereo_3D_Matches)
  _3d_matches = query.filter_by(sp_3d_matches_id = _3d_matches_id).first()
  return _3d_matches

def get_3d_point_matches(descs1, descs2):
  global flann

  matches = flann.match(descs1, descs2)

  return matches

def get_last_feature_node(current_node):
  global session

  session.commit()

  query = session.query(Graph_Nodes).filter(Graph_Nodes.node_id == 2)
  #query = session.query(Graph_Nodes).filter(Graph_Nodes.node_type == 'feature')
  #query = query.order_by(Graph_Nodes.node_id.desc())

  last_node = query.first()
  
  if(last_node != None):
    log.publish("last_node_id: {0}".format(last_node.node_id))
    log.publish("this_node_id: {0}".format(current_node.node_id))

  return last_node

def get_all_feature_nodes(is_wpi_feature_node):
  global session

  session.commit()

  query = session.query(Graph_Nodes)

  if not is_wpi_feature_node:
    feature_nodes = query.filter(Graph_Nodes.node_type == 'feature')
  else:
    feature_nodes = query.filter(Graph_Nodes.node_type == 'wpi_feature')

  return feature_nodes.all()

def triangulate(matches, kpts1, kpts2):

  _3d_points = []

  for match in matches:  

    query_pt = kpts1[match.queryIdx]
    train_pt = kpts2[match.trainIdx]

    #should check against epipolar line    
    if(math.fabs(query_pt.pt[1] - train_pt.pt[1]) > 10):
      continue

    disparity = math.fabs(query_pt.pt[0] - train_pt.pt[0])

    #consider query image as origin
    x = query_pt.pt[0]
    y = query_pt.pt[1]

    #see pg 371 and 416 of O'REILLY "Learning OpenCV" 2008
    Z = (f*camera_dist)/disparity
    Y = (y*Z)/f
    X = (x*Z)/f

    _3d_points.append([match, X, Y, Z])

  return _3d_points

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
  return kpts_descs

def get_right_keypoints(stereo_pair_keypoint):
  right_keypoints_filepath = "{0}".format(stereo_pair_keypoint.right_keypoints_filepath)

  return pickle.load(open(right_keypoints_filepath, "rb"))

def get_sp_keypoint_matches(sp_keypoint_matches_id):
  global session
  query = session.query(Stereo_Pair_Keypoint_Matches)
  sp_keypoint_matches = query.filter_by(sp_keypoint_matches_id = int(sp_keypoint_matches_id)).first()
  return sp_keypoint_matches

def update_graph():
  rospy.init_node("stereo_graph_manager")
  rospy.Subscriber(stereo_feature_triangulator_topic, String, update_graph_callback)
  rospy.spin()

class SerializableKeypoint():
  pass

def get_node(node_id):
  global session

  session.commit()

  query = session.query(Graph_Nodes).filter(Graph_Nodes.node_id == node_id)

  return query.first()

def get_3d_matches_obj_by_node(node):
  global session

  _3d_matches = get_3d_points(node.sp_3d_matches_id)
  return get_3d_matches_object(_3d_matches)

def print_top_matches(matches, pos1, pos2, num_top):
  for i in range(num_top):
    print("pnt 1: {0} --> pnt 2: {1}".format(pos1[matches[i].queryIdx, :], pos2[matches[i].trainIdx, :]))
    

def match_ordered_points(matches, node_start_points, node_end_points):

  P = np.zeros([0, 3], node_start_points.dtype)
  Q = np.zeros([0, 3], node_end_points.dtype)

  for match in matches:
    pnt1 = node_start_points[match.queryIdx, :]
    pnt2 = node_end_points[match.trainIdx, :]

    P = np.vstack((P, pnt1))
    Q = np.vstack((Q, pnt2))

  return [P, Q]

def get_axis_angle(R):
  theta = np.arccos((np.trace(R) - 1)/2)

  a = np.zeros([1, 3])

  a[0, 0] = R[2, 1] - R[1, 2]
  a[0, 1] = R[0, 2] - R[2, 0]
  a[0, 2] = R[1, 0] - R[1, 0]

  axis = np.dot((1/(2*np.sin(theta))), a)

  return [axis, theta]

def get_edge(node_1_id, node_2_id):
  global session

  session.commit()

  query = session.query(Graph_Edges)
  query = query.filter(and_(Graph_Edges.node_1_id == node_1_id, Graph_Edges.node_2_id == node_2_id))

  return query.first()

def get_edge_transform(edge):
  global session

  filepath = edge.opt_transform_1_to_2_filepath

  [R, t] = pickle.load(open(filepath))
  return [R, t] 

if __name__ == '__main__':

  node_start = get_node(10)
  node_end = get_node(40)

  [node_start_descs, node_start_points] = get_3d_matches_obj_by_node(node_start)
  [node_end_descs, node_end_points] = get_3d_matches_obj_by_node(node_end)

  node_start_descs = np.matrix(node_start_descs, np.uint8)
  node_end_descs = np.matrix(node_end_descs, np.uint8)
  node_start_points = np.matrix(node_start_points, np.float32)
  node_end_points = np.matrix(node_end_points, np.float32)

  matches = get_3d_point_matches(node_start_descs, node_end_descs)
  matches = sorted(matches, key = lambda x:x.distance) 

  print_top_matches(matches, node_start_points, node_end_points, 10)

  [R, t, err] = calculate_3d_transform(matches, node_start_points, node_end_points)

  '''
  edge = get_edge(16, 40)
  [R, t] = get_edge_transform(edge)
  '''

  R = np.asmatrix(R)
  R_t = np.transpose(R)

  [axis, theta] = get_axis_angle(R)

  print("R: {0}".format(R))
  print("t: {0}".format(t))
 
  print("Axis: {0}".format(axis))
  print("Angle: {0}".format(theta))
   
  zero = np.zeros([3, 1])

  origin = np.dot(R, zero) + t
  R_t = np.transpose(R)
  origin_2 = np.dot(R_t, np.negative(t))

  print("origin 1 in 2 frame: {0}".format(origin))
  print("origin 2 in 1 frame: {0}".format(origin_2))

