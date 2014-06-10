#!/usr/bin/python
'''
This node is a service interface to slam

a user may call get_absolute_positions, which will:
  - look to see if there are any wpi feature nodes in the graph. if there are:
    - update the latest pose node in the graph (and all intermediate nodes)
    - return the latest pose node
  - if not, return None
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

stereo_imagepath_base = "{0}/Code/wpi-sample-return-robot-challenge/rockie_code/src/stereo_historian/scripts/".format(os.getenv("HOME"))

engine = create_engine('mysql://root@localhost/rockie')
Base.metadata.bind = engine
DBSession = sessionmaker(bind=engine)
session = DBSession()

stereo_historian_topic = '/my_stereo/stereo_image_saves'
stereo_feature_identifier_topic = '/my_stereo/stereo_image_keypoint_saves'
stereo_feature_matcher_topic = '/my_stereo/stereo_image_keypoint_matches'
stereo_feature_triangulator_topic = '/my_stereo/stereo_image_3D_points'
stereo_graph_manager_topic = '/my_stereo/stereo_graph_node_updates'

def get_last_position():
  #get first wpi feature node

  #if no wpi feature node:
    #return "no wpi features found"
  #else
    #percolate transforms out (recursive). find edges connected to this wpi feature node 
    #for each edge:
      #apply percolate_global_transform(edge12, node1, node2)
      #this will apply the transform to node2 from node1, and once 
      #that is done, it will find edges connected to node2, and perform
      #percolate_global_transform for each of those.
      #if node2 has no edges, terminate
  
def get_edge_transform(edge):
  transform_filepath = edge.rigid_body_transform_filepath 
  return pickle.load(open(transform_filepath, "rb"))

def get_node_edges(node, previous_node):
  global session
  query = session.query(Graph_Edges)
  query.filter(Graph_Edges.node_1_id == node.node_id or Graph_Edges.node_2_id == node.node_id)
  edges = query.all()

  #don't return the previous node
  return [new_edges if e.node_1_id != previous_node.node_id and e.node_2_id != previous_node.node_id for e in edges]  

def get_node_by_id(node_id):
  global session
  query = session.query(Graph_Nodes)
  query.filter_by(node_id = int(node_id))

  return query.first()

def get_connected_node_frame_transformation(edge_transform, global_transform):
  return np.dot(edge_transform, global_transform)

def get_node_position(node):
  return [node.x, node.y, node.y]

def get_connected_node(node, edge):
  node_1_id = edge.node_1_id
  node_2_id = edge.node_2_id

  if int(node.node_id) == int(node_1_id):
    return get_node_by_id(node_2_id)
  else:
    return get_node_by_id(node_1_id)

def save_global_transform(transform, node):
  filepath = "{0}node_{1}_global_transform.transform".format(stereo_imagepath_base, node.node_id)
  pickle.dump(transform, open(filepath, 'wb'))
  return filepath

def set_node_global_transform(node, transform):
  global session
  
  filepath = save_global_transform(transform, node)
  node.global_transformation_filepath = filepath

  session.commit()

def percolate_global_transform(edge, node, traversed_edges):
  #node1 has x,y,z. if node2 doesn't have x,y,z, apply transform in edge to get node2 x,y,z.
  #find edges connected to node2. for each connected node, recurse
  
  traversed_edges.append(edge)

  connected_node = get_connected_node(node, edge)

  if connected_node.global_transformation == None:
    #this is the tranform between nodes (node 1 to node 2)
    edge_transform = get_edge_transform(edge)

    #this is the transform from wpi frame to node frame
    node_global_transform = get_node_global_transform(node)

    #T_ac = _T_ab*T_bc
    connected_node_global_transform = get_connected_node_global_transform(edge_transform, node_global_transform)

    set_node_global_transform(connected_node, connected_node_global_transform)

    edges = get_node_edges(connected_node)

  for edge in edges:
    if edge not in traversed_edges:
      percolate_global_transform(edge, connected_node, traversed_edges)

def get_global_transform():
  wpi_node = get_wpi_node()
  wpi_node_edges = get_node_edges(wpi_node)

  traversed_edges = []

  for edge in wpi_node_edges:
    percolate_global_transform(wpi_node, edge, traversed_edges)

  current_pose_node = get_latest_pose_node_with_global_transform()

def get_latest_pose_node_with_global_transform():
  global session

  

if __name__ == '__main__':
  rospy.init_node('stereo_localizer')
  s = rospy.Service('get_last_position', , get_last_position)
  rospy.spin()

