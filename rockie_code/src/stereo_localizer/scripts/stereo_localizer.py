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
from stereo_feature_identifier_db import Stereo_Pair_Keypoints, Stereo_Pair_Keypoint_Matches, Stereo_3D_Matches, Graph_Nodes, Graph_Edges, Stereo_Image_Pair, Base
import datetime
import cPickle as pickle
from sqlalchemy import create_engine, or_, and_
import random
from sqlalchemy.orm import sessionmaker
import tf
import time
import MySQLdb

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

log = rospy.Publisher("/stereo_localizer/log", String)

#pub = rospy.Publisher("/my_stereo/stereo_image_saves", String)

def get_last_position():
  pass

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
  transform_filepath = edge.opt_transform_1_to_2_filepath 
  transform = pickle.load(open(transform_filepath, "rb"))

  transform =  [np.array(transform[0]), np.array(transform[1])]
  
  if transform[1].shape != (3, 1):
    transform[1] = np.transpose(transform[1])

  return transform

def get_node_edges(node, previous_node):
  global session

  session.commit()

  query = session.query(Graph_Edges)
  query = query.filter(or_(Graph_Edges.node_1_id == node.node_id, Graph_Edges.node_2_id == node.node_id)) 

  #don't get edge we just came from
  if previous_node != None:
    query = query.filter(and_(Graph_Edges.node_1_id != previous_node.node_id, Graph_Edges.node_2_id != previous_node.node_id))

  edges = query.all()

  return edges

def get_node_by_id(node_id):
  global session
  query = session.query(Graph_Nodes)
  query = query.filter_by(node_id = int(node_id))

  return query.first()

def invert_transform(transform):
  [R, t] = transform
  R_inverted = np.transpose(R)

  t_inverted = -np.dot(R_inverted, t)

  return [R_inverted, t_inverted]

#TODO: Convert to homogeneous, will be much easier :)
def get_connected_node_global_transform(edge, root_node, connected_node):

    #this is the tranform between nodes (node 1 to node 2)
    edge_transform = get_edge_transform(edge)

    #edge transform is from 1 to 2

    #if the connected node is node 2, we want g_2w = g_21*g_1w
    #where g_21 is transfrom from 2 to 1, and
    # g_1w is transform from 1 to wpi frame
    if edge.node_2_id == connected_node.node_id:

      #this is the transform from wpi frame to node frame
      [R_1w, t_1w] = get_node_global_transform(root_node)

      [R_21, t_21] = invert_transform(edge_transform)

      #R_ac = R_ab*R_bc
      R_2w = np.dot(R_21, R_1w)
      t_2w = np.dot(R_21, t_1w) + t_21

      R = R_2w
      t = t_2w

    else:

      #this is the transform from wpi frame to node frame
      [R_2w, t_2w] = get_node_global_transform(root_node)

      [R_12, t_12] = edge_transform

      R_1w = np.dot(R_12, R_2w)
      t_1w = np.dot(R_12, t_2w) + t_12
      
      R = R_1w
      t = t_1w

    return [R.tolist(), t.tolist()]
 
def get_connected_node(root_node, edge):
  node_1_id = edge.node_1_id
  node_2_id = edge.node_2_id

  if int(root_node.node_id) == int(node_1_id):
    return get_node_by_id(node_2_id)
  else:
    return get_node_by_id(node_1_id)

def save_global_transform(transform, node):
  filepath = "{0}node_{1}_global_transform.transform".format(stereo_imagepath_base, node.node_id)
  pickle.dump(transform, open(filepath, 'wb'))
  return filepath

def get_node_global_transform(node):
  global session

  filepath = node.global_transformation_filepath
  [R, t] = pickle.load(open(filepath, "rb"))

  transform = [np.array(R), np.array(t)]

  if transform[1].shape != (3, 1):
    transform[1] = np.transpose(transform[1])

  return transform

def set_node_global_transform(node, transform):
  global session
  
  filepath = save_global_transform(transform, node)
  node.global_transformation_filepath = filepath

  session.commit()

def percolate_global_transform(root_node, edge, traversed_edges):
  #node1 has x,y,z. if node2 doesn't have x,y,z, apply transform in edge to get node2 x,y,z.
  #find edges connected to node2. for each connected node, recurse
  
  traversed_edges.append(edge)
  connected_node = get_connected_node(root_node, edge)

  if connected_node.global_transformation_filepath == None:

    #T_ac = _T_ab*T_bc
    connected_node_global_transform = get_connected_node_global_transform(edge, root_node, connected_node)
    set_node_global_transform(connected_node, connected_node_global_transform)

  edges = get_node_edges(connected_node, root_node)

  #print("connected_node id: {0}".format(connected_node.node_id))
  #print("root_node id: {0}".format(root_node.node_id))
  #print("number of node edges: {0}".format(len(edges)))

  for edge in edges:
    if edge not in traversed_edges:
      percolate_global_transform(connected_node, edge, traversed_edges)

def get_wpi_node():
  global session

  query = session.query(Graph_Nodes)
  query.filter(Graph_Nodes.global_transformation_filepath != None)

  return query.first()

def get_global_transform():

  wpi_node = get_latest_pose_node_with_global_transform()

  if wpi_node != None:
    wpi_node_edges = get_node_edges(wpi_node, None)
    #print("number of wpi node edges: {0}".format(len(wpi_node_edges)))

    traversed_edges = []

    for edge in wpi_node_edges:
      #print("connecting node {0} to node {1}".format(edge.node_1_id, edge.node_2_id))
      percolate_global_transform(wpi_node, edge, traversed_edges)

    current_pose_node = get_latest_pose_node_with_global_transform()

    current_global_transform = get_node_global_transform(current_pose_node)

    return current_global_transform

def get_latest_pose_node_with_global_transform():
  global session

  mysql_db = MySQLdb.connect(host="localhost", user="root", passwd="", db="rockie") 

  cur = mysql_db.cursor()
 
  my_query = "SELECT node_id, node_type, sp_3d_matches_id, global_transformation_filepath "
  my_query += "FROM graph_nodes "
  my_query += "WHERE global_transformation_filepath IS NOT NULL "
  my_query += "AND node_id IS NOT NULL "
  #my_query += "AND node_type = 'pose' "
  my_query += "ORDER BY node_id DESC"

  cur.execute(my_query)
  result = cur.fetchone()

  if result != None:
    most_recent_pose_node = graph_node_from_query_result(result) 
  else:
    most_recent_pose_node = None

  return most_recent_pose_node

def graph_node_from_query_result(result):
  latest_pose_node = Graph_Nodes()
  latest_pose_node.node_id = result[0]
  latest_pose_node.node_type = result[1]
  latest_pose_node.sp_3d_matches_id = result[2]
  latest_pose_node.global_transformation_filepath = result[3]

  return latest_pose_node

def get_axis_angle(R):
  theta = np.arccos((np.trace(R) - 1)/2)

  a = np.empty([1, 3])

  a[0] = R[3,2] - R[2,3]
  a[1] = R[1,3] - R[3,1]
  a[2] = R[2,1] - R[2,1]

  axis = np.dot((1/(2*np.sin(angle))), a)

  return [axis, theta]

def get_stamped_transform(R, t):
  m = geometry_msgs.msg.TransformStamped()
  m.header.frame_id = 'map'
  m.child.frame_id = 'rockie'

  m.transform.translation.x = t[0]
  m.transform.translation.y = t[1]
  m.transform.translation.z = t[2]

  [axis, angle] = get_axis_angle(R) 

  m.transform.rotation.x = axis[0]
  m.transform.rotation.y = axis[1]
  m.transform.rotation.z = axis[2]
  m.transform.rotation.w = angle
  
  return m

#refer to http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/
def get_quaternion_from_rotation_matrix(R):
  w = np.sqrt(1 + R[0, 0] + R[1, 1] + R[2, 2])/2
  x = (R[2, 1] - R[1, 2])/(4*w) 
  y = (R[0, 2] - R[2, 0])/(4*w) 
  z = (R[1, 0] - R[0, 1])/(4*w) 

  return (w, x, y, z)

if __name__ == '__main__':
  engine = create_engine('mysql://root@localhost/rockie')

  rospy.init_node('stereo_localizer')

  while rospy.get_param("/fs_cleaning") == 1 or rospy.get_param('/db_cleaning') == 1:
    time.sleep(1) 
  
  br = tf.TransformBroadcaster()
  rate = rospy.Rate(.2)

  while not rospy.is_shutdown():
    global_transform = get_global_transform()

    print(global_transform)
    quaternion = get_quaternion_from_rotation_matrix(R)
    br.sendTransform((t[0, 1], t[0, 1], t[0, 2]), quaternion, rospy.Time.now(), 'rockie', 'map')

    #when looping rosbag, this will go backwards and throw and error :)
    try:
      rate.sleep()
    except:
      pass

