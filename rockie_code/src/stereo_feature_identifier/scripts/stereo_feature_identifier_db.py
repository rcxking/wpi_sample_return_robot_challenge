#!/usr/bin/python
import datetime

from sqlalchemy.dialects.mysql import DATETIME, VARCHAR, DOUBLE
from sqlalchemy import Column, ForeignKey, Integer, String, Boolean, DateTime
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import relationship
from sqlalchemy import create_engine
import MySQLdb

Base = declarative_base()

class Stereo_Pair_Keypoints(Base):
  __tablename__='stereo_pair_keypoints'
  stereo_pair_keypoint_id = Column(Integer, primary_key=True, autoincrement=True)
  stereo_image_pair_id = Column(Integer, index=True)
  left_keypoints_filepath = Column(VARCHAR(6500))
  right_keypoints_filepath = Column(VARCHAR(6500))

class Stereo_Pair_Keypoint_Matches(Base):
  __tablename__='stereo_pair_keypoint_matches'
  sp_keypoint_matches_id = Column(Integer, primary_key=True, autoincrement=True)
  stereo_pair_keypoint_id = Column(Integer, index=True)
  sp_keypoint_matches_filepath = Column(VARCHAR(6500))
  sp_matches_3d_points_filepath = Column(VARCHAR(6500))

class Stereo_3D_Matches(Base):
  __tablename__='stereo_3d_matches'
  sp_3d_matches_id = Column(Integer, primary_key=True, autoincrement=True)
  sp_matches_id = Column(Integer, index=True)
  sp_3d_matches_filepath = Column(VARCHAR(6500))
  sp_wpi_feature_name = Column(VARCHAR(6500))

class Graph_Nodes(Base):
  __tablename__='graph_nodes'
  node_id = Column(Integer, primary_key=True, autoincrement=True)
  node_type = Column(VARCHAR(6500))
  sp_3d_matches_id = Column(VARCHAR(6500))
  global_transformation_filepath = Column(VARCHAR(6500))

class Graph_Edges(Base):
  __tablename__='graph_edges'
  edge_id = Column(Integer, primary_key=True, autoincrement=True)
  node_1_id = Column(Integer, index=True)
  node_2_id = Column(Integer, index=True)
  _3d_matches_filepath = Column(VARCHAR(6500))
  opt_transform_1_to_2_filepath = Column(VARCHAR(6500))

class Stereo_Image_Pair(Base):
  __tablename__='stereo_image_pair'
  stereo_image_pair_id = Column(Integer, primary_key=True, autoincrement=True)
  capture_time = Column(DATETIME)
  left_filepath = Column(VARCHAR(6500))
  right_filepath = Column(VARCHAR(6500))

if __name__ == '__main__':
  engine = create_engine('mysql://root@localhost/rockie')
 
  db = MySQLdb.connect(host="localhost", user="root", passwd="", db="rockie") 

  cur = db.cursor()

  cur.execute("DROP TABLE IF EXISTS graph_nodes")
  cur.execute("DROP TABLE IF EXISTS graph_edges")
  cur.execute("DROP TABLE IF EXISTS stereo_3d_matches")
  cur.execute("DROP TABLE IF EXISTS stereo_image_pair")
  cur.execute("DROP TABLE IF EXISTS stereo_pair_keypoint_matches")
  cur.execute("DROP TABLE IF EXISTS stereo_pair_keypoints")

  #drop_tables = "USE rockie;"
  #drop_tables += " DROP TABLE graph_edges;"
  #drop_tables += " DROP TABLE graph_nodes;"
  #drop_tables += " DROP TABLE stereo_3d_matches;"
  #drop_tables += " DROP TABLE stereo_image_pair;"
  #drop_tables += " DROP TABLE stereo_pair_keypoint_matches;"
  #drop_tables += " DROP TABLE stereo_pair_keypoint"

  #result = conn.execute(drop_tables)

  #Base.metadata.drop_all(engine)

  Base.metadata.create_all(engine)
