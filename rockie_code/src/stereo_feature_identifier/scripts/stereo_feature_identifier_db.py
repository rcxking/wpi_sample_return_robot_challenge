#!/usr/bin/python
import datetime

from sqlalchemy.dialects.mysql import DATETIME, VARCHAR, DOUBLE
from sqlalchemy import Column, ForeignKey, Integer, String, Boolean, DateTime
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import relationship
from sqlalchemy import create_engine

Base = declarative_base()

class Stereo_Pair_Keypoints(Base):
  __tablename__='stereo_pair_keypoints'
  stereo_pair_keypoint_id = Column(Integer, primary_key=True, autoincrement=True)
  stereo_image_pair_id = Column(Integer)
  left_keypoints_filepath = Column(VARCHAR(6500))
  right_keypoints_filepath = Column(VARCHAR(6500))

class Stereo_Pair_Keypoint_Matches(Base):
  __tablename__='stereo_pair_keypoint_matches'
  sp_keypoint_matches_id = Column(Integer, primary_key=True, autoincrement=True)
  stereo_pair_keypoint_id = Column(Integer)
  sp_keypoint_matches_filepath = Column(VARCHAR(6500))
  sp_matches_3d_points_filepath = Column(VARCHAR(6500))

class Stereo_3D_Matches(Base):
  __tablename__='stereo_3d_matches'
  sp_3d_matches_id = Column(Integer, primary_key=True, autoincrement=True)
  sp_matches_id = Column(Integer)
  sp_3d_matches_filepath = Column(VARCHAR(6500))
  sp_wpi_feature_name = Column(VARCHAR(6500))

class Graph_Nodes(Base):
  __tablename__='graph_nodes'
  node_id = Column(Integer, primary_key=True, autoincrement=True)
  node_type = Column(VARCHAR(6500))
  sp_3d_matches_id = Column(VARCHAR(6500))
  optimal_transformation_filepath = Column(VARCHAR(6500))
  x = Column(DOUBLE)
  y = Column(DOUBLE)
  z = Column(DOUBLE)

class Graph_Edges(Base):
  __tablename__='graph_edges'
  edge_id = Column(Integer, primary_key=True, autoincrement=True)
  node_1_id = Column(Integer)
  node_2_id = Column(Integer)
  _3d_matches_filepath = Column(VARCHAR(6500))
  rigid_body_transform_filepath = Column(VARCHAR(6500))

if __name__ == '__main__':
  engine = create_engine('mysql://root@localhost/rockie')
  Base.metadata.create_all(engine)
