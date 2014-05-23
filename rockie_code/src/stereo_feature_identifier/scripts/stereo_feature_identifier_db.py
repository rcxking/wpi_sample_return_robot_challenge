#!/usr/bin/python
import datetime

from sqlalchemy.dialects.mysql import DATETIME, VARCHAR
from sqlalchemy import Column, ForeignKey, Integer, String, Boolean, DateTime
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import relationship
from sqlalchemy import create_engine

Base = declarative_base()

class Stereo_Pair_Keypoints(Base):
  __tablename__='stereo_pair_keypoints'
  id = Column(Integer, primary_key=True, autoincrement=True)
  stereo_image_pair_id = Column(Integer)
  left_keypoints_filepath = Column(VARCHAR(6500))
  right_keypoints_filepath = Column(VARCHAR(6500))

if __name__ == '__main__':
  engine = create_engine('mysql://root@localhost/rockie')
  Base.metadata.create_all(engine)
