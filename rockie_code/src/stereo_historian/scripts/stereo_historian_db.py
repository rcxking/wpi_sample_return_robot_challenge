#!/usr/bin/python
import datetime

from sqlalchemy.dialects.mysql import DATETIME, VARCHAR
from sqlalchemy import Column, ForeignKey, Integer, String, Boolean, DateTime
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import relationship
from sqlalchemy import create_engine

Base = declarative_base()

class Image_Frame(Base):
    __tablename__='image_frame'
    id = Column(Integer, primary_key=True, autoincrement=True)
    is_left = Column(Boolean)
    capture_time = Column(DATETIME)
    filepath = Column(VARCHAR(65000))

if __name__ == '__main__':
    engine = create_engine('mysql://root@localhost/rockie')
    Base.metadata.create_all(engine)

