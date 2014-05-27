#!/usr/bin/python
import numpy as np
import math
import cv2
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image as ros_image
from cv_bridge import CvBridge, CvBridgeError
import datetime
from stereo_historian_db import Stereo_Image_Pair, Base
from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker
import os

stereo_ns = 'my_stereo'
image_name = 'image_raw'

#stereo_ns = 'rrbot/camera1'
##image_name = 'image_raw'

#Max number of seconds allowed for approx sync
#1/framerate should be the max diff in timestamps for stereo pairs
max_sync_period = .01

left_timestamp = None
right_timestamp = None

path_to_img_store = 'images'
bridge = CvBridge()

engine = create_engine('mysql://root@localhost/rockie')
Base.metadata.bind = engine
DBSession = sessionmaker(bind=engine)

#Capture every nth frame
rate = 10
left_iteration = 0
right_iteration = 0

pub = rospy.Publisher("/my_stereo/stereo_image_saves", String)

def ConvertToCV2Grayscale(img):
    cv2_img = bridge.imgmsg_to_cv2(img, "bgr8")
    cv2_img_gray = cv2.cvtColor(cv2_img, cv2.COLOR_BGR2GRAY)
    return cv2_img_gray

def WriteToFile(img, time, camera):
    img_file_string = "{0}/{1}/{2}.jpg".format(path_to_img_store, camera, str(time))
    cv2.imwrite(img_file_string, img)
    return img_file_string

def WriteToDatabase(left_filepath, right_filepath, time):
    session = DBSession()

    stereo_pair = Stereo_Image_Pair()
    stereo_pair.left_filepath = left_filepath
    stereo_pair.right_filepath = right_filepath
    stereo_pair.capture_time = time 

    session.add(stereo_pair)
    session.commit()
    stereo_image_pair_id = stereo_pair.stereo_image_pair_id
    session.close()

    return stereo_image_pair_id

def save_image(img, time, camera):
    global left_timestamp
    global right_timestamp
    global left_img
    global right_img
    global pub

    if left_timestamp == None or right_timestamp == None:
        return

    total_time_left = left_timestamp.secs + left_timestamp.nsecs*math.pow(10, -9) 
    total_time_right = right_timestamp.secs + right_timestamp.nsecs*math.pow(10, -9) 

    time_diff_between_stereo_cams = math.fabs(total_time_left - total_time_right)

    if time_diff_between_stereo_cams > max_sync_period:
        return

    left_img = ConvertToCV2Grayscale(left_img)
    right_img = ConvertToCV2Grayscale(right_img)

    left_filepath = WriteToFile(left_img, left_timestamp, 'left')
    right_filepath = WriteToFile(right_img, right_timestamp, 'right')

    stereo_pair_db_id = WriteToDatabase(left_filepath, right_filepath, datetime.datetime.now())

    pub.publish(str(stereo_pair_db_id))

def left_callback(left_image):
    global left_timestamp
    global left_img
    global left_iteration
    global rate

    left_iteration += 1
  
    if left_iteration % rate != 0:
        return

    left_timestamp = left_image.header.stamp 
    left_img = left_image
    save_image(left_image, left_timestamp, "left")

def right_callback(right_image):
    global right_timestamp
    global right_img
    global right_iteration
    global rate

    right_iteration += 1

    if right_iteration % rate != 0:
        return

    right_timestamp= right_image.header.stamp
    right_img = right_image
    save_image(right_image, right_timestamp, "right")

def store_stereo_images():
    rospy.init_node("stereo_historian")

    left_img_topic = stereo_ns + '/left/' + image_name
    right_img_topic = stereo_ns + '/right/' + image_name

    rospy.Subscriber(left_img_topic, ros_image, left_callback)
    rospy.Subscriber(right_img_topic, ros_image, right_callback)
    
    rospy.spin()
if __name__ == '__main__':
  try:
    os.makedirs('images/left')
    os.makedirs('images/right')
  except:
    pass

  store_stereo_images()

