#!/usr/bin/python
import numpy as np
import cv2
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image as ros_image
from cv_bridge import CvBridge, CvBridgeError
import datetime
from stereo_historian_db import Image_Frame, Base
from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker

#stereo_ns = '/my_stereo'
#image_name = 'image_rect'

stereo_ns = 'rrbot/camera1'
image_name = 'image_raw'
path_to_img_store = '/home/rockie/Code/wpi-sample-return-robot-challenge/rockie_code/src/stereo_historian/images'
bridge = CvBridge()

engine = create_engine('mysql://root@localhost/rockie')
Base.metadata.bind = engine
DBSession = sessionmaker(bind=engine)
session = DBSession()

pub = rospy.Publisher("/{0}/stereo_image_saves".format(stereo_ns), String)


def ConvertToCV2Grayscale(img):
    cv2_img = bridge.imgmsg_to_cv2(img, "bgr8")
    cv2_img_gray = cv2.cvtColor(cv2_img, cv2.COLOR_BGR2GRAY)
    return cv2_img_gray

def WriteToFile(img, time, camera):
    img_file_string = "{0}/{1}/{2}.jpg".format(path_to_img_store, camera, str(time))
    cv2.imwrite(img_file_string, img)
    return img_file_string

def WriteToDatabase(filepath, camera, time):
    new_frame = Image_Frame()
    new_frame.filepath = filepath
    new_frame.is_left = True if camera == 'left' else False
    new_frame.capture_time = time

    session.add(new_frame)
    session.commit()

def save_image(img, time, camera):

    img = ConvertToCV2Grayscale(img)
    filepath = WriteToFile(img, time, camera)
    WriteToDatabase(filepath, camera, time)

    pub.publish(filepath)

def left_callback(left_image):
    currenttime = datetime.datetime.now()
    save_image(left_image, currenttime, "left")

def right_callback(right_image):
    currenttime = datetime.datetime.now()
    save_image(right_image, currenttime, "right")

def store_stereo_images():

    rospy.init_node("stereo_historian")

    left_img_topic = stereo_ns + '/left/' + image_name
    right_img_topic = stereo_ns + '/right/' + image_name

    rospy.Subscriber(left_img_topic, ros_image, left_callback)
    rospy.Subscriber(right_img_topic, ros_image, right_callback)

    rospy.spin()

if __name__ == '__main__':
    store_stereo_images()

