import numpy as np
import cv2
import rospy
from std_msgs.msg import String
import datetime

if __name__ == '__main__':

  video2 = cv2.VideoCapture(2)
  video1 = cv2.VideoCapture(1)

  while(True):

    ret1, img1 = video1.read()
    ret2, img2 = video2.read()

    gray1 = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)
    gray2 = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)

    cv2.imshow('gray1', gray1)
    cv2.imshow('gray2', gray2)
 
    currenttime = datetime.datetime.now().time()

    cv2.imwrite('images/' + str(currenttime)  + '-left.jpg', gray1)
    cv2.imwrite('images/' + str(currenttime)  + '-right.jpg', gray2)

    cv2.waitKey(300)

