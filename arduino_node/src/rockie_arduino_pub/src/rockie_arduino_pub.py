#!/usr/bin/python

# rockie_arduino_pub.py - Publisher to the Arduino to send motor commands and
# read encoder data.
#
# RPI Rock Raiders
# 4/26/14
#
# Last Updated: 5/12/14 - 11:28 AM

# ROS Libraries:
import rospy
from std_msgs.msg import String, Float64
from geometry_msgs.msg import Twist

pub = rospy.Publisher('arduino', String)

def cmdVelCallback(data):

	# This node is expecting "data" to be of type geometry_msgs/Twist.

	# First, let's get the linear and angular velocity request from the Twist msg:
	linearVel = data.linear.x
	angularVel = data.angular.z
		 
	pub.publish("DRIVE " + str(linearVel) + " " + str(angularVel))

def replCallback(data):

	pub.publish(data)
   

def arduino_publisher():
	
	# Create the subscriber to listen for Twist messages:  
	rospy.init_node('cmdVelListener', anonymous=True)
	rospy.Subscriber('cmd_vel', Twist, cmdVelCallback)
	rospy.Subscriber('repl_ctrl', String, replCallback)

	rospy.spin()


if __name__ == '__main__':
	try:
		arduino_publisher()
	except rospy.ROSInterruptException: pass
