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

def arduino_publisher():
	
	# Create the publisher to talk to the Arduino:
	pub = rospy.Publisher('arduino', String) 

	# Create the subscriber to listen for Twist messages:  

	# Start the arduino_publisher node:
	rospy.init_node('arduino_publisher', anonymous=True)

	while not rospy.is_shutdown():
	
		nextCommand = str(raw_input("Please enter your next command: "))
		print("nextCommand: " + nextCommand)

		if nextCommand == 'BYE':
			print("Goodbye!")
			return

		pub.publish(nextCommand)

if __name__ == '__main__':
	try:
		arduino_publisher()
	except rospy.ROSInterruptException: pass
