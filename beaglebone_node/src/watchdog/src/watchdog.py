#!/usr/bin/python

'''
watchdog.py - Node to connect the Beaglebone to the Rockie Computer.

RPI Rock Raiders
4/20/15 

Last Updated: Bryant Pong: 4/21/15 - 10:49 AM
'''

# ROS Python Library:
import rospy
from std_msgs.msg import String

# STD Python Libraries
import socket

# Global Constants:
beagleIP = ""
beaglePort = 9001
server_address = (beagleIP, beaglePort)
# TCP Socket for Main Computer:
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# End Global Constants

def beaglenode():
	pub = rospy.Publisher("watchdog", String)
	rospy.init_node("watchdog", anonymous=True)
	r = rospy.Rate(10);

	sock.connect(server_address)

	while not rospy.is_shutdown():
		#sock.connect(server_address)

		message = "This is the message.  It will be repeated."
		sock.send(message)
		data = sock.recv(1024)

		print("Received data: " + str(data))
		
				
if __name__ == "__main__":
	try:
		beaglenode()
	except rospy.ROSInterruptException: pass

	 
