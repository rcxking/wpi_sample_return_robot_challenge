#!/usr/bin/python

'''
watchdog.py - Node to connect the Beaglebone to the Rockie Computer.

RPI Rock Raiders
4/20/15 

Last Updated: Bryant Pong: 4/20/15 - 7:28 PM
'''

# ROS Python Library:
import rospy

# STD Python Libraries
import socket

# Global Constants:
beagleIP = "192.168.7.2"
beaglePort = 9001
server_address = (beagleIP, beaglePort)
# TCP Socket for Main Computer:
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# End Global Constants

def beaglenode():
	rospy.init_node('beaglebone_node')

	rospy.spin()

if __name__ == "__main__":
	sock.connect(server_address)

	try:
		message = 'This is the message.  It will be repeated.'
		sock.sendall(message)

		amount_received = 0
		amount_expected = len(message)

		while amount_received < amount_expected:
			data = sock.recv(16)
			amount_received += len(data)

	finally:
		sock.close()
	 
