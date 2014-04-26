#!/usr/bin/python

'''
watchdog.py - Node to connect the Arduino to the Rockie Computer.

RPI Rock Raiders
4/20/15 

Last Updated: Bryant Pong: 4/25/15 - 6:01 PM
'''

# ROS Python Library:
import rospy
from std_msgs.msg import String

def beaglenode():
	pub = rospy.Publisher("watchdog", String)
	rospy.init_node("watchdog", anonymous=True)
	r = rospy.Rate(10);

	sock.connect(server_address)

	while not rospy.is_shutdown():

		message = str(raw_input("Please enter in a message: "))


		print("You entered: " + message)

		
		if message == "bye":
			break

		sock.send(message)
		data = sock.recv(len(message))

		print("Received data: " + str(data))
		
				
if __name__ == "__main__":
	try:
		beaglenode()
	except rospy.ROSInterruptException: pass

	 
