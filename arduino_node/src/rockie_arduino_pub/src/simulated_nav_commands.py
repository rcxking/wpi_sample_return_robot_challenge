#!/usr/bin/python

# simulated_nav_commands.py - Node to send simulated ROS geometry_msgs/Twist
# messages to Rockie's Arduino Node.
#
# RPI Rock Raiders
# 5/12/14
#
# Last Updated: Bryant Pong: 5/27/14 - 3:35 PM

# ROS Libraries:
import rospy
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Float64

# Random Library:
import random

def random_commands():
	# Create a ROS topic called "random_vel_messages" that 
	# communicates via geometry_msgs/Twist messages:
	pub = rospy.Publisher('cmd_vel', Twist, queue_size=10) 	
	rospy.init_node('simulated_nav_commands', anonymous=True)
	r = rospy.Rate(20)

	while not rospy.is_shutdown():
		# This node spoofs geometry_msgs/Twist messages.
		# We need 2 random doubles and two Vector3 data structures:
		randomLinearVelocity = random.random() * 2
		randomAngularVelocity = random.random() * 2 	 

		print("randomLinearVelocity is " + str(randomLinearVelocity))
		print("randomAngularVelocity is " + str(randomAngularVelocity))

		# Construct the two Vector3 data structures:
		linearVel = Vector3(randomLinearVelocity, 0, 0)
		print("linearVel is: [" + str(linearVel.x) + "," + str(linearVel.y) + "," + str(linearVel.z) + "]")

		angularVel = Vector3(0, 0, randomAngularVelocity)

		# Now we can construct and publish the Twist message:
		twistMsg = Twist(linearVel, angularVel)		 
		pub.publish(twistMsg)
		r.sleep()

if __name__ == '__main__':
	try:
		random_commands()
	except rospy.ROSInterruptException: 
		pass
