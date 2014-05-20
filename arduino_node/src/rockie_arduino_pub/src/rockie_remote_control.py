#!/usr/bin/python

'''
This node allows users to send commands to Rockie
'''

import rospy
from std_msgs.msg import String

def talker():
	pub = rospy.Publisher('repl_ctrl', String, queue_size=10)
	rospy.init_node('repl', anonymous=True)
	
	while not rospy.is_shutdown():
		nextCommand = str(raw_input("Please enter your next command: "))
		pub.publish(nextCommand)
		
if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException: pass	

   
