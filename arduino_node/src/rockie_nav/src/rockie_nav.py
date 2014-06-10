#!/usr/bin/python
import roslib
import rospy
import numpy
import tf

def wpi_frame_subscriber(msg):
	pass 

# Main function for rockie_nav:
if __name__ == '__main__':
	rospy.init_node('rockie_nav')	 

	# Transform Listener:
	listener = tf.TransformListener()

	# Loop forever:
	while not rospy.is_shutdown():
		try:
			# Get the next rotation and translation:
			(trans, rot) = listener.lookupTransform('/robot_frame', '/wpi_frame', rospy.Time(0))

			currentPos = numpy.array([[0], [0], [0]])
			currentPosAhead = numpy.array([[0], [0], [1]])

			wpiCoordinates = numpy.dot(rot, currentPos) + trans
			wpiCoordinatesPosAhead = numpy.dot(rot, currentPosAhead) + trans

			# XYZ - Vector - Ignore Z
			orientationVector = wpiCoordinatesPosAhead - wpiCoordinates
			print(orientationVector)

		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			continue
	
			
		
		
		 
