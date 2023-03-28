#!/usr/bin/env python3

'''
COURSE CODE:	CSC592 (SPRING 2023)
COURSE NAME:	INTRODUCTION TO ROBOTICS
INSTRUCTOR:		DR. HAMED SAEIDI
CODE CREDITS:	DR. HAMED SAEIDI
NAME:			KENNETH MURIUKI GATOBU
ASSIGNMENT:		ROBOTICS LAB 5
DATE:			MARCH 27 2023
'''


'''
============================================================================
				IMPORT NECESSARY MODULES
============================================================================
'''

import rospy
import numpy as np
import math
from geometry_msgs.msg import Point
from robot_vision_lectures.msg import XYZarray
from robot_vision_lectures.msg import SphereParams

'''
============================================================================
				DEFINE MESSAGE VARIABLES
============================================================================
'''
current_sphere_params = []


'''
============================================================================
				DEFINE NECESSARY CALL BACK FUNCTIONS
============================================================================
'''

# get the image message
def call_back(data):
	
	# Declare global XYZarray message object
	global current_sphere_params
	
	# Get x,y,z co-ordinates of the 3D camera co-ordinate frame
	# and append it to the current_sphere_params list
	for params in data.points:
		current_sphere_params.append([params.x, params.y, params.z])
	

'''
============================================================================
				DEFINE MAIN FUNCTION
============================================================================
'''
	
	
if __name__ == '__main__':
	# Initialize the node
	rospy.init_node('sphere_fit', anonymous = True)
	
	# define a subscriber to read white point x,y,z co-ordinates
	rospy.Subscriber("/xyz_cropped_ball", XYZarray, call_back)
	 
	# define a publisher to publish the estimated xc,yc,zc and radius values
	sphere_centre_pub = rospy.Publisher('/sphere_params', SphereParams, queue_size = 1)
	
	# Declare object variable of type SphereParams message for use in publishing a
	# .....mathematical sphere model
	sphere_model = SphereParams()
	
	# set the loop frequency
	rate = rospy.Rate(10)

	while not rospy.is_shutdown():
		
		# Create A matrix
		matrix_A = []
		for params in current_sphere_params:
			matrix_A.append([2*params.x, 2*params.y, 2*params.z, 1])
		matrix_A = np.array(matrix_A)
		
		# Create B matrix
		matrix_B = []
		for params in current_sphere_params:
			matrix_B.append([(params.x**2) + (params.y**2) + (params.z**2)])
		matrix_B = np.array(matrix_B)
		
		# Determine ATA matrix
		ATA = np.matmul(matrix_A.T, matrix_A)
		
		# Determine ATB matrix
		ATB = np.matmul(matrix_A.T, matrix_B)
		
		# Determine P based on solution provided in class
		P = np.matmul(np.linalg.inv(ATA), ATB)
		
		# Get xc, xy & xz
		xc = P[0]
		yc = P[1]
		zc = P[2]
		
		# Get radius
		r = math.sqrt(P[3] + (xc**2) + (yc**2) + (zc**2))
		
		# Assign SphereParams message object derived values for publishing purposes
		sphere_model.xc = xc
		sphere_model.yc = yc
		sphere_model.zc = zc
		sphere_model.radius = r
		
		# Publish the derived values to sphere_params topic
		sphere_centre_pub.publish(sphere_model)
		
		# pause until the next iteration			
		rate.sleep()

