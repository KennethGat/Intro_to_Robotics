#!/usr/bin/env python3

'''
COURSE CODE:	CSC592 (SPRING 2023)
COURSE NAME: 	INTRODUCTION TO ROBOTICS
INSTRUCTOR:		DR. HAMED SAEIDI
CODE CREDITS:	DR. HAMED SAEIDI
NAME:			KENNETH MURIUKI GATOBU
ASSIGNMENT:		ROBOTICS LAB 5
DATE:			MARCH 27 2023
'''


'''
===================================================================
				IMPORT NECESSARY MODULES
===================================================================
'''

import rospy
import numpy as np
import math
from robot_vision_lectures.msg import XYZarray
from robot_vision_lectures.msg import SphereParams


'''
===================================================================
				DEFINE MESSAGE VARIABLE TO ENSURE DATA
								IS RECEIVED
===================================================================
'''
msg_received = False


'''
===================================================================
				DEFINE NECESSARY CALL BACK FUNCTIONS
===================================================================
'''
def call_back(data):
	'''
		Call back function that receives data from....
		/xyz_cropped_ball topic and uses the data to extract 
		the white space's x,y and z co-ordinates		
	'''
	
	global msg_received
	global current_sphere_params
	
	# Checker to ensure data is recieved
	msg_received = True
	
	# Get x,y,z co-ordinates of the 3D camera co-ordinate frame
	# and append it to the current_sphere_params list
	current_sphere_params = [(point.x, point.y, point.z) for point in data.points]


def get_radius(P):
	'''
		Function to receive the calculated unknown matrix P for use in extracting the
		sphere's centre 3D co-ordinates and its radius
	'''
	
	# Extract the centre points and 4th element of P
	xc, yc, zc, r_param = P[0]
	
	# Get radius
	radius = math.sqrt(r_param + (xc**2) + (yc**2) + (zc**2))
	
	# Return the radius
	return radius
	
	
def fit_sphere(points):
	'''
		Function to take the readings from XYZArray msg for use in pre-processing the 
		matrices needed to calculate the unknown matrix P
	'''

	# Define A and B matrices
	matrix_A = []
	matrix_B = []
	
	# Loop through current_sphere_params
	# to extract x, y, and z values
	for params in points:
		x, y, z = params
		
		# Append sub-lists of the results of factor 2 of
		# x, y and z values as defined for matrix A
		matrix_A.append([2*x,2*y, 2*z, 1])
		
		# Append sub-lists of the results of the square of
		# x, y and z values as defined for matrix B
		matrix_B.append([(x**2) + (y**2) + (z**2)])
		
	# Convert the matrices to arrays
	matrix_A = np.array(matrix_A)
	matrix_B = np.array(matrix_B)
	
	# Reshape matrix_A to n rows by 4 columns
	matrix_A = matrix_A.reshape(len(matrix_A), 4)
	
	# Reshape matrix_B to n rows by 1 column
	matrix_B = matrix_B.reshape(len(matrix_B), 1)
	
	# Determine P based on solution provided in class
	P = np.linalg.lstsq(matrix_A, matrix_B, rcond=None)
	
	# Return P
	return P
	
'''
===================================================================
				DEFINE MAIN FUNCTION
===================================================================
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
	
	# Set the loop frequency
	rate = rospy.Rate(10)
	
	# Loop to continuosly get the unknown sphere model parameters
	# and publish them to the respective topic in order to project
	# the mathematical sphere mode on to the whitepoint
	while not rospy.is_shutdown():
		
		# Check point to ensure data is received
		if msg_received:
			
			# Get the unknown but estimated matrix P
			P = fit_sphere(current_sphere_params)
			
			# Extract the co-ordiantes of the sphere's centre from the 
			# 1st element of the unkown but estimated matrix P
			xc, yc, zc, r_param = P[0]
			
			# Get the estimated radius of the mathematical sphere model
			radius = get_radius(P)
			
			# Assign the extracted xc, yc, zc and radius to a Sphere_Params msg
			sphere_model.xc = float(xc)
			sphere_model.yc = float(yc)
			sphere_model.zc = float(zc)
			sphere_model.radius = radius
			
			# Publish the derived values to sphere_params topic
			sphere_centre_pub.publish(sphere_model)
			
		# Pause till the next iteration
		rate.sleep()
