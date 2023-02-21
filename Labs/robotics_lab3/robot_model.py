#!/usr/bin/env python3

'''
COURSE CODE:	CSC592 (SPRING 2023)
COURSE NAME:	INTRODUCTION TO ROBOTICS
NAME:			KENNETH MURIUKI GATOBU
ASSIGNMENT:		ROBOTICS LAB 3
TYPE:			UNIVERSAL ROBOT UR5E KINEMATICS MODULE (Problem 1)
DATE:			FEB 20 2023
'''

import math
import numpy as np

def dh_transformation (DH_Param):

	'''
		Function that receives the Denavit-Hartenberg parameters of a single link as a list...
		and returns the homgenous transformation A_i of a link_(i)...
		with respect to link_(i-1)
		Parameters:
			trans_Xa (Link length)
			rotAlpha_X (Link twist)
			trans_Zd (Link Offset)
			rotTheta_Z (Joint angle)									
	'''
	
	# Assign the elements of the list to variables for readability
	trans_Xa = DH_Param[0]
	rotAlpha_X = DH_Param[1]
	trans_Zd = DH_Param[2]
	rotTheta_Z = DH_Param[3]
	
	A_i = np.array([
					[math.cos(rotTheta_Z), (-math.sin(rotTheta_Z) * math.cos(rotAlpha_X)), (math.sin(rotTheta_Z) * math.sin(rotAlpha_X)), (trans_Xa * math.cos(rotTheta_Z))],
					[math.sin(rotTheta_Z), (math.cos(rotTheta_Z) * math.cos(rotAlpha_X)), (-math.cos(rotTheta_Z) * math.sin(rotAlpha_X)), (trans_Xa * math.sin(rotTheta_Z))],
					[0.0, math.sin(rotAlpha_X), math.cos(rotAlpha_X), trans_Zd],
					[0.0, 0.0, 0.0, 1.0]
					])
					
					
	return A_i
	


def kinematic_chain (DH_chain):
	'''
		Function that receives a list of lists.......
		where each sub-list represents the DH parameters of each link in the kinematic chain
		
		Assumptions:
		
			Each sub-list contains 4 parameters in the order that follows:
			- trans_Xa		(Link Length a_i)
			- rotAlpha_X 	(Link twist alpha_i)
			- trans_Zd 		(Link Offset d_i)
			- rotTheta_Z 	(Joint angle theta_i)
			
			The sublists are ordered using forward kinematics convention 
			i.e (base frame to end effector, all in current frames minus the base frame)
			
			
		Function Returns:
			- The total transformation of the forward kinematic chain		
	'''
	
	
	# Initialize an identity 4 x 4 matrix)
	final_Transform = np.identity(4)
	
	
	# Initiate loop to read each sublist while passing it to the dh_transformation function above
	for params in range(len(DH_chain)):
	
		# Call dh_transformation function passing 1 sub-list (each link's DH parameters) at a time
		link_Transform = dh_transformation(DH_chain[params])
		
		# Do a dot product with each link transformation obtained from function above
		final_Transform = np.matmul(final_Transform, link_Transform)
		
		# Print Result of each loop's calculation
		print (f'Link {params} result:\n {final_Transform}\n')
		
		
	# Return final transformation
	return final_Transform
		


def get_pos(final_Transform):
	'''
		Function that receives an array of the total transformation from...
		the kinematic_chain function above and...
		return the x, y and z components of the end effector
	'''
	
	# Use indexing to extract the required translational position co-ordinates
	x_comp = final_Transform[0][3]
	y_comp = final_Transform[1][3]
	z_comp = final_Transform[2][3]
	
	# Return the translational components in as a tuple
	return (x_comp, y_comp, z_comp)
	


def get_rot(final_Transform):
	'''
		Function that receives an array of the total transformation from
		the knematic chain function and returns
		the roll-pitch-yaw angles
	'''
	
	# Get the roll angle in radians
	Roll = math.atan2(final_Transform[2][1], final_Transform[2][2])
	
	# Get the pitch angle in radians
	Pitch = math.atan2(-final_Transform[2][0], math.sqrt((final_Transform[2][1] ** 2) + (final_Transform[2][2] ** 2)))
	
	# Get the yaw angle in radians
	Yaw = math.atan2(final_Transform[1][0], final_Transform[0][0])
	
	# Return the rotational components as a tuple
	return (Roll, Pitch, Yaw)
	


