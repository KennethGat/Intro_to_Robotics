#!/usr/bin/env python3

'''
COURSE CODE:	CSC592 (SPRING 2023)
COURSE NAME:	INTRODUCTION TO ROBOTICS
NAME:			KENNETH MURIUKI GATOBU
ASSIGNMENT:		ROBOTICS LAB 3
TYPE:			UNIVERSAL ROBOT UR5E KINEMATICS Cofiguration (Problem 2)
DATE:			FEB 20 2023
'''

import robot_model as rm
import math
import numpy as np


def two_link_planar(planar_m_params):
	'''
		Function to implement the two planar manipulator 
		explained in class
		
		Takes: DH chain parameters
		
		Returns: N/A
	'''
	# Two-link planar maipulator
	print ('\n\n===================================')
	print ('Two-link planar manipulator problem')
	print ('===================================')
	print (f'a1=1M, a2=1M, theta1=pi/2, theta2=pi/2')
	print ('======================================')
		
	final = rm.kinematic_chain(planar_m_params)
	
	print (f'\nFinal chain tranformation for the planar-manipulator is: \n{final}')
	
	final_pos = rm.get_pos(final)
	
	print (f'\nFinal end-effector position [x,y,z] for the planar-manipulator is:' 
			f'\n[{final_pos[0]:.2f},{final_pos[1]:.2f},{final_pos[2]:.2f}] in Meters')
	
	final_rot = rm.get_rot(final)
	
	print (f'\nFinal end-effector rotation [roll-pitch-yaw] for the planar-manipulator is:'
			f'\n[{final_rot[0]:.2f}, {final_rot[1]:.2f}, {final_rot[2]:.2f}] in radians')
	

def UR5e_6DoF_static_params(thetas):
	'''
		Function that takes the variable theta angles as parameters and
		returns a complete 2D DH_table array/list for the 
		6DoF_UR5e robot kinematic chain
		 
		
		Takes: A list of joint angles for the UR5e robot
		
		Returns: A complete 2D array of the DH parameteres with the joint angles appended
	'''
	
	# Define the ur5e joint constants with the thetas appended ('j' for joint)
	j1 = [0.0, math.pi/2, 0.1625, thetas[0]]
	j2 = [-0.425, 0.0, 0.0, thetas[1]]
	j3 = [-0.3922, 0.0, 0.0, thetas[2]]
	j4 = [0.0, math.pi/2, 0.1333, thetas[3]]
	j5 = [0.0, -math.pi/2, 0.0997, thetas[4]]
	j6 = [0.0, 0.0, 0.0996, thetas[5]]
	
	
	# Create a 2D array of robot's DH parameters
	ur5e_params = [j1,j2,j3,j4,j5,j6]
	
	# Return the robot's DH array
	return ur5e_params


	
	
def UR5e_6DoF_robot(ur5e_DH_params):
	'''
		Function to implement the UR5e robot with
		6 degrees of freedom as explained in class
		
		Takes: DH chain parameters
		
		Returns: N/A
	'''
	# UR5e Robot
	print ('\n\n============================')
	print ('6 DoF UR5e robot problem')
	print ('============================')
	print (f'theta1={ur5e_DH_params[0][3]},theta2={ur5e_DH_params[1][3]},'
			f'theta3={ur5e_DH_params[2][3]}\ntheta4={ur5e_DH_params[3][3]},'
			f'theta5={ur5e_DH_params[4][3]},theta6={ur5e_DH_params[5][3]}')
	print ('====================================================')
		
	final = rm.kinematic_chain(ur5e_DH_params)
	
	print (f'\nFinal chain tranformation for the 6DoF-UR5e-robot is: \n{final}')
	
	final_pos = rm.get_pos(final)
	
	print (f'\nFinal end-effector position [x,y,z] for the 6DoF-UR5e-robot is: '
			f'\n[{final_pos[0]:.2f}, {final_pos[1]:.2f}, {final_pos[2]:.2f}] in Meters')
	
	final_rot = rm.get_rot(final)
	
	print (f'\nFinal end-effector rotation [roll-pitch-yaw] for the 6DoF-UR5e-robot is:'
			f'\n[{final_rot[0]:.2f}, {final_rot[1]:.2f}, {final_rot[2]:.2f}] in radians')
	
	



if __name__ == '__main__':

	# Format output
	np.set_printoptions(precision = 3, suppress = True)
	
	
	# Two-link planar parameters
	planar_m_params = [[1.0, 0.0, 0.0, math.pi/2], [1.0, 0.0, 0.0, math.pi/2]]
	
	# Pass planar parameters to resepctive function
	two_link_planar(planar_m_params)
	
	
	
	# 1st 6DoF-UR5e-robot Parameters
	thetas = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
	
	# Pass 1st set of thetas to UR5e_6DoF_static_params function
	# inorder to create the kinematic chain DH parameters
	ur5e_DH_params = UR5e_6DoF_static_params(thetas)
	
	# Pass ur5e parameters to resepctive function
	UR5e_6DoF_robot(ur5e_DH_params)
	
	
	
	# 2nd 6DoF-UR5e-robot Parameters
	thetas = [0.0, -math.pi/2, 0.0, 0.0, 0.0, 0.0]
	
	# Pass 1st set of thetas to UR5e_6DoF_static_params function
	# inorder to create the kinematic chain DH parameters
	ur5e_DH_params = UR5e_6DoF_static_params(thetas)
	
	# Pass ur5e parameters to resepctive function
	UR5e_6DoF_robot(ur5e_DH_params)
	
	
	
	
	
	

