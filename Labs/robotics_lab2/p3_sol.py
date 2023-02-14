#!/usr/bin/env python3

'''
COURSE CODE:	CSC592 (SPRING 2023)
COURSE NAME:	INTRODUCTION TO ROBOTICS
NAME:			KENNETH MURIUKI GATOBU
ASSIGNMENT:		ROBOTICS LAB 2 (p3_sol)
TYPE:			HOMOGENOUS PROBLEMS & SOLUTIONS
DATE:			FEB 13 2023
'''

import p2_sol as ht
import math
import numpy as np

if __name__ == '__main__':

	# Format output
	np.set_printoptions(precision = 3, suppress = True)
	
	# Transfomation 1 (H_1)
	print ('==========================================================================')
	print ('\nTrasnformation H_1 ')
	print ('==========================================================================')
	print ('Resulting transformation will be currentTransX.currentTransZ.currentTransY')
	print ('==========================================================================')
	# Pass 2.5 units along x_axis
	x_trans = ht.trans_x(2.5)
	
	# Pass 0.5 units along z axis
	z_trans = ht.trans_z(0.5)
	
	# Pass -1.5 units along y axis
	y_trans = ht.trans_y(-1.5)
	
	# Perform resulting h_1 calculation
	h_1 = np.matmul(x_trans,z_trans)
	h_1 = np.matmul(h_1,y_trans)
	print (f'Homogenous transformation H_1 result is: \n{h_1}\n')


	# Transfomation 2 (H_2)
	print ('==========================================================================')
	print ('\nTrasnformation H_2 ')
	print ('==========================================================================')
	print ('Resulting transformation will be currentTransZ.currentTransX.currentTransY')
	print ('==========================================================================')
	# Pass 0.5 units along z axis
	z_trans = ht.trans_z(0.5)
	
	# Pass 2.5 units along x_axis
	x_trans = ht.trans_x(2.5)
	
	# Pass -1.5 units along y axis
	y_trans = ht.trans_y(-1.5)
	
	# Perform resulting h_2 calculation
	h_2 = np.matmul(z_trans,x_trans)
	h_2 = np.matmul(h_2,y_trans)
	print (f'Homogenous transformation H_2 result is: \n{h_2}\n')
	
	
	# Transfomation 3 (H_3)
	print ('=========================================================================')
	print ('\nTrasnformation H_3 ')
	print ('=========================================================================')
	print ('Resulting transformation will be fixedTransY.fixedTransZ.fixedTransX')
	print ('=========================================================================')
	# Pass -1.5 units along y axis
	y_trans = ht.trans_y(-1.5)
	
	# Pass 0.5 units along z axis
	z_trans = ht.trans_z(0.5)
	
	# Pass 2.5 units along x_axis
	x_trans = ht.trans_x(2.5)
	
	# Perform resulting h_3 calculation
	h_3 = np.matmul(y_trans,z_trans)
	h_3 = np.matmul(h_3,x_trans)
	print (f'Homogenous transformation H_3 result is: \n{h_3}\n')
	
	
	# Transfomation 4 (H_4)
	print ('===========================================================')
	print ('\nTrasnformation H_4 ')
	print ('===========================================================')
	print ('Resulting transformation will be fixedTransY.fixedTransX.fixedTransZ')
	print ('===========================================================')
	# Pass -1.5 units along y axis
	y_trans = ht.trans_y(-1.5)
	
	# Pass 2.5 units along x_axis
	x_trans = ht.trans_x(2.5)
	
	# Pass 0.5 units along z axis
	z_trans = ht.trans_z(0.5)
	
	# Perform resulting h_4 calculation
	h_4 = np.matmul(y_trans,x_trans)
	h_4 = np.matmul(h_4,z_trans)
	print (f'Homogenous transformation H_4 result is: \n{h_4}\n')

	
	
	# Transfomation 5 (H_5)
	print ('===========================================================')
	print ('\nTrasnformation H_5 ')
	print ('===========================================================')
	print ('Resulting transformation will be.... \n currrentRotX.currentTransX.currentTransZ.currentRotZ')
	print ('===========================================================')
	# Pass pi/2 in radians around x axis
	roll = ht.rot_x(math.pi/2)
	
	# Pass 3 units along x_axis
	x_trans = ht.trans_x(3)
	
	# Pass -3 units along z axis
	z_trans = ht.trans_z(-3)
	
	# Pass -pi/2 in radians around z axis
	yaw = ht.rot_z(-math.pi/2)
	
	# Perform resulting h_5 calculation
	h_5 = np.matmul(roll,x_trans)
	h_5 = np.matmul(h_5,z_trans)
	h_5 = np.matmul(h_5,yaw)
	
	# Print Result
	print (f'Homogenous transformation H_5 result is: \n{h_5}\n')
