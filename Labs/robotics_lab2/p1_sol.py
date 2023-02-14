#!/usr/bin/env python3

'''
COURSE CODE:	CSC592 (SPRING 2023)
COURSE NAME:	INTRODUCTION TO ROBOTICS
NAME:			KENNETH MURIUKI GATOBU
ASSIGNMENT:		ROBOTICS LAB 2 (p1_sol)
TYPE:			Roll, Pitch and Yaw Calculation
DATE:			FEB 13 2023
'''

import rbm
import math
import numpy as np

if __name__ == '__main__':

	# Declare variable psi/theta/phi (Since all angles are the same as specified in problem)
	
	# Format output
	np.set_printoptions(precision = 3, suppress = True)
	
	# Set theta in radians since it is the same for all dimensions
	theta = math.pi/2
	
	# Calcultion of Roll with theta as pi/2
	print ('==============================================')
	print ('\nRoll rotation function: ')
	print ('==============================================')
	print (rbm.rot_x.__doc__)
	print ('==============================================')
	roll = rbm.rot_x(theta)
	print (f'Roll rotation matrix for pi/2 == {theta:.3f} is \n {roll} \n')
	
	# Calcultion of Pitch with theta as pi/2
	print ('\n==============================================')
	print ('\nPitch rotation function: ')
	print ('==============================================')
	print (rbm.rot_y.__doc__)
	print ('==============================================')
	pitch = rbm.rot_y(theta)
	print (f'Pitch rotation matrix for pi/2 == {theta:.3f} is \n {pitch} \n')
	
	# Calcultion of Yaw with theta as pi/2
	print ('\n==============================================')
	print ('\nYaw rotation function: ')
	print ('==============================================')
	print (rbm.rot_z.__doc__)
	print ('==============================================')
	yaw = rbm.rot_z(theta)
	print (f'Yaw rotation matrix for pi/2 == {theta:.3f} is \n {yaw} \n')
	
	
	# Final Rotation matrix of the fixed frame will be:
	# Yaw.Pitch.Roll
	print ('\nFinal Roll-Pitch-Yaw rotation following a fixed frame 0 for pi/2 \nover x, y & z will be (yaw.pitch.roll): ')
	print ('==========================================================')
	
	# Yaw.Pitch
	R = np.matmul(yaw,pitch, roll)
	
	# ...then Yaw.Pitch.Roll
	# R = np.matmul(R,roll)
	
	print (R)
	
	
