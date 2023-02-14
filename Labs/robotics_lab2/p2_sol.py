#!/usr/bin/env python3

'''
COURSE CODE:	CSC592 (SPRING 2023)
COURSE NAME:	INTRODUCTION TO ROBOTICS
NAME:			KENNETH MURIUKI GATOBU
ASSIGNMENT:		ROBOTICS LAB 2 (p2_sol)
TYPE:			BASIC HOMOGENEOUS TRANSFORMATIONS MODULE
DATE:			FEB 13 2023
'''

import math
import numpy as np

def trans_x(a):
	'''
	Unit Translational matrix along the x-axis
	that takes a value 'a' in displacment units and returns a
	translational x-axis homogeneous matrix
	'''

	trans = np.array([
					[1.0, 0.0, 0.0, a],
					[0.0, 1.0, 0.0, 0.0],
					[0.0, 0.0, 1.0, 0.0],
					[0.0, 0.0, 0.0, 1.0]
					])
	return trans



def trans_y(b):
	'''
	Unit Translational matrix along the y-axis
	that takes a value 'b' displacement units and returns a
	translational y-axis homogeneous matrix
	'''

	trans = np.array([
					[1.0, 0.0, 0.0, 0.0],
					[0.0, 1.0, 0.0, b],
					[0.0, 0.0, 1.0, 0.0],
					[0.0, 0.0, 0.0, 1.0]
					])
	return trans
	

	
def trans_z(c):
	'''
	Unit Translational matrix along the z-axis
	that takes a value 'c' in displacement units and returns a
	translational z-axis homogeneous matrix
	'''

	trans = np.array([
					[1.0, 0.0, 0.0, 0.0],
					[0.0, 1.0, 0.0, 0.0],
					[0.0, 0.0, 1.0, c],
					[0.0, 0.0, 0.0, 1.0]
					])
	return trans
	
	
def rot_x(alpha):
	"""
	Receives an input in radians and
	returns a Roll rotation homogeneous matrix
	"""
	roll = np.array([
					[1.0,  0.0, 0.0, 0.0],
				    [0.0, math.cos(alpha), -math.sin(alpha), 0.0],
					[0.0, math.sin(alpha), math.cos(alpha),  0.0],
					[0.0, 0.0, 0.0, 1.0]
					])
	return roll
	
	
def rot_y(beta):
	"""
	Receives an input in radians and
	returns a Pitch rotation homogeneous matrix
	"""
	pitch = np.array([
					[math.cos(beta),  0.0, math.sin(beta), 0.0],
				    [0.0, 1.0, 0.0, 0.0],
					[-math.sin(beta), 0.0, math.cos(beta), 0.0],
					[0.0, 0.0, 0.0, 1.0]
					])
	return pitch
	
	
def rot_z(gamma):
	"""
	Receives an input in radians and
	returns a Yaw rotation homogeneous matrix
	"""
	yaw = np.array([
					[math.cos(gamma),  -math.sin(gamma), 0.0, 0.0],
					[math.sin(gamma),   math.cos(gamma), 0.0, 0.0],
				    [0.0, 0.0, 1.0, 0.0],
				    [0.0, 0.0, 0.0, 1.0]
				    ])
	return yaw
