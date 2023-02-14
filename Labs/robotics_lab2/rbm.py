import math
import numpy as np

def rot_2d(theta):
	"""
	Receives an input in radians and
	returns a 2D rotation matrix
	R = [cos(theta) -sin(theta)
	     sin(theta)  cos(theta)]
	"""
	rot = np.array([[math.cos(theta), -math.sin(theta)],
					[math.sin(theta), math.cos(theta)]])
	return rot

# use lecture slides for the mathematical equations 
def rot_x(psi):
	"""
	Receives an input in radians and
	returns a Roll rotation matrix
	R = [	1.0	0.0	0.0
		0.0 cos(theta) -sin(theta)
	     	0.0 sin(theta) cos(theta)	]
	"""
	rot = np.array([
					[1.0,  0.0, 0.0],
				    [0.0, math.cos(psi), -math.sin(psi)],
					[0.0, math.sin(psi), math.cos(psi)]
					])
	return rot
	
	
def rot_y(theta):
	"""
	Receives an input in radians and
	returns a Pitch rotation matrix
	R = [	cos(theta)	0.0	sin(theta)
		0.0		1.0	0.0
	     	-sin(theta)	0.0	cos(theta)	]
	"""
	rot = np.array([
					[math.cos(theta),  0.0, math.sin(theta)],
				    [0.0, 1.0, 0.0],
					[-math.sin(theta), 0.0, math.cos(theta)]
					])
	return rot
	
	
def rot_z(phi):
	"""
	Receives an input in radians and
	returns a Yaw rotation matrix
	R = [	cos(theta) -sin(theta 0.0)
		sin(theta) cos(theta) 0.0
	     	0.0	   0.0	      1.0	]
	"""
	rot = np.array([
					[math.cos(phi),  -math.sin(phi), 0.0],
					[math.sin(phi),   math.cos(phi), 0.0],
				    [0.0, 0.0, 1.0]
				    ])
	return rot
	
def vec(x,y,z):
	'''
	Define a vector as a numpy and transpose it to a column vector.
	'''
	vec = np.array([[x, y, z]]).T 
	return 
