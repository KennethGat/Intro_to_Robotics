# import the rigid body motion module
import rbm
import math
import numpy as np

if __name__ == '__main__':
	# define a test value 
	theta = math.pi/4 
	print('rot_2d function:', rbm.rot_2d.__doc__)
	print('a %.2f radian rotation in 2D is shown by'%theta)
	# update the output format
	np.set_printoptions(precision = 2, suppress = True)
	print(rbm.rot_2d(theta))
	print(np.linalg.det(rbm.rot_2d(theta)))
	print(rbm.rot_x(theta))
	
