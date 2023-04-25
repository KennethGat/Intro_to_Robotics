#!/usr/bin/env python3

'''
COURSE CODE:	CSC592 (SPRING 2023)
COURSE NAME: 	INTRODUCTION TO ROBOTICS
INSTRUCTOR:		DR. HAMED SAEIDI
CODE CREDITS:	DR. HAMED SAEIDI
NAME:			KENNETH MURIUKI GATOBU
ASSIGNMENT:		PHASE 2 REPORT
DATE:			APRIL 25 2023
'''


'''
===================================================================
				IMPORT NECESSARY MODULES
===================================================================
'''
import rospy
import tf2_ros
from tf.transformations import *
from geometry_msgs.msg import Quaternion
import tf2_geometry_msgs
from robot_vision_lectures.msg import SphereParams


'''
===================================================================
				DEFINE MESSAGE VARIABLE TO ENSURE DATA
						IS RECEIVED AS WELL AS
						OTHER GLOBAL VARIABLES
===================================================================
'''
msg_received = False
x_c = float()
y_c = float()
z_c = float()
radius = float()

'''
===================================================================
				DEFINE NECESSARY CALL BACK FUNCTIONS
===================================================================
'''
def call_back(data):
	'''
		Call back function that receives data from....
		/sphere_params topic and uses the data to extract 
		the ball's centre x,y,z co-ordinates and the radius		
	'''
	
	global msg_received
	global x_c
	global y_c
	global z_c
	global radius
	
	# Checker to ensure data is recieved
	msg_received = True
	
	# Get x,y,z co-ordinates of the 3D camera co-ordinate frame
	# and append it to the current_sphere_params list
	x_c = data.xc
	y_c = data.yc
	z_c = data.zc
	radius = data.radius

'''
===================================================================
				DEFINE MAIN FUNCTION
===================================================================
'''
if __name__ == '__main__':
	# initialize the node
	rospy.init_node('ros_tf_kenneth', anonymous = True)
	
	# add a subscriber to the /sphere_params topic to ......
	# get the estimated tennis ball parameters for use in frame calculations
	rospy.Subscriber("/sphere_params", SphereParams, call_back)
	
	# add a ros transform listener
	tfBuffer = tf2_ros.Buffer()
	listener = tf2_ros.TransformListener(tfBuffer)
	
	# set a 10Hz frequency for this loop
	loop_rate = rospy.Rate(10)

	# define a quaternion msg
	q_rot = Quaternion()
	
	# Initiate sphere fit old_params for use in error checking
	'''x_c_old = 0.0
	y_c_old = 0.0
	z_c_old = 0.0
	radius_old = 0.0'''
	
	while not rospy.is_shutdown():

		# Get the most recent updated transform info between
		# (This is also used as a safety check to ensure there are frames being
		# (published from the static frame broadcasters):
		try:
		
			# (a) the camera as compared to the checkerboard
			trans_cam_chckbrd = tfBuffer.lookup_transform("checkerboard", "camera_color_optical_frame", rospy.Time())
			
			# (b) the checkerboard as compared to the robot base frame
			trans_chckbrd_rbase = tfBuffer.lookup_transform("base", "checkerboard", rospy.Time())
			
			# (c) the robot tool tip as compared to the robot base frame
			trans_rtip_rbase = tfbuffer.lookup_transform("base", "fk_tooltip", rospy.Time())
			
		except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
			print('One or more frames not available!!!')
			loop_rate.sleep()
			continue
			
					
		# Define the mathematically derived sphere fit centre parameters as points in the camera frame
		pt_in_camera = tf2_geometry_msgs.PointStamped()
		pt_in_camera.header.frame_id = 'camera_color_optical_frame'
		pt_in_camera.header.stamp = rospy.get_rostime()
		pt_in_camera.x = x_c
		pt_in_camera.y = y_c
		pt_in_camera.z = z_c
		
		
		# Get the transform of the sphere camera fit in relation to the checker board
		pt_in_chckbrd_ball = tfBuffer.transform(pt_in_camera,'checkerboard',rospy.Duration(1.0))
		
		# Use the transform pt_in_chckbrd_ball to find out where the ball
		# is with respect to the robot
		pt_in_base_ball = tfbuffer.transform(pt_in_chckbrd_ball,'base', rospy.Duration(1.0))
		
		# Print the results of the sphere fit params with respect to the robot base frame
		print('Transformed sphere fit in the BASE frame:  x= ', format(pt_in_base_ball.point.x, '.3f'), '(m), y= ', format(pt_in_base_ball.point.y, '.3f'), '(m), z= ', format(pt_in_base_ball.point.z, '.3f'),'(m)')
		print('-------------------------------------------------')
		
		
		# Pause till the next iteration
		loop_rate.sleep()

