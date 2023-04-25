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
		
			# (a) the camera as compared to the robot base
			trans_cam_chckbrd = tfBuffer.lookup_transform("base", "camera_color_optical_frame", rospy.Time())
			
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
		
		
		# Get the transform of the sphere camera fit in relation to the robot base frame board
		pt_in_base_ball = tfbuffer.lookup_transform ('base', pt_in_camera, rospy.Time())
		
		# extract the xyz coordinates
		x = pt_in_base_ball.transform.translation.x
		y = pt_in_base_ball.transform.translation.y
		z = pt_in_base_ball.transform.translation.z
		# extract the quaternion and convert to Roll, Pitch and Yaw
		q_rot = pt_in_base_ball.transform.rotation
		roll, pitch, yaw, = euler_from_quaternion([q_rot.x, q_rot.y, q_rot.z, q_rot.w])
		
		
		# Print the results of the sphere fit params with respect to the robot base frame
		print('Ball camera frame position and orientation w.r.t base: x= ', format(x, '.3f'), '(m),  y= ', format(y, '.3f'), '(m), z= ', format(z, '.3f'),'(m)')
		print('roll= ', format(roll, '.2f'), '(rad), pitch= ', format(pitch, '.2f'), '(rad), yaw: ', format(yaw, '.2f'),'(rad)') 
		print('-------------------------------------------------')
		
		
		# Pause till the next iteration
		loop_rate.sleep()

