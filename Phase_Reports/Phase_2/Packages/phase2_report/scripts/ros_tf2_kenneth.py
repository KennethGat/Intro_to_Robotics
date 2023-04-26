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
xCentre_cur = float()
yCentre_cur = float()
zCentre_cur = float()
radius_cur = float()

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
	global xCentre_cur
	global yCentre_cur
	global zCentre_cur
	global radius_cur
	
	# Initiate sphere fit previous_params for use in error checking
	# Give arbitrary values to avoid divbyZero error
	xCentre_prev = 0.1
	yCentre_prev = 0.1
	zCentre_prev = 0.1
	radius_prev = 0.1
	
	# Get x,y,z co-ordinates of the 3D camera co-ordinate frame
	# and append it to the current_sphere_params list
	xCentre_cur = data.xc
	yCentre_cur = data.yc
	zCentre_cur = data.zc
	radius_cur = data.radius
	
	# Checker to ensure data is recieved
	msg_received = True
	
	# Error checking to ensure the sphere point cloud parameters are passed only when the crop fit has stabilized
		
	while abs(((xCentre_cur - xCentre_prev)/xCentre_prev)*100) > 0.1:
		xCentre_prev = xCentre_cur
		xCentre_cur = data.xc
		yCentre_cur = data.yc
		zCentre_cur = data.zc
		radius_cur = data.radius

'''
===================================================================
				DEFINE MAIN FUNCTION
===================================================================
'''
if __name__ == '__main__':
	# initialize the node
	rospy.init_node('ros_tf2_kenneth', anonymous = True)
	
	# add a subscriber to the /sphere_params topic to ......
	# get the estimated tennis ball parameters for use in frame calculations
	rospy.Subscriber('/sphere_params', SphereParams, call_back)
	
	# add a ros transform listener
	tfBuffer = tf2_ros.Buffer()
	listener = tf2_ros.TransformListener(tfBuffer)
	
	# set a 10Hz frequency for this loop
	loop_rate = rospy.Rate(10)

	# define a quaternion msg
	q_rot = Quaternion()
	
	
	while not rospy.is_shutdown():
		
		# Get the transform of the sphere camera fit in relation to the robot base frame board
		try:			
			trans_base_camera = tfBuffer.lookup_transform ('base', 'camera_color_optical_frame', rospy.Time())		
			
		except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
			print('One or more frames not available!!!')
			loop_rate.sleep()
			continue
			
		# extract the xyz coordinates
		x = trans_base_camera.transform.translation.x
		y = trans_base_camera.transform.translation.y
		z = trans_base_camera.transform.translation.z
		# extract the quaternion and convert to Roll, Pitch and Yaw
		q_rot = trans_base_camera.transform.rotation
		roll, pitch, yaw, = euler_from_quaternion([q_rot.x, q_rot.y, q_rot.z, q_rot.w])
		
		
		# Print the results of the camera frame with respect to the robot base frame
		'''
		print('Camera frame position and orientation w.r.t Base: x= ', format(x, '.3f'), '(m),  y= ', format(y, '.3f'), '(m), z= ', format(z, '.3f'),'(m)')
		print('roll= ', format(roll, '.2f'), '(rad), pitch= ', format(pitch, '.2f'), '(rad), yaw: ', format(yaw, '.2f'),'(rad)') 
		print('-------------------------------------------------')
		'''
				
		# Define the mathematically derived sphere fit centre parameters as points in the camera frame
		pt_in_camera = tf2_geometry_msgs.PointStamped()
		pt_in_camera.header.frame_id = 'camera_color_optical_frame'
		pt_in_camera.header.stamp = rospy.get_rostime()
		pt_in_camera.point.x = xCentre_cur
		pt_in_camera.point.y = yCentre_cur
		pt_in_camera.point.z = zCentre_cur
		
		# Get the tranformation of the ball camera frame with respect to the robot base frame
		pt_in_base = tfBuffer.transform(pt_in_camera,'base',rospy.Duration(1.0))
		
		
		print('Transformed ball camera point in the BASE frame:  x= ', format(pt_in_base.point.x, '.3f'), '(m), y= ', format(pt_in_base.point.y, '.3f'), '(m), z= ', format(pt_in_base.point.z, '.3f'),'(m)')
		
		
		print(pt_in_base)
		
		# Pause till the next iteration
		loop_rate.sleep()

