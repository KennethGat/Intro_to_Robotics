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


if __name__ == '__main__':
	# initialize the node
	rospy.init_node('ros_tf_kenneth', anonymous = True)
	# add a ros transform listener
	tfBuffer = tf2_ros.Buffer()
	listener = tf2_ros.TransformListener(tfBuffer)
	# set a 10Hz frequency for this loop
	loop_rate = rospy.Rate(10)

	q_rot = Quaternion()	
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
		# extract the xyz coordinates
		x = trans.transform.translation.x
		y = trans.transform.translation.y
		z = trans.transform.translation.z
		# extract the quaternion and converto RPY
		q_rot = trans.transform.rotation
		roll, pitch, yaw, = euler_from_quaternion([q_rot.x, q_rot.y, q_rot.z, q_rot.w])
		# a quick check of the readings
		print('Tool frame position and orientation w.r.t base: x= ', format(x, '.3f'), '(m),  y= ', format(y, '.3f'), '(m), z= ', format(z, '.3f'),'(m)')
		print('roll= ', format(roll, '.2f'), '(rad), pitch= ', format(pitch, '.2f'), '(rad), yaw: ', format(yaw, '.2f'),'(rad)') 
		# define a testpoint in the tool frame (let's say 10 cm away from flange)
		pt_in_tool = tf2_geometry_msgs.PointStamped()
		pt_in_tool.header.frame_id = 'fk_tooltip'
		pt_in_tool.header.stamp = rospy.get_rostime()
		pt_in_tool.point.z= 0.1 # 10 cm away from flange
		# convert the 3D point to the base frame coordinates
		pt_in_base = tfBuffer.transform(pt_in_tool,'base', rospy.Duration(1.0))
		print('Test point in the TOOL frame:  x= ', format(pt_in_tool.point.x, '.3f'), '(m), y= ', format(pt_in_tool.point.y, '.3f'), '(m), z= ', format(pt_in_tool.point.z, '.3f'),'(m)')
		print('Transformed point in the BASE frame:  x= ', format(pt_in_base.point.x, '.3f'), '(m), y= ', format(pt_in_base.point.y, '.3f'), '(m), z= ', format(pt_in_base.point.z, '.3f'),'(m)')
		print('-------------------------------------------------')
		# wait for 0.1 seconds until the next loop and repeat
		loop_rate.sleep()

