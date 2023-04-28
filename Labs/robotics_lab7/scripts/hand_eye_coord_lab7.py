#!/usr/bin/env python3

'''
COURSE CODE:	CSC592 (SPRING 2023)
COURSE NAME: 	INTRODUCTION TO ROBOTICS
INSTRUCTOR:		DR. HAMED SAEIDI
CODE CREDITS:	DR. HAMED SAEIDI
NAME:			KENNETH MURIUKI GATOBU
ASSIGNMENT:		LAB 7
DATE:			APRIL 27 2023
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

# import the plan message
from ur5e_control.msg import Plan
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt8


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

	
	# Error checking to ensure the sphere point cloud parameters are passed only when the crop fit has stabilized
		
	while abs(((xCentre_cur - xCentre_prev)/xCentre_prev)*100) > 0.01 or \
		abs(((yCentre_cur - yCentre_prev)/yCentre_prev)*100) > 0.01 or \
		abs(((zCentre_cur - zCentre_prev)/zCentre_prev)*100) > 0.01 or \
		abs(((radius_cur - radius_prev)/radius_prev)*100) > 0.01:
		
		xCentre_prev, xCentre_cur = xCentre_cur, data.xc
		yCentre_prev, yCentre_cur = yCentre_cur, data.yc
		zCentre_prev, zCentre_cur = zCentre_cur, data.zc
		radius_prev, radius_cur = radius_cur, data.radius
	
	# Checker to ensure data is recieved
	msg_received = True	

'''
===================================================================
				DEFINE MAIN FUNCTION
===================================================================
'''
if __name__ == '__main__':
	# initialize the node
	rospy.init_node('ros_tf2_kenneth', anonymous = True)
	
	# add a publisher for sending joint position commands
	plan_pub = rospy.Publisher('/plan', Plan, queue_size = 10)
	
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
		if msg_received:
		
			# Prompt to ensure the sphere point cloud has stalibilized
			print ("\nSphere cloud point stabilized!!!")
			print ("================================\n")

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
			
			# Get the transformation of the ball camera frame with respect to the robot base frame
			pt_in_base = tfBuffer.transform(pt_in_camera,'base',rospy.Duration(1.0))
			
			# Print the resultant ball camera frame with respect to the robot base frame
			print('Transformed ball camera point in the BASE frame:  x= ', format(pt_in_base.point.x, '.3f'), '(m), y= ', format(pt_in_base.point.y, '.3f'), '(m), z= ', format(pt_in_base.point.z, '.3f'),'(m)')
			
			# define a plan variable
			plan = Plan()
			
			# Implement the planner

			# define a point near pickup position			
			plan_point1 = Twist()
			point_mode1 = UInt8()
			plan_point1.linear.x = -0.022072
			plan_point1.linear.y = -0.383
			plan_point1.linear.z = 0.39035
			plan_point1.angular.x = 3.02043
			plan_point1.angular.y = 0.3132
			plan_point1.angular.z = 1.5132
			point_mode1.data = 0
			# add this point to the plan
			plan.points.append(plan_point1)
			plan.modes.append(point_mode1)


			# define a point for the pickup position with GRIPPER IN...
			# REGULAR MOTION MODE
			# x, y & z co-ordinates referenced from the 
			# derived sphere fit frame w.r.t. ur5e base frame
			plan_point2_0 = Twist()
			point_mode2_0 = UInt8()
			plan_point2_0.linear.x = pt_in_base.point.x
			plan_point2_0.linear.y = pt_in_base.point.y
			plan_point2_0.linear.z = pt_in_base.point.z + 0.17 #Adjustment for flanger
			plan_point2_0.angular.x = 3.13381
			plan_point2_0.angular.y = 0.06079
			plan_point2_0.angular.z = 0.27032
			point_mode2_0.data = 0
			# add this point to the plan
			plan.points.append(plan_point2_0)
			plan.modes.append(point_mode2_0)

			
			# define a point for the pickup position with GRIPPER CLOSED
			# x, y & z co-ordinates referenced from the 
			# derived sphere fit frame w.r.t. ur5e base frame
			plan_point2_1 = Twist()
			point_mode2_1 = UInt8()
			plan_point2_1.linear.x = pt_in_base.point.x
			plan_point2_1.linear.y = pt_in_base.point.y
			plan_point2_1.linear.z = pt_in_base.point.z + 0.17 #Adjustment for flanger
			plan_point2_1.angular.x = 3.13381
			plan_point2_1.angular.y = 0.06079
			plan_point2_1.angular.z = 0.27032
			point_mode2_1.data = 2
			# add this point to the plan
			plan.points.append(plan_point2_1)
			plan.modes.append(point_mode2_1)
			
			
		
			# define a point for the pickup position with GRIPPER back to...
			# REGULAR MOTION MODE
			# x, y & z co-ordinates referenced from the 
			# derived sphere fit frame w.r.t. ur5e base frame
			plan_point2_2 = Twist()
			point_mode2_2 = UInt8()
			plan_point2_2.linear.x = pt_in_base.point.x
			plan_point2_2.linear.y = pt_in_base.point.y
			plan_point2_2.linear.z = pt_in_base.point.z + 0.17 #Adjustment for flanger
			plan_point2_2.angular.x = 3.13381
			plan_point2_2.angular.y = 0.06079
			plan_point2_2.angular.z = 0.27032
			point_mode2_2.data = 0
			# add this point to the plan
			plan.points.append(plan_point2_2)
			plan.modes.append(point_mode2_2)
		
			
			# define a point for a safe position above 'place' point 
			plan_point3 = Twist()
			point_mode3 = UInt8()
			plan_point3.linear.x = -0.37053
			plan_point3.linear.y = -0.2878
			plan_point3.linear.z = 0.421558
			plan_point3.angular.x = 2.5016
			plan_point3.angular.y = 0.04878
			plan_point3.angular.z = -0.64514
			point_mode3.data = 0
			# add this point to the plan
			plan.points.append(plan_point3)
			plan.modes.append(point_mode3)
			
			
			
			# Final 'place/drop' position definition with gripper in...
			# REGULAR MOTION MODE
			plan_point4_0 = Twist()
			point_mode4_0 = UInt8()
			plan_point4_0.linear.x = -0.36131
			plan_point4_0.linear.y = -0.2743
			plan_point4_0.linear.z = 0.177717
			plan_point4_0.angular.x = 3.12911
			plan_point4_0.angular.y = 0.06079
			plan_point4_0.angular.z = -0.609966
			point_mode4_0.data = 0
			# add this point to the plan
			plan.points.append(plan_point4_0)
			plan.modes.append(point_mode4_0)

			
			# Final 'place/drop' position definition with gripper in...
			# OPEN MODE
			plan_point4_1 = Twist()
			point_mode4_1 = UInt8()
			plan_point4_1.linear.x = -0.36131
			plan_point4_1.linear.y = -0.2743
			plan_point4_1.linear.z = 0.177717
			plan_point4_1.angular.x = 3.12911
			plan_point4_1.angular.y = 0.06079
			plan_point4_1.angular.z = -0.609966
			point_mode4_1.data = 1
			# add this point to the plan
			plan.points.append(plan_point4_1)
			plan.modes.append(point_mode4_1)
			
			
			
			# Final 'place/drop' position definition with gripper BACK TO...
			# REGULAR MOTION MODE
			plan_point4_2 = Twist()
			point_mode4_2 = UInt8()
			plan_point4_2.linear.x = -0.36131
			plan_point4_2.linear.y = -0.2743
			plan_point4_2.linear.z = 0.177717
			plan_point4_2.angular.x = 3.12911
			plan_point4_2.angular.y = 0.06079
			plan_point4_2.angular.z = -0.609966
			point_mode4_2.data = 0
			# add this point to the plan
			plan.points.append(plan_point4_2)
			plan.modes.append(point_mode4_2) 
	
			
			
			# Back to initial position
			plan_point5 = Twist()
			point_mode5 = UInt8()
			plan_point5.linear.x = -0.022072
			plan_point5.linear.y = -0.383
			plan_point5.linear.z = 0.39035
			plan_point5.angular.x = 3.02043
			plan_point5.angular.y = 0.3132
			plan_point5.angular.z = 1.5132
			point_mode5.data = 0
			# add this point to the plan
			plan.points.append(plan_point5)
			plan.modes.append(point_mode5)

	
			# publish the plan
			plan_pub.publish(plan)		
		
		# Pause till the next iteration
		loop_rate.sleep()
