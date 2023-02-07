#!/usr/bin/env python3

'''
COURSE CODE:	CSC592 (SPRING 2023)
COURSE NAME:	INTRODUCTION TO ROBOTICS
NAME:			KENNETH MURIUKI GATOBU
ASSIGNMENT:		ROBOTICS LAB 1
DATE:			FEB 6 2023
'''


'''
============================================================================
				IMPORT NECESSARY MODULES
============================================================================
'''

# import ROS for developing the node
import rospy

# Import Pose message to get the position of the turtle
from turtlesim.msg import Pose

# import geometry_msgs/Twist for control commands
from geometry_msgs.msg import Twist

# Import the 'Turtlecontrol' message from 'robotics_lab1' package
from robotics_lab1.msg import Turtlecontrol



'''
============================================================================
				DEFINE MESSAGE VARIABLES
============================================================================
'''
# Define variable current_pos_msg based on Turtle Pos Message
current_pos_msg = Pose()

# Define variable desired_pos_msg based on the custom Turtlecontrol Message
# ..to publish the desired gain and final destination
desired_pos_msg = Turtlecontrol()

'''
============================================================================
			DEFINE NECESSARY FUNCTIONS TO GET REQUIRED VARIABLES
============================================================================
'''
# Function to get the current x-axis position of Turtle Bot
def pos_callback(data):

	
	# Declare Global Pose() Message Variable
	global current_pos_msg
	
	# Receive x position
	current_pos_msg.x = data.x
	
	# Show the results
	rospy.loginfo("x is %0.2f cm", (current_pos_msg.x * 100))
	


# Function to get the current control gain and final destination
def desired_pos_gain_callback(data):
	
	# Declare Global Turtlecontrol() Message Variable
	global desired_pos_msg
	
	# Receive kp and xd
	desired_pos_msg.kp = data.kp
	desired_pos_msg.xd = data.xd
	
'''

=====================================================================================
					DEFINE MAIN FUNCTION
=====================================================================================
'''
if __name__ == '__main__':

	# Initialize the node
	rospy.init_node('turtlebot_motion_control', anonymous = True)

	# Add 1st subscriber to read the position information
	rospy.Subscriber('/turtle1/pose', Pose, pos_callback)
		
	# Add 2nd subscriber to a new topic that receives the desired position and a control gain
	rospy.Subscriber('turtle1/control_params', Turtlecontrol, desired_pos_gain_callback)
		
	# Add a proportional control that publishes to the velocity command topic of the turtlesim node
	prop_control_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size = 10)
	
	# Declare a variable of type Twist for sending control commands
	vel_cmd = Twist()
	
	# Set a 10Hz frequncy for this loop
	loop_rate = rospy.Rate(10)

	# Run this control loop regularly
	while not rospy.is_shutdown():
			
		# Set the linear (forward/backward) velocity command
		vel_cmd.linear.x = desired_pos_msg.kp * (desired_pos_msg.xd - current_pos_msg.x)
		
		# Publish the command to the defined topic
		prop_control_pub.publish(vel_cmd)		
		
		# Wait for 0.1 seconds until the next loop and repeat
		loop_rate.sleep()
