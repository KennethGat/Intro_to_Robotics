#!/usr/bin/env python3

'''
COURSE CODE:	CSC592 (SPRING 2023)
COURSE NAME:	INTRODUCTION TO ROBOTICS
NAME:		KENNETH MURIUKI GATOBU
ASSIGNMENT:	ROBOTICS LAB 1
DATE:		FEB 6 2023
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

# Define variable pos_msg to get the current position of the turtle bot
current_pos_msg = Pose()

# Define variable desired_pos_msg based on the Turtlecontrol Message created
# ..to publish the desired gain and final destination
desired_pos_msg = Turtlecontrol()

'''
============================================================================
		DEFINE NECESSARY FUNCTIONS TO GET REQUIRED VARIABLES
============================================================================
'''
# Function to get the current x-axis postion of Turtle Bot
def pose_callback(data):

	# Declare Global Pose() Message Variable
	global current_pos_msg
	
	# Receive x position while converting to cm
	current_pos_msg.x = data.x
	
	# Show the results
	rospy.loginfo("x is %0.2f cm", (current_pos_msg.x * 100))
	
	
# Function to assign control gain and desired position to Turtlecontrol() Message
def final_param(gain, final):
	
	# Declare Global Turtlecontrol() Message Variable
	global desired_pos_msg
	
	# Assign control gain and final destination parameters to desired_pos_msg()
	desired_pos_msg.kp = gain
	desired_pos_msg.xd = final			


'''
=====================================================================================
				DEFINE MAIN FUNCTION
=====================================================================================
'''
if __name__ == '__main__':

	# Initialize the node
	rospy.init_node('turtlebot_motion_control', anonymous = True)

	# Get control gain and final destination parameters from end user
	gain = float(input("Enter Control Gain Kp: "))
	final = float(input("Enter Final Destination Xd (Meters): "))
	
	# Declare a variable of type Twist for sending control commands
	vel_cmd = Twist()
	
	# Set a 10Hz frequncy for this loop
	loop_rate = rospy.Rate(10)
	
	# Run this control loop regularly
	while not rospy.is_shutdown():
	
		# Add 1st subscriber to read the position information
		rospy.Subscriber('/turtle1/pose', Pose, pose_callback)
		
		# Add 2nd subscriber to a new topic that receives the desired position and a control gain
		rospy.Subscriber('turtle1/control_params', Turtlecontrol, final_param(gain, final))
		
		# Add a proportional control that publishes to the velocity command topic of the turtlesim node
		prop_control_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size = 10)
		
		# Set tolerance level at which the turtle bot should stop
		if (desired_pos_msg.xd - current_pos_msg.x) > 0.3:
			
			# Set the linear (forward/backward) velocity command
			vel_cmd.linear.x = desired_pos_msg.kp * (desired_pos_msg.xd - current_pos_msg.x)
			
		else:
			# Set the linear (forward/backward) velocity to 0 if tolerance condition < 0.3
			vel_cmd.linear.x = 0
		
		# Publish the command to the defined topic
		prop_control_pub.publish(vel_cmd)		
		
		# Wait for 0.1 seconds until the next loop and repeat
		loop_rate.sleep()

