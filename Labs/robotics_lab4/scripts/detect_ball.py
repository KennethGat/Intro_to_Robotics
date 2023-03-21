#!/usr/bin/env python3

'''
COURSE CODE:	CSC592 (SPRING 2023)
COURSE NAME:	INTRODUCTION TO ROBOTICS
INSTRUCTOR:		DR. HAMED SAEIDI
CODE CREDITS:	DR. HAMED SAEIDI
NAME:			KENNETH MURIUKI GATOBU
ASSIGNMENT:		ROBOTICS LAB 4
DATE:			MARCH 20 2023
'''


'''
============================================================================
				IMPORT NECESSARY MODULES
============================================================================
'''
import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

img_received = False

# define a 720x1280 3-channel image with all pixels equal to zero
rgb_img = np.zeros((720, 1280, 3), dtype = "uint8")


# get the image message
def get_image(ros_img):
	global rgb_img
	global img_received	
	# convert to opencv image
	rgb_img = CvBridge().imgmsg_to_cv2(ros_img, "rgb8")		
	# raise flag
	img_received = True

	
if __name__ == '__main__':
	# define the node and subcribers and publishers
	rospy.init_node('detect_ball', anonymous = True)
	# define a subscriber to read tennis ball images
	img_sub = rospy.Subscriber("/camera/color/image_raw", Image, get_image) 
	# define a publisher to publish images
	img_pub = rospy.Publisher('/ball_2D', Image, queue_size = 1)
	# set the loop frequency
	rate = rospy.Rate(10)

	while not rospy.is_shutdown():
		# make sure we process if the camera has started streaming images
		if img_received:
			# Filter the Image using HSV Color Space			
			hsv_img = cv2.cvtColor(rgb_img, cv2.COLOR_RGB2HSV)
			
			# Define the upper and lower hsv ranges
			lower_yellow_hsv = np.array([20,30,15])
			upper_yellow_hsv = np.array([60,255,255])
			
			# Filter the image to make the yellow ball stand out
			hsv_img_filtered = cv2.inRange(hsv_img, lower_yellow_hsv, upper_yellow_hsv)
			
			# convert it to ros msg and publish it
			img_msg = CvBridge().cv2_to_imgmsg(hsv_img_filtered, encoding="mono8")
			
			# publish the image
			img_pub.publish(img_msg)
		
		# pause until the next iteration			
		rate.sleep()
