#!/usr/bin/env python3

import rospy
import math

# import the plan message
from ur5e_control.msg import Plan
from geometry_msgs.msg import Twist

if __name__ == '__main__':
	# initialize the node
	rospy.init_node('simple_planner', anonymous = True)
	# add a publisher for sending joint position commands
	plan_pub = rospy.Publisher('/plan', Plan, queue_size = 10)
	# set a 10Hz frequency for this loop
	loop_rate = rospy.Rate(10)

	# define a plan variable
	plan = Plan()


	
	plan_point1 = Twist()
	# define a point near pickup position
	plan_point1.linear.x = -0.022072
	plan_point1.linear.y = -0.383
	plan_point1.linear.z = 0.39035
	plan_point1.angular.x = 3.02043
	plan_point1.angular.y = 0.3132
	plan_point1.angular.z = 1.5132
	# add this point to the plan
	plan.points.append(plan_point1)
	
	plan_point2 = Twist()
	# define a point for the pickup position
	plan_point2.linear.x = -0.022591
	plan_point2.linear.y = -0.463483
	plan_point2.linear.z = 0.0192461 + 0.17
	plan_point2.angular.x = 3.13381
	plan_point2.angular.y = 0.06079
	plan_point2.angular.z = 0.27032
	# add this point to the plan
	plan.points.append(plan_point2)
	
	plan_point3 = Twist()
	# define a point for a safe position above 'place' point 
	plan_point3.linear.x = -0.37053
	plan_point3.linear.y = -0.2878
	plan_point3.linear.z = 0.421558
	plan_point3.angular.x = 2.5016
	plan_point3.angular.y = 0.04878
	plan_point3.angular.z = -0.64514
	# add this point to the plan
	plan.points.append(plan_point3)
	
	
	plan_point4 = Twist()
	# Final 'place/drop' position definition
	plan_point4.linear.x = -0.36131
	plan_point4.linear.y = -0.2743
	plan_point4.linear.z = 0.177717
	plan_point4.angular.x = 3.12911
	plan_point4.angular.y = 0.06079
	plan_point4.angular.z = -0.609966
	# add this point to the plan
	plan.points.append(plan_point4)
	
	
	plan_point5 = Twist()
	# Back to initial position
	plan_point5.linear.x = -0.022072
	plan_point5.linear.y = -0.383
	plan_point5.linear.z = 0.39035
	plan_point5.angular.x = 3.02043
	plan_point5.angular.y = 0.3132
	plan_point5.angular.z = 1.5132
	# add this point to the plan
	plan.points.append(plan_point5)

	
	
	while not rospy.is_shutdown():
		# publish the plan
		plan_pub.publish(plan)
		# wait for 0.1 seconds until the next loop and repeat
		loop_rate.sleep()
