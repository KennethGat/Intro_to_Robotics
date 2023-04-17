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
	# define a point for the pickup position
	plan_point1.linear.x = -0.6755
	plan_point1.linear.y = -0.1276
	plan_point1.linear.z = 0.2989
	plan_point1.angular.x = -3.076
	plan_point1.angular.y = -0.059
	plan_point1.angular.z = 1.567
	# add this point to the plan
	plan.points.append(plan_point1)
	
	plan_point2 = Twist()
	# define a point for a safe position above 'place' point 
	plan_point2.linear.x = -0.5438
	plan_point2.linear.y = 0.3359
	plan_point2.linear.z = 0.4952
	plan_point2.angular.x = -2.7618
	plan_point2.angular.y = -0.05499
	plan_point2.angular.z = 0.794875
	# add this point to the plan
	plan.points.append(plan_point2)
	
	
	plan_point3 = Twist()
	# Final 'place/drop' position definition
	plan_point3.linear.x = -0.579
	plan_point3.linear.y = 0.369
	plan_point3.linear.z = 0.2989
	plan_point3.angular.x = -3.0764
	plan_point3.angular.y = -0.0591
	plan_point3.angular.z = 0.813
	# add this point to the plan
	plan.points.append(plan_point3)
	
	
	plan_point4 = Twist()
	# Back to the safe initial position as defined in the
	# manual_initialization module
	plan_point4.linear.x = -0.626
	plan_point4.linear.y = -0.127
	plan_point4.linear.z = 0.495
	plan_point4.angular.x = -2.762
	plan_point4.angular.y = -0.05499
	plan_point4.angular.z = 1.549
	# add this point to the plan
	plan.points.append(plan_point4)

	
	
	while not rospy.is_shutdown():
		# publish the plan
		plan_pub.publish(plan)
		# wait for 0.1 seconds until the next loop and repeat
		loop_rate.sleep()
