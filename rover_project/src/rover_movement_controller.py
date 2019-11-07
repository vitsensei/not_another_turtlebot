#!/usr/bin/env python

import rospy
from gazebo_msgs.srv import GetModelState
from std_msgs.msg import Float64
import time

def callService(rover_name):
	# rover_name = "{model_name: " + rover_name + "}"
	rospy.wait_for_service("gazebo/get_model_state")
	try:
		get_model_state = rospy.ServiceProxy("gazebo/get_model_state", GetModelState)
		current_state = get_model_state(rover_name, "world")
		return current_state 

	except rospy.ServiceException as e:
		print("Service call failed: ",e)

def main():
	rospy.init_node("movement_controller_node")
	rate = rospy.Rate(1)
	left_wheel_pub = rospy.Publisher("/rover/left_wheel/command", Float64, queue_size=10)
	right_wheel_pub = rospy.Publisher("/rover/right_wheel/command", Float64, queue_size=10)

	desired_point = [10,10,0]

	while not rospy.is_shutdown():
		returned_state = callService("rover")
		print("Returned state: ",returned_state)

		left_wheel_cmd = 1
		right_wheel_cmd = -1

		left_wheel_pub.publish(left_wheel_cmd)
		right_wheel_pub.publish(right_wheel_cmd)

		rate.sleep()

if __name__ == '__main__':
	main()