#!/usr/bin/env python

import rospy
from gazebo_msgs.srv import GetModelState
import time

def callService(rover_name):
	# rover_name = "{model_name: " + rover_name + "}"
	rospy.wait_for_service("gazebo/get_model_state")
	try:
		get_model_state = rospy.ServiceProxy("gazebo/get_model_state", GetModelState)
		current_state = get_model_state(rover_name, "dummy_link")
		return current_state 

	except rospy.ServiceException as e:
		print("Service call failed: ",e)

def main():
	desired_point = [10,10,0]

	while 1:
		returned_state = callService("rover")
		print("Returned state: ",returned_state)
		time.sleep(1)

if __name__ == '__main__':
	main()