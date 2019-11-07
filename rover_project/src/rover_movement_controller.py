#!/usr/bin/env python

import rospy
from gazebo_msgs.srv import GetModelState
from std_msgs.msg import Float64

import numpy as np

integrated_heading_error = 0;

def callService(rover_name):
	# rover_name = "{model_name: " + rover_name + "}"
	rospy.wait_for_service("gazebo/get_model_state")
	try:
		get_model_state = rospy.ServiceProxy("gazebo/get_model_state", GetModelState)
		current_state = get_model_state(rover_name, "world")
		return current_state 

	except rospy.ServiceException as e:
		print("Service call failed: ",e)

def heading_pid_controller(heading_error):
	global integrated_heading_error

	kp = 1
	ki = 0.1
	kd = 0

	if (abs(integrated_heading_error) < 3.14):
		integrated_heading_error = integrated_heading_error + heading_error

	angular_vel = kp*heading_error + ki*integrated_heading_error

	return angular_vel

def control_vector_compute(current_point, desired_point, current_heading):
	error_vector = desired_point - current_point

	k = atan2(np.linalg.norm(error_vector)/5,1)
	control_vector = k*error_vector

	linear_vel = np.linalg.norm(control_vector)
	desired_heading = atan(control_vector[2]/control_vector[1])

	delta_heading = atan2(sin(desired_heading-current_heading)/cos(desired_heading-current_heading))

	angular_vel = heading_pid_controller(delta_heading)

	left_wheel_cmd = (2*linear_vel + angular_vel*1.1)/2
	right_wheel_cmd = -(2*linear_vel - angular_vel*1.1)/2

	return left_wheel_cmd, right_wheel_cmd

def main():
	rospy.init_node("movement_controller_node")
	rate = rospy.Rate(1)
	left_wheel_pub = rospy.Publisher("/rover/left_wheel/command", Float64, queue_size=10)
	right_wheel_pub = rospy.Publisher("/rover/right_wheel/command", Float64, queue_size=10)

	desired_point = [10,10,0]

	while not rospy.is_shutdown():
		returned_state = callService("rover")
		# print("Returned state: ",returned_state)
		print("Rover's pose:")
		print("X: ", returned_state.pose.position.x)
		print("Y: ", returned_state.pose.position.y)
		print("Phi: ", returned_state.pose.orientation.z)

		left_wheel_cmd = 2
		right_wheel_cmd = -1

		left_wheel_pub.publish(left_wheel_cmd)
		right_wheel_pub.publish(right_wheel_cmd)

		rate.sleep()

if __name__ == '__main__':
	main()