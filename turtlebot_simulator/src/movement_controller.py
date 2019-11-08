#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, Vector3
from turtlesim.msg import Pose as turtle_pose

import time
import numpy as np
from math import atan, atan2, sin, cos

desired_point = np.ones((2,))*5
current_point = np.ones((2,))*5
current_heading = 0

rotation_matrix = np.array([[0, -1],[1, 0]])

integrated_error_heading = 0
integrated_error_distance = 0

def cmd_callback(data):
	global desired_point

	print("Here's what I got: ",data.position.x, data.position.y)
	desired_point = np.array([data.position.x, data.position.y])

def pose_callback(data):
	global current_point
	global current_heading

	current_point = np.array([data.x, data.y])
	current_heading = data.theta
	current_heading = atan2(sin(current_heading),cos(current_heading))

def heading_pid_controller(error_heading):
	global integrated_error_heading

	kp = 2
	ki = 0.1
	kd = 00
	if (abs(integrated_error_heading) < 3.14):
		integrated_error_heading = integrated_error_heading + error_heading

	angular_vel = kp*error_heading + ki*integrated_error_heading

	return angular_vel

def compute_path():
	global current_point
	global desired_point
	global current_heading
	global integrated_error_distance

	error_vector = desired_point - current_point
	if np.linalg.norm(error_vector) == 0:
		return 0, 0

	k_a = atan(np.linalg.norm(error_vector))

	control_vector = (k_a*(error_vector/np.linalg.norm(error_vector)))

	linear_vel = np.linalg.norm(control_vector)

	desired_heading = atan2(control_vector[1],control_vector[0])

	print("Desired heading: ", desired_heading/3.14*180)
	print("Current heading: ", current_heading/3.14*180)

	print("Linear velocity: ", linear_vel)
	print("Error vector: ", error_vector)

	error_heading = atan2(sin(desired_heading-current_heading),cos(desired_heading-current_heading))

	angular_vel = heading_pid_controller(error_heading)
	print("Angular velocity: ", angular_vel)

	return linear_vel, angular_vel

def main():
	global current_point, desired_point

	rospy.init_node("movement_controller")
	command_feed = rospy.Subscriber("turtlebot_command_topic", Pose, cmd_callback)
	turtlepose_feed = rospy.Subscriber("/turtle1/pose", turtle_pose, pose_callback)

	movement_control_pub = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)

	# rospy.spin()
	rate = rospy.Rate(20)
	time.sleep(1)

	desired_point = current_point
	print("Before running")
	print("Current point: ", current_point)
	print("Desired point: ", desired_point)
	while not rospy.is_shutdown(): 
		linear_vel, angular_vel = compute_path()

		new_twist = Twist(linear= Vector3(linear_vel, 0, 0), angular= Vector3(0, 0, angular_vel))

		movement_control_pub.publish(new_twist)

		rate.sleep()

if __name__ == '__main__':
	main()