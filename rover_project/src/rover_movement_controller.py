#!/usr/bin/env python

import rospy
from math import atan2, sin, cos, atan, sqrt
from gazebo_msgs.srv import GetModelState, SpawnModel, DeleteModel
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose, Point, Quaternion
from controller_manager_msgs.srv import LoadController, LoadControllerRequest
from urdf_parser_py.urdf import URDF

import numpy as np
from copy import deepcopy

integrated_heading_error = 0
previous_heading_error = 0

def spawn_myRover(rover_name):
	rospy.wait_for_service("/gazebo/spawn_urdf_model")
	rospy.wait_for_service("/gazebo/delete_model")

	# Delete existing model
	deleteModel = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
	deleteModel(rover_name)

	# Spawn the model
	rover_spawner = rospy.ServiceProxy("/gazebo/spawn_urdf_model", SpawnModel)
	rover_spawner(rover_name, open("/home/anh/catkin_ws/src/rover_project/rover_description/urdf/rover.urdf",'r').read(), "/rover", Pose(position= Point(0,0,2),orientation=Quaternion(0,0,0,0)),"world")

	rospy.wait_for_service('/rover/controller_manager/load_controller')

	# Load the controller
	controller_spawner = rospy.ServiceProxy('/rover/controller_manager/load_controller', LoadController)
	controller_spawner("/rover/left_wheel")
	controller_spawner("/rover/right_wheel")
	controller_spawner("/rover/joint_state_controller")

def compute_yaw(current_state):
	current_state_fnc = deepcopy(current_state)
	q1 = current_state_fnc.pose.orientation.x
	q2 = current_state_fnc.pose.orientation.y
	q3 = current_state_fnc.pose.orientation.z
	q0 = current_state_fnc.pose.orientation.w

	# calculate yaw using Wikipedia guideline
	yaw = atan2(2*(q0*q3 + q1*q2),1-2*(q2*q2+q3*q3))

	return atan2(sin(yaw + 1.5708),cos(yaw + 1.5708))

def getCurrentState(rover_name):
	# rover_name = "{model_name: " + rover_name + "}"
	rospy.wait_for_service("gazebo/get_model_state")

	get_model_state = rospy.ServiceProxy("gazebo/get_model_state", GetModelState)
	current_state = get_model_state(rover_name, "world")
	return current_state 


def heading_pid_controller(heading_error):
	global integrated_heading_error
	global previous_heading_error

	heading_error_fnc = deepcopy(heading_error)

	kp = 4.5
	ki = 0
	kd = 0

	if ((abs(integrated_heading_error) < 0.5) or (heading_error_fnc*integrated_heading_error<0)):
		integrated_heading_error = integrated_heading_error + heading_error_fnc

	angular_vel = kp*heading_error_fnc + ki*integrated_heading_error + kd*(heading_error_fnc - previous_heading_error)
	previous_heading_error = heading_error_fnc

	return angular_vel

def control_vector_compute(current_point, desired_point, current_heading):
	global integrated_heading_error

	current_point_fnc = deepcopy(current_point)
	desired_point_fnc = deepcopy(desired_point)
	current_heading_fnc = deepcopy(current_heading)

	error_vector = desired_point_fnc - current_point_fnc

	if np.linalg.norm(error_vector) == 0:
		return 0, 0

	k = atan(np.linalg.norm(error_vector))
	control_vector = (5*k*(error_vector/np.linalg.norm(error_vector)))
	linear_vel = np.linalg.norm(control_vector)

	desired_heading = atan2(control_vector[1],control_vector[0])

	error_heading = atan2(sin(desired_heading-current_heading_fnc),cos(desired_heading-current_heading_fnc))

	angular_vel = heading_pid_controller(error_heading)

	left_wheel_cmd = (2*linear_vel - angular_vel*1.1)/(2*0.25)
	right_wheel_cmd = -(2*linear_vel + angular_vel*1.1)/(2*0.25)

	return left_wheel_cmd, right_wheel_cmd

def main():
	global integrated_heading_error

	rospy.init_node("movement_controller_node")
	# spawn_myRover("rover")
	rate = rospy.Rate(20)
	left_wheel_pub = rospy.Publisher("/rover/left_wheel/command", Float64, queue_size=20)
	right_wheel_pub = rospy.Publisher("/rover/right_wheel/command", Float64, queue_size=20)

	desired_point = [0,0]

	while not rospy.is_shutdown():
		returned_state = getCurrentState("rover")
		# print("Returned state: ",returned_state)
		# print("Yaw: ", compute_yaw(returned_state)/3.14*180)
		print("Rover's status:")
		print("X: ", returned_state.pose.position.x)
		print("Y: ", returned_state.pose.position.y)
		print("Phi: ", compute_yaw(returned_state)/3.14*180)

		current_point = [returned_state.pose.position.x, returned_state.pose.position.y]
		current_heading = compute_yaw(returned_state)

		if np.linalg.norm(np.array(desired_point) - np.array(current_point)) < 1:
			left_wheel_cmd, right_wheel_cmd = 0, 0
			integrated_heading_error = 0

		else:
			left_wheel_cmd, right_wheel_cmd = control_vector_compute(np.array(current_point), np.array(desired_point), current_heading)
		
		print("Left wheel speed: ", left_wheel_cmd)
		print("Right wheel speed: ", right_wheel_cmd)
		left_wheel_pub.publish(left_wheel_cmd/10)
		right_wheel_pub.publish(right_wheel_cmd/10)


		rate.sleep()

if __name__ == '__main__':
	main()