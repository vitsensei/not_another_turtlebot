#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose, Point, Quaternion

def main():
	rospy.init_node("command_publisher")
	pub = rospy.Publisher("turtlebot_command_topic", Pose, queue_size=10)

	while not rospy.is_shutdown(): 
		print("\n\nNew position")
		x = float(input("X: "))
		y = float(input("Y: "))

		new_pose = Pose(position= Point(x,y,0), orientation= Quaternion(0,0,0,0))
		pub.publish(new_pose)

if __name__ == '__main__':
	main()