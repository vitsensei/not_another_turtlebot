#!/usr/bin/env python

import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import JointState

def talker():
	pub = rospy.Publisher('joint_states', JointState, queue_size=10)
	rospy.init_node('joint_state_publisher', anonymous = True)
	rate = rospy.Rate(10) # 10hz
	i = 0
	while not rospy.is_shutdown():
		jointState_msg = JointState()
		jointState_msg.header = Header()
		jointState_msg.header.stamp = rospy.Time.now()
		jointState_msg.name = ['left_wheel_holder_to_left_wheel', 'right_wheel_holder_to_right_wheel']
		jointState_msg.position = [i, i]
		jointState_msg.velocity = [1, 1]
		jointState_msg.effort = []

		i = i + 1

		pub.publish(jointState_msg)

		rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass