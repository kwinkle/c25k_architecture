#!/usr/bin/env python

import rospy
import time
import sys
from std_msgs.msg import Float32
import random #just for generating initial random data

def speed():
	pub = rospy.Publisher('fake_speed', Float32, queue_size=10) 
	rospy.init_node('fake_speed')
	speed= 4.0
	rate = rospy.Rate(1) 
	time_since_speed_change = rospy.get_time()
	while not rospy.is_shutdown():
		rate.sleep()
		time_since_speed_change = rospy.get_time() - time_since_speed_change
		if time_since_speed_change > 60:
			speed = random.randint(4.0,8.0)
		pub.publish(speed)
		#print heart_rate

if __name__ == '__main__':
	try:
		speed()
	except rospy.ROSInterruptException:
		pass