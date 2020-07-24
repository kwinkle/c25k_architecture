#!/usr/bin/env python

import rospy
import time
import sys
from std_msgs.msg import Int8
import random #just for generating initial random data

def heart_rate():
	pub = rospy.Publisher('heart_rate', Int8, queue_size=10) 
	rospy.init_node('heart_rate')
	rate = rospy.Rate(10) 
	while not rospy.is_shutdown():
		heart_rate = random.randint(60,101)
		pub.publish(heart_rate)
		#print heart_rate

if __name__ == '__main__':
	try:
		heart_rate()
	except rospy.ROSInterruptException:
		pass