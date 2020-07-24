#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32

class RestingHeartCalculator():
	def __init__(self):
		rospy.init_node('heart_calculator')
		self.heart_sub = rospy.Subscriber('heartrate', Int32, self.on_heart_rate) #this is the *actual* heart rate sensor
		self.start_time = rospy.get_time()
		self.heart_rate = []

	def on_heart_rate(self, heart_rate):
			self.heart_rate.append(heart_rate.data)

if __name__ == '__main__':
	calculator = RestingHeartCalculator()
	finished_flag = 0
	while True:
		if rospy.get_time() - calculator.start_time >= 60 and finished_flag == 0:	
			calculator.heart_sub.unregister()
			print "collected heart rate values were: "
			print calculator.heart_rate
			avg_heart_rate = sum(calculator.heart_rate)/len(calculator.heart_rate)
			print "average resting heart rate is: "
			print str(avg_heart_rate)
			finished_flag = 1
		else: 
			if finished_flag == 0:
				print "calculating"