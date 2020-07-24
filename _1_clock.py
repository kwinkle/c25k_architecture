#!/usr/bin/env python

import rospy
import time
import sys

from std_msgs.msg import Float32, String
from std_srvs.srv import Empty, EmptyResponse

from user_class import User

class Clock(): 
	def __init__(self):
		rospy.init_node('clock', disable_signals=True)

		rospy.Subscriber('user_start', String, self.on_start)

		self.user = User()
		self.running = False
		self.session_start_flag = 0
		self.session_time = 0.0
		self.warmup_time = 0.0
		self.prev_programme_time = 0.0
		self.session_end_flag = 0

		self.warmup_time_pub = rospy.Publisher('warmup_time', Float32, queue_size=1)
		self.session_time_pub = rospy.Publisher('session_time', Float32, queue_size=1) 
		self.programme_time_pub = rospy.Publisher('programme_time', Float32, queue_size=1) 

		#rospy.Subscriber('warmup_complete', String, self.on_warmup_complete)
		rospy.Subscriber('session_complete', String, self.on_session_complete)

		s = rospy.Service('warmup_complete', Empty, self.handle_warmup_complete)

	def on_start(self, userID):
		self.user.set_user_id(userID.data) 
		self.user.set_from_csv()
		self.prev_programme_time = self.user.programme_time
		self.start_time = rospy.get_time() 

		rate = rospy.Rate(1)
		while True:
			rate.sleep()
			if self.session_end_flag == 0:
				if self.session_start_flag == 0:
					self.warmup_time = rospy.get_time() - self.start_time
					print "warmup time is: ", self.warmup_time
					self.warmup_time_pub.publish(self.warmup_time)
					#if int(self.warmup_time) >= 345: #end of warm up + 10 sec for first instruction to trigger
					#	self.session_start_flag = 1
				else: 
					self.session_time = rospy.get_time() - self.session_start_time #remove (actual!) warmup time so that csv timings can go from 0 
					self.programme_time = self.session_time + self.prev_programme_time #note this doesn't include warm up time (important for assessing fraction complete)
					self.session_time_pub.publish(self.session_time)
					self.programme_time_pub.publish(self.programme_time)
					print "session time is: ", self.session_time
					print "programme time is: ", self.programme_time
			else:
				print "session ended"


	def on_session_complete(self,signal):
		print "session complete - shutting down node"
		rospy.signal_shutdown("session complete")
		#self.session_end_flag == 1

	def handle_warmup_complete(self, params):
		print("Warm-up complete! Starting to publish (session + programme) clock")
		self.session_start_flag = 1
		self.session_start_time = rospy.get_time()

		return EmptyResponse()

if __name__ == '__main__':
	clock = Clock()
	rospy.spin()