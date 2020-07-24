#!/usr/bin/env python

import rospy
import csv
import c25k_msgs
import uuid

import math

from std_msgs.msg import String, Int8, Float32, Time, Int32 
from c25k_msgs.msg import C25KAction, ValidatedC25KAction, C25KState, C25KUserInfo 
from Queue import Queue
from user_class import User

class State_Analyser():
	def __init__(self):
		rospy.init_node('state_analyser', disable_signals=True)

		self.user = User()
		self.session_length = 0

		self.warmup_complete_flag = False
		
		rospy.Subscriber('user_info', C25KUserInfo, self.on_user_info)

		self.state_pub = rospy.Publisher('state',C25KState, queue_size=1) #TO-DO: proper message type 
		self.state = C25KState()
		self.learning_state = []

		#task state
		self.task_success = 1
		self.programme_state = "WALK"
		rospy.Subscriber('programme_state', String, self.on_programme_state)

		#task timing + other timing data
		self.session_time = 0
		self.programme_time = int(self.user.programme_time)
		self.programme_length = 38970.0#TO-DO: make sure this number is correct - total of all session lengths!
		rospy.Subscriber('session_time', Float32, self.on_session_time)
		rospy.Subscriber('programme_time', Float32, self.on_programme_time)

		self.time_spent_prog_action = 0.0
		rospy.Subscriber('time_spent_prog_action', Float32, self.on_time_spent_prog_action)

		self.time_remaining_prog_action = 0.0
		rospy.Subscriber('time_remaining_prog_action', Float32, self.on_time_remaining_prog_action)

		self.programme_action_duration = 0 
		rospy.Subscriber('programme_action_duration', Int32, self.on_programme_action_duration)

		self.programme_action_progress = 0.0
		rospy.Subscriber('programme_action_progress', Float32, self.on_programme_action_progress)

		self.current_speed = 0.0 # speed
		self.relative_speed_average = 1.0 #speed/average
		self.relative_speed_best = 0.5 #speed/best 
		rospy.Subscriber('speed', Float32, self.on_treadmill_data) 
		rospy.Subscriber('fake_speed', Float32, self.on_treadmill_data)
		self.speed_pub = rospy.Publisher('speed_str', String, queue_size=1) 

		self.time_of_last_action = 0.0
		rospy.Subscriber('robot_busy', String, self.on_did_an_action)

		self.heart_rate = []
		self.time_since_heart_rate = 0
		rospy.Subscriber('heartrate', Int32, self.on_heart_rate) #this is the *actual* heart rate sensor
		#rospy.Subscriber('heart_rate', Int8, self.on_heart_rate) #false data stream (heart_rate.py) for testing
		self.hr_pub = rospy.Publisher('hr_str', String, queue_size=1)

		self.session_mood = 1
		rospy.Subscriber('session_mood', String, self.on_session_mood)

		self.stateDuration_pub = rospy.Publisher('stateDuration_str', String, queue_size=1)
		self.sessionDuration_pub = rospy.Publisher('sessionDuration_str', String, queue_size=1)

		rospy.Subscriber('au_12', String, self.on_au12)
		rospy.Subscriber('au_25', String, self.on_au25)

		rospy.Subscriber('session_condition', String, self.on_session_condition)


		# #static data that only needs to be set once
		# self.user_id = ""
		# self.state.self_attitude = self.user.self_attitude
		# self.state.expert_attitude = self.user.expert_attitude
		# self.state.extraversion = self.user.extraversion
		# self.state.agreeableness = self.user.agreeableness
		# self.state.conscientiousness = self.user.conscientiousness
		# self.state.emotional_stability = self.user.emotional_stability
		# self.state.openness_experience  = self.user.openness_experience 
		# #self.state.personality = self.user.personality
		# self.state.activity_level = self.user.activity_level
		
		
		

		rospy.Subscriber('session_complete', String, self.on_session_complete)

		self.session_complete_flag = 0

		self.walk_speed_list = []
		self.run_speed_list = []

		rate = rospy.Rate(2)

		while True:
			rate.sleep()

			if not self.warmup_complete_flag:
				continue

			self.state.user_id = self.user_id
			self.state.session_number = self.user.session_number
			self.state.programme_state = self.programme_state
			self.state.task_success = self.task_success
			self.state.session_time = self.session_time
			self.state.session_time_remaining = self.session_length - self.session_time
			session_time_sec = self.state.session_time_remaining % 60
			session_time_mins = (self.state.session_time_remaining - session_time_sec) / 60
			self.sessionDuration_pub.publish(str(session_time_mins)+"m" + " " + ("%.2f" % session_time_sec))
			if self.session_time/self.session_length < 1:
				self.state.normalised_session_time = self.session_time/self.session_length 
			else:
				self.state.normalised_session_time = 1
			self.state.programme_time = self.programme_time 
			if self.programme_time/self.programme_length < 1:
				self.state.normalised_programme_time = self.programme_time/self.programme_length 
			else:
				self.state.normalised_programme_time =1
			self.state.time_spent_prog_action = self.time_spent_prog_action #is this relevant for anything?
			self.state.time_remaining_prog_action = self.time_remaining_prog_action
			self.stateDuration_pub.publish(("%.2f" % self.time_remaining_prog_action))
			self.state.programme_action_progress = self.programme_action_progress
			self.state.programme_action_duration = self.programme_action_duration 
			self.state.current_speed = self.current_speed
			self.state.relative_speed_average = self.relative_speed_average
			if self.state.relative_speed_average > 1.0:
				self.state.relative_speed_average = 1.0
			self.state.relative_speed_best = self.relative_speed_best
			if self.state.relative_speed_best > 1.0:
				self.state.relative_speed_best = 1.0
			self.state.time_since_last_action = rospy.get_time() - self.time_of_last_action
			if self.state.time_since_last_action > 60:
				self.state.normalised_time_since_last_action = 1
			else:
				self.state.normalised_time_since_last_action = self.state.time_since_last_action/60 
			if len(self.heart_rate) != 0 and self.time_since_heart_rate <= 2:
				self.state.heart_rate = (sum(self.heart_rate)/len(self.heart_rate))
			else: 
				self.state.heart_rate = 0 #wOULD LIKE TO MAKE NaN but had some issues regenerating C25KState msg under time pressure 
			#self.hr_pub.publish(str(self.state.heart_rate))
			if len(self.heart_rate) != 0 and self.time_since_heart_rate <= 2:
				#print "len self.heart_rate if triggering"
				if self.state.heart_rate >= (2*self.user_resting_heart):
					# print "heart rate > 2*resting is triggering"
					self.state.relative_heart_rate = 1
				else:
					# print "else in >=2*resting is triggering"
					# print "self.state.heart_rate is: "
					# print self.state.heart_rate
					self.state.relative_heart_rate = float(self.state.heart_rate)/(2*float(self.user_resting_heart))
					# print "relative heart_rate is: "
					# print self.state.relative_heart_rate
			else:
				self.state.relative_heart_rate = float('NaN') 
			self.state.session_mood = self.session_mood
			self.state.user_run_pb = self.user_run_pb
			self.state.user_walk_pb = self.user_walk_pb
			self.state.user_run_avg = self.user_run_avg
			self.state.user_walk_avg = self.user_walk_avg
			#TO-DO: facial expression
			self.state.state_id = str(uuid.uuid4())
			self.state.timestamp = rospy.get_time()		

			if self.session_complete_flag == 0: #only publish & state if not in session complete

				self.state_pub.publish(self.state)

				state_record = str(self.state.timestamp) + ',' + \
				str(self.state.state_id) + ',' + \
				str(self.state.user_id) + ',' + \
				str(self.state.session_condition) + ',' + \
				str(self.state.self_attitude) + ',' + \
				str(self.state.expert_attitude) + ',' + \
				str(self.state.extraversion) + ',' + \
				str(self.state.agreeableness) + ',' + \
				str(self.state.conscientiousness) + ',' + \
				str(self.state.emotional_stability) + ',' + \
				str(self.state.openness_experience)  + ',' + \
				str(self.state.activity_level) + ',' + \
				str(self.state.session_number) + ',' + \
				str(self.state.programme_state) + ',' + \
				str(self.state.task_success) + ',' + \
				str(self.state.session_time) + ',' + \
				str(self.state.session_time_remaining) + ',' + \
				str(self.state.normalised_session_time) + ',' +\
				str(self.state.programme_time) + ',' + \
				str(self.state.normalised_programme_time) + ',' + \
				str(self.state.time_spent_prog_action) + ',' + \
				str(self.state.time_remaining_prog_action) + ',' + \
				str(self.state.programme_action_progress) + ',' + \
				str(self.state.programme_action_duration) + ',' + \
				str(self.state.current_speed) + ',' + \
				str(self.state.relative_speed_average) + ',' + \
				str(self.state.relative_speed_best) + ',' + \
				str(self.state.time_since_last_action) + ',' + \
				str(self.state.normalised_time_since_last_action) + ',' + \
				str(self.state.heart_rate) + ',' + \
				str(self.state.relative_heart_rate) + ',' + \
				str(self.state.au12) + ',' + \
				str(self.state.au25) + ',' + \
				str(self.state.session_mood) + ',' + \
				str(self.state.user_run_pb) + ',' + \
				str(self.state.user_walk_pb) + ',' + \
				str(self.state.user_run_avg) + ',' + \
				str(self.state.user_walk_avg)	

			#data logging
			with open('state_logger.csv', 'a') as logfile:
				logfile.write(state_record + "\n")
				logfile.flush()
			#logfile.close()


	def on_programme_time(self, programme_time):
		self.programme_time = programme_time.data

	def on_session_time(self,session_time):

		# first time published -> warmup is complete, we can start!
		if not self.warmup_complete_flag:
			print "starting state publisher"
			self.warmup_complete_flag = True

		self.session_time = session_time.data

	def on_heart_rate(self, heart_rate):
		self.time_since_heart_rate = 0
		self.heart_rate.append(heart_rate.data)
		if len(self.heart_rate) > 10:
			self.heart_rate = self.heart_rate[-10:]

		if len(self.heart_rate) != 0:
			heart_rate = (sum(self.heart_rate)/len(self.heart_rate))
		else: 
			heart_rate = 0

		self.hr_pub.publish(str(self.state.heart_rate))

	def on_treadmill_data(self, treadmill_data):
		treadmill_data = treadmill_data.data
		self.current_speed = treadmill_data 
		self.speed_pub.publish(str(self.current_speed))
		if self.warmup_complete_flag == True: #otherwise throws errors during warm up - TBC whether want to do something more clever here 

			if self.programme_state == 1: #supposed to be running
				if treadmill_data > self.user.run_speed_pb and self.state.programme_action_progress <= 0.8: #update user personal best run speed
					self.user_run_pb = treadmill_data 

				if treadmill_data > 4.0: #implies user is indeed definitely running (based on >4mph = run across population)
					self.task_success = 1
				else:
					self.task_success = 0
				
				self.relative_speed_average = treadmill_data/(2*self.user.run_speed_avg)
				self.relative_speed_best = treadmill_data/(2*self.user.run_speed_pb)
			
			else:
				if treadmill_data > self.user.walk_speed_pb and self.programme_state == 0.5 and self.state.programme_action_progress <= 0.8: #i.e. walking not warmup
					self.user_walk_pb = treadmill_data 
				
				if treadmill_data > 0:
					self.task_success = 1

				else: 
					self.task_success = 0 #no speed at all! 

				self.relative_speed_average = treadmill_data/self.user.walk_speed_avg if self.user.walk_speed_avg > 0 else 0
				self.relative_speed_best = treadmill_data/self.user.walk_speed_pb if self.user.walk_speed_pb > 0 else 0

	def on_did_an_action(self, signal):
		self.time_of_last_action = rospy.get_time()

	def on_programme_state(self, state):
		if state.data == "WARMUP":
			self.progamme_state = 0
		elif state.data == "WALK":
			self.programme_state = 0.5
		else:
			self.programme_state = 1

	def on_session_mood(self, mood):
		if mood.data == "POSITIVE": #enumerate for input to the learner
			self.session_mood = 1
			#print "setting session mood to positive"
		elif mood.data == "NEUTRAL":
			self.session_mood = 0.5
			#print "setting session mood to neutral"
		else:
			#print "setting session mood to negative"
			self.session_mood = 0

	def on_time_spent_prog_action(self, duration):
		self.time_spent_prog_action = duration.data

	def on_time_remaining_prog_action(self, duration):
		self.time_remaining_prog_action = duration.data

	def on_programme_action_duration(self, duration):
		if duration.data <= 180:
			self.programme_action_duration = 0
		elif duration.data >= 1200:
			self.programme_action_duration = 1
		else:
			self.programme_action_duration = 0.5

	def on_programme_action_progress(self, progress):
		self.programme_action_progress = progress.data

	def on_user_info(self, user_info):
		self.user_id = user_info.user_id
		self.user.set_user_id(user_info.user_id)
		self.user.set_from_csv()
		# print "user resting heart rate is: "
		# print self.user.resting_heart_rate
		self.state.self_attitude = self.user.self_attitude
		self.state.expert_attitude = self.user.expert_attitude
		self.state.extraversion = self.user.extraversion
		self.state.agreeableness = self.user.agreeableness
		self.state.conscientiousness = self.user.conscientiousness
		self.state.emotional_stability = self.user.emotional_stability
		self.state.openness_experience  = self.user.openness_experience
		self.user_resting_heart = self.user.resting_heart_rate
		#self.state.personality = self.user.personality
		self.state.activity_level = self.user.activity_level
		self.user_run_pb = self.user.run_speed_pb
		self.user_walk_pb = self.user.walk_speed_pb
		self.user_run_avg = self.user.run_speed_avg
		self.user_walk_avg = self.user.walk_speed_avg

		with open('programme_database.csv') as csvDataFile:
			programme_database = csv.reader(csvDataFile)
			for session in programme_database:
				if int(session[1]) == self.user.session_number:
					self.session_length = int(session[2])

	def on_au12(self,au12):
		au12=float(au12.data)

		if math.isnan(au12):
			self.state.au12 = 0.10190
			print "faking au12 data"
		else:
			if au12 >= 5:
				self.state.au12 = 1
			else: 
				self.state.au12 = au12/5

	def on_au25(self,au25):
		au25=float(au25.data)

		if math.isnan(au25):
			self.state.au12 = 0.28160
			print "faking au12 data"

		else:
			if au25 >= 5:
				self.state.au25 = 1
			else: 
				self.state.au25 = au25/5

	def on_session_condition(self, condition):
		self.state.session_condition = condition.data

	def on_session_complete(self, string):
		print "state analyser: session complete - shutting down"
		rospy.signal_shutdown("session complete")

if __name__ == '__main__':
	state_analyser = State_Analyser()
	rospy.spin()
