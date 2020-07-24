#!/usr/bin/env python

import rospy
import uuid
import copy
import random

from std_msgs.msg import Float32, String
from c25k_msgs.msg import C25KState, ValidatedC25KAction, C25KActionType, C25KActionStyle, C25KUserInfo  

class Control_Script():
	def __init__(self):

		rospy.Subscriber('state', C25KState, self.on_state) 
		rospy.Subscriber('user_info', C25KUserInfo, self.on_user_info)
		rospy.Subscriber('session_time', Float32, self.on_time)
		rospy.Subscriber('warmup_complete', String, self.on_start)
		rospy.Subscriber('session_complete', String, self.on_session_complete)

		self.state = C25KState()
		self.last_action = ValidatedC25KAction()

		self.mood = C25KActionStyle
		self.mood = C25KActionStyle.NEUTRAL
		self.heart_effort_threshold = 0
		self.max_heart_rate = 0
		self.session_time = 0.0
		self.time_since_last_action = 0
		self.time_of_last_trigger = 0.0
		self.time_since_last_triggered = 0
		self.session_ended = 0

		self.check_pre_flag = False

		self.action_pub = rospy.Publisher("validated_actions", ValidatedC25KAction, queue_size=1)


	def on_start(self, signal):
		print "starting control script"
		rate = rospy.Rate(1) 

		while True:
			rate.sleep()
			state = copy.copy(self.state) #to prevent updates midway through loop
			if self.session_ended == 0 and self.state.session_time > 25 and self.state.normalised_session_time < 0.98: #hack to avoid control actions clashing with end session script  
				print "control: time since last action = " + str (self.time_since_last_action)

				#TBC as per discussion with Severin - not doing any style changes, just pre-decided control actions
				# if state.relative_speed_average <= 1: # <= average speed
				# 	self.mood = C25KActionStyle.CHALLENGING
				# elif self.state.relative_speed_average >= 1 and state.relative_speed_best >= 1: # >= pb speed
				# 	self.mood = C25KActionStyle.POSITIVE				
				# elif state.relative_speed_average >= 1 and state.relative_speed_best <= 1 and state.heart_rate > self.heart_effort_threshold: #speed ok, effort high
				# 	self.mood = C25KActionStyle.SYMPATHETIC
				# else: #good (but not best) speed and < max effort
				# 	self.mood = C25KActionStyle.NEUTRAL

				#TBC: improvements to control condition - randomise the timing of actions a little bit, more dialogue...

				if self.time_since_last_action >= 30 and self.time_since_last_triggered > 10: #captures delay in action publishing to avoid multiple action generations 
					next_action = ValidatedC25KAction()
					next_action.validation = ValidatedC25KAction.CONTROL
					next_action.duration = 0
					next_action.trigger_timestamp = state.timestamp
					next_action.trigger_state_id = state.state_id
					next_action.action_id = str(uuid.uuid4())
					# print "time on programme action is " + str(state.time_spent_prog_action)

					# if int(state.programme_action_progress*100) == 25 or int(state.programme_action_progress*100) == 50 or int(state.programme_action_progress*100) == 75:    
					# 	print "triggering creation of a timing control action"
					# 	next_action.action.value = C25KActionType.TIME
					# 	#next_action.style.value = self.mood
									
					# else: 

					print "control: triggering creation of an engagement control action"
					print "control: task performance: " + str(state.relative_speed_average) + " " + str(state.relative_speed_best)

					if 0.45 <= state.normalised_session_time <= 0.55 and self.check_pre_flag == False:
						next_action.action.value = C25KActionType.CHECKPRE
						next_action.style.value = C25KActionStyle.SYMPATHETIC
						self.check_pre_flag = True

					elif state.heart_rate >= self.max_heart_rate:
						next_action.action.value = C25KActionType.SPEEDDOWN
						next_action.style.value = C25KActionStyle.SYMPATHETIC

					elif state.relative_speed_average <= 0.5: # <= average speed tim
						next_action.action.value = C25KActionType.CHALLENGE
						r = random.randint(1,10)
						if r >= 6:
							next_action.style.value = C25KActionStyle.CHALLENGING
						else:
							next_action.style.value = C25KActionStyle.SYMPATHETIC
						print "control: triggering a control challenge action"
					
					elif state.relative_speed_average >= 0.5 and state.relative_speed_best >= 1: # >= pb speed			
						print "control: triggering a control praise action"
						next_action.action.value = C25KActionType.PRAISE
						if r >= 6:
							next_action.style.value = C25KActionStyle.POSITIVE
						else:
							next_action.style.value = C25KActionStyle.SYMPATHETIC
					
					elif state.relative_speed_average >= 0.5 and state.relative_speed_best <= 1 and state.heart_rate > self.heart_effort_threshold: #speed ok, effort high
						print "control: triggering a control sympathise action"
						next_action.action.value = C25KActionType.SYMPATHISE
						next_action.style.value = C25KActionStyle.SYMPATHETIC
					
					else: #good (but not best) speed and < max effort
						print "control: triggering a control maintain action"
						next_action.action.value = C25KActionType.MAINTAIN
						next_action.style.value = C25KActionStyle.POSITIVE
							
					self.action_pub.publish(next_action)
					self.time_since_last_triggered = 0
					self.time_of_last_trigger = rospy.get_time()
			else:
				print "control: session complete"

	def on_state(self, state):
		self.state = state
		self.time_since_last_action = state.time_since_last_action

	def on_user_info(self, user_info):
		self.heart_effort_threshold = user_info.heart_effort_threshold
		self.max_heart_rate = int((float(self.heart_effort_threshold)/80.0)*100)

	def on_time(self,time):
		self.time_since_last_triggered = rospy.get_time() - self.time_of_last_trigger

	def on_session_complete(self, string):
		print "control: session complete - shutting down node"
		rospy.signal_shutdown("session complete")


if __name__ == "__main__":
	rospy.init_node('control_script', disable_signals=True)
	control = Control_Script()
	rospy.spin()