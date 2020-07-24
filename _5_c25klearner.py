#!/usr/bin/env python

import rospy
import uuid
import pandas as pd
import numpy as np
import random
import csv
import sys
import copy
import threading

from std_msgs.msg import String, Float32
from c25k_msgs.msg import C25KAction, C25KActionType, C25KActionStyle, C25KTraining, C25KState, ValidatedC25KAction
from Queue import Queue

from knnlearner import KNNLearner
from mlplearner import MLPLearner

TIME = "TIME"
SOCIAL = "SOCIAL"
TASK = "TASK"
REWARD = "REWARD"
ANIMATION = "ANIMATION"
GET_CLOSER = "GET_CLOSER"
BACK_OFF = "BACK_OFF" 
CHECKPRE = "CHECKPRE"
NONE = "NONE"


class C25KLearner():
	def __init__(self, style_learner, action_learner):

		self.style_learner = style_learner
		self.action_learner = action_learner

		self.style_learner_dict = {
			0: C25KActionStyle.NEUTRAL,
			1: C25KActionStyle.CHALLENGING,
			2: C25KActionStyle.POSITIVE,
			3: C25KActionStyle.SYMPATHETIC,
			4: NONE,
			C25KActionStyle.NEUTRAL: 0,
	        C25KActionStyle.CHALLENGING: 1,
			C25KActionStyle.POSITIVE: 2,
			C25KActionStyle.SYMPATHETIC: 3,
			NONE: 4
		}

		self.action_learner_dict = {
			0: NONE,
			1: TIME,
			2: TASK,
			3: REWARD,
			4: ANIMATION,
			5: GET_CLOSER,
			6: BACK_OFF,
			7: CHECKPRE,
			8: SOCIAL,
			NONE: 0,
			TIME: 1,
			TASK: 2,
			REWARD: 3,
			ANIMATION: 4,
			GET_CLOSER: 5,
			BACK_OFF: 6,
			CHECKPRE: 7,
			SOCIAL: 8
		}

		rospy.Subscriber('session_condition', String, self.on_session_condition)
		self.time_subscriber = rospy.Subscriber('session_time', Float32, self.on_session_time)

		self.state = C25KState()
		self.learner_input = []

		self.current_style = C25KActionStyle()
		self.current_style.value = C25KActionStyle.NEUTRAL #this should probably then be updated from behaviour logger

		#self.last_published_action = C25KActionType()
		self.last_actionclass_seen = ""
		self.last_actionclass_style_seen = ""

		self.suggested_action_pub = rospy.Publisher('suggested_actions', C25KAction, queue_size = 1) 
		self.suggested_style_pub = rospy.Publisher('suggested_style', C25KAction, queue_size = 1)

		rospy.Subscriber('action_learning_examples', C25KTraining, self.on_action_example)
		rospy.Subscriber('style_learning_examples', C25KTraining, self.on_style_example)

		rospy.Subscriber('state', C25KState, self.on_state)

		rospy.Subscriber('session_complete', String, self.on_session_complete)

		self.warmup_complete_flag = 0
		self.last_style_state_seen = ""
		self.last_action_state_seen = ""

		self.lock = threading.Lock()

	def main_loop(self): #maybe want to return to be 2s instead of on every state in prep for MLP' also want to put this in lock
		with self.lock: 
			state = C25KState()
			state = copy.copy(self.state) #prevent updates midway through loop
			learner_input = copy.copy(self.learner_input) #prevent updates midway through loop
			#print learner_input

			if self.session_condition != "CONTROL":
				if state.state_id != self.last_style_state_seen:
					learner_suggested_style = self.style_learner_dict[self.style_learner.predict(learner_input)]
					self.last_style_state_seen = state.state_id
					if learner_suggested_style != NONE:
						self.create_update_style_action(learner_suggested_style, state)
						#self.current_style.value = learner_suggested_style
						# print "style suggested: "
						# print learner_suggested_style
					else:
						print("learner: no style suggestion this time")
						learner_output_record = str(state.timestamp) + ',' + \
						state.state_id + ',' + \
						"(no style suggestion)" + ',' + \
						"(no style suggestion)" + ',' + \
						"(no style suggestion)" + ',' + \
						"False"

						with open('mlp_learner_output_logger.csv', 'a') as logfile:
							logfile.write(learner_output_record + "\n")
							logfile.flush()	

				if state.state_id != self.last_action_state_seen:
					learner_suggested_action = self.action_learner_dict[self.action_learner.predict(learner_input)]
					self.last_action_state_seen = state.state_id
					if learner_suggested_action != NONE:
						self.create_engagement_action(learner_suggested_action, state)
						# print "action suggested: "
						# print learner_suggested_action
					else:
						print("learner: no action suggestion this time")
						learner_output_record = str(state.timestamp) + ',' + \
						state.state_id + ',' + \
						"(no action suggestion)" + ',' + \
						"(no action suggestion)" + ',' + \
						"(no action suggestion)" + ',' + \
						"False"

						with open('mlp_learner_output_logger.csv', 'a') as logfile:
							logfile.write(learner_output_record + "\n")
							logfile.flush()	

	def create_update_style_action(self, learner_suggested_style, state):
		will_publish = False
		suggested_style_action = C25KAction()
		suggested_style_action.action.value = C25KActionType.STYLE_UPDATE
		suggested_style_action.style.value = learner_suggested_style
		duration = 0
		suggested_style_action.trigger_state_id = state.state_id
		suggested_style_action.trigger_timestamp = state.timestamp

		suggested_style_action.action_id = str(uuid.uuid4())
		if learner_suggested_style != self.current_style.value:
			self.suggested_style_pub.publish(suggested_style_action)
			self.current_style.value = learner_suggested_style

		#MOVE TO ACTION MODERATOR publish suggestion only if style is different to current style
		# if learner_suggested_style != self.current_style:
		# 	will_publish = True
			
		# 	#print "publishing style update suggestion"
			
		# 	self.current_style = suggested_style_action.style
		# else:
		# 	print "learner: not publishing style suggestion as style not new"
		# 	suggested_style_action.action_id = "(style update action not published as style not new)"
		# 	will_publish = False

		learner_output_record = str(state.timestamp) + ',' + \
		state.state_id + ',' + \
		suggested_style_action.action_id + ',' + \
		suggested_style_action.style.value + ',' + \
		suggested_style_action.action.value + ',' + \
		str(will_publish)

		with open('mlp_learner_output_logger.csv', 'a') as logfile:
			logfile.write(learner_output_record + "\n")
			logfile.flush()	

	def create_engagement_action(self, learner_suggested_action, state):
		actionclass = learner_suggested_action
		print("learner: suggested actionclass is: ")
		print(actionclass)
		style = self.current_style.value
		will_publish = False

		suggested_action_valid = False
		suggested_action = C25KAction()
		suggested_action.style.value = style
		suggested_action.duration = 0
		suggested_action.trigger_state_id = state.state_id
		suggested_action.trigger_timestamp = state.timestamp

		if actionclass == TIME:
			if style != C25KActionStyle.NEUTRAL:
				suggested_action.action.value = C25KActionType.TIME
				suggested_action_valid = True

		elif actionclass == SOCIAL:
			if style == C25KActionStyle.POSITIVE:
				suggested_action.action.value = C25KActionType.HUMOUR
				suggested_action_valid = True
			elif style == C25KActionStyle.CHALLENGING:
				suggested_action.action.value = C25KActionType.CHALLENGE
				suggested_action_valid = True
			elif style == C25KActionStyle.SYMPATHETIC:
				choice = random.randint(0,1)
				suggested_action_valid = True
				if choice == 0:
					suggested_action.action.value = C25KActionType.SYMPATHISE
				else:
					suggested_action.action.value = C25KActionType.CHALLENGE
		
		elif actionclass == TASK:
			if style == C25KActionStyle.POSITIVE:
				suggested_action.action.value = C25KActionType.MAINTAIN
				suggested_action_valid = True
			elif style == C25KActionStyle.CHALLENGING:
				suggested_action.action.value = C25KActionType.SPEEDUP
				suggested_action_valid = True
			elif style == C25KActionStyle.SYMPATHETIC:
				suggested_action.action.value = C25KActionType.SPEEDDOWN
				suggested_action_valid = True
		
		elif actionclass == REWARD:
			suggested_action.action.value = C25KActionType.PRAISE
			if style == C25KActionStyle.POSITIVE or style == C25KActionStyle.SYMPATHETIC:
				suggested_action_valid = True
		
		elif actionclass == ANIMATION:
			suggested_action.action.value = C25KActionType.ANIMATION
			suggested_action_valid = True

		elif actionclass == GET_CLOSER:
			suggested_action.action.value = C25KActionType.GET_CLOSER
			suggested_action_valid = True

		elif actionclass == BACK_OFF:
			suggested_action.action.value = C25KActionType.BACK_OFF
			suggested_action_valid = True

		elif actionclass == CHECKPRE:
			suggested_action.action.value = C25KActionType.CHECKPRE
			suggested_action_valid = True
			suggested_action.style.value = C25KActionStyle.SYMPATHETIC

		if suggested_action_valid:
			print("learner: suggested style+action is valid")
			suggested_action.action_id = str(uuid.uuid4())
			self.suggested_action_pub.publish(suggested_action)

			learner_output_record = str(state.timestamp) + ',' + \
			state.state_id + ',' + \
			suggested_action.action_id + ',' + \
			suggested_action.style.value + ',' + \
			suggested_action.action.value + ',' + \
			str(will_publish)

			with open('mlp_learner_output_logger.csv', 'a') as logfile:
				logfile.write(learner_output_record + "\n")
				logfile.flush()	


			#MOVE RESTRICTING LOGIC TO ACTION MODERATOR
			# if actionclass == ANIMATION or actionclass == GET_CLOSER or actionclass == BACK_OFF or actionclass == CHECKPRE:
			# 	if actionclass == self.last_actionclass_seen: #style irrelevant for these 
			# 		self.time_since_last_action = rospy.get_time() - self.time_of_last_action
			# 		if self.time_since_last_action >= 10:
			# 			will_publish = True
						
			# 			print "learner: publishing suggested action"
			# 			self.suggested_action_pub.publish(suggested_action)
			# 			self.last_actionclass_seen = actionclass
			# 			#self.last_actionclass_style_seen = style
			# 			self.time_of_last_action = rospy.get_time()
			# 		else:
			# 			will_publish = False
			# 			suggested_action.action_id = "(action not published as same action seen <10s ago)"
			# 			print "learner: not publishing suggestion as same action seen <10s ago"

			# 	else: #if this is a new combo
			# 		will_publish = True
			# 		suggested_action.action_id = str(uuid.uuid4())
			# 		#print "publishing suggested action"
			# 		print "learner: publishing suggested action with different actionclass/style"
			# 		self.suggested_action_pub.publish(suggested_action)
			# 		#self.last_published_action = suggested_action.action
			# 		self.last_actionclass_seen = actionclass
			# 		self.last_actionclass_style_seen = style
			# 		self.time_of_last_action = rospy.get_time()

			# else:	
			# 	if actionclass == self.last_actionclass_seen and style == self.last_actionclass_style_seen: #if prev example/suggestion was this exact combo
			# 		self.time_since_last_action = rospy.get_time() - self.time_of_last_action
			# 		if self.time_since_last_action >= 10:
			# 			will_publish = True
			# 			suggested_action.action_id = str(uuid.uuid4())
			# 			print "learner: publishing suggested action"
			# 			self.suggested_action_pub.publish(suggested_action)
			# 			self.last_actionclass_seen = actionclass
			# 			self.last_actionclass_style_seen = style
			# 			self.time_of_last_action = rospy.get_time()
			# 		else:
			# 			will_publish = False
			# 			suggested_action.action_id = "(action not published as same action seen <10s ago)"
			# 			print "learner: not publishing suggestion as same action seen <10s ago"

			# 	else: #if this is a new combo
			# 		will_publish = True
			# 		suggested_action.action_id = str(uuid.uuid4())
			# 		#print "publishing suggested action"
			# 		print "learner: publishing suggested action with different actionclass/style"
			# 		self.suggested_action_pub.publish(suggested_action)
			# 		#self.last_published_action = suggested_action.action
			# 		self.last_actionclass_seen = actionclass
			# 		self.last_actionclass_style_seen = style
			# 		self.time_of_last_action = rospy.get_time()


	def on_style_example(self, learning_example):
		if learning_example.validated_action.validation == ValidatedC25KAction.UNPROMPTED or learning_example.validated_action.validation == ValidatedC25KAction.VALIDATED:
			reward = 1
		elif learning_example.validated_action.validation == ValidatedC25KAction.PASSIVE_ACCEPTED:
			if learning_example.validated_action.action in [C25KActionType.WALK, C25KActionType.RUN]: #hack fix 
				reward = 1
			else:
				reward = 0
		elif learning_example.validated_action.validation == ValidatedC25KAction.REFUSED:
			reward = -1

		reduced_learner_example_state = []
		reduced_learner_example_state.append(learning_example.state.self_attitude)
		reduced_learner_example_state.append(learning_example.state.expert_attitude)
		reduced_learner_example_state.append(learning_example.state.extraversion)
		reduced_learner_example_state.append(learning_example.state.agreeableness)
		reduced_learner_example_state.append(learning_example.state.conscientiousness)
		reduced_learner_example_state.append(learning_example.state.emotional_stability)
		reduced_learner_example_state.append(learning_example.state.openness_experience)
		reduced_learner_example_state.append(learning_example.state.activity_level)
		reduced_learner_example_state.append(learning_example.state.programme_state)
		reduced_learner_example_state.append(learning_example.state.normalised_session_time)
		reduced_learner_example_state.append(learning_example.state.normalised_programme_time)
		reduced_learner_example_state.append(learning_example.state.programme_action_progress)
		reduced_learner_example_state.append(learning_example.state.programme_action_duration)
		reduced_learner_example_state.append(learning_example.state.relative_speed_average)
		reduced_learner_example_state.append(learning_example.state.relative_speed_best)
		reduced_learner_example_state.append(learning_example.state.normalised_time_since_last_action)
		reduced_learner_example_state.append(learning_example.state.relative_heart_rate)
		# reduced_learner_example_state.append(learning_example.state.au12)
		# reduced_learner_example_state.append(learning_example.state.au25)
		reduced_learner_example_state.append(learning_example.state.session_mood)
		
										
		training_data = reduced_learner_example_state
		label = self.style_learner_dict[learning_example.validated_action.style.value]

		action_record = learning_example.validated_action.action_id  + ',' + \
		str(learning_example.validated_action.action.value)  + ',' + \
		str(learning_example.validated_action.style.value)  + ',' + \
		str(learning_example.validated_action.validation)  + ',' + \
		str(learning_example.state.self_attitude)  + ',' + \
		str(learning_example.state.expert_attitude)  + ',' + \
		str(learning_example.state.extraversion)  + ',' + \
		str(learning_example.state.agreeableness)  + ',' + \
		str(learning_example.state.conscientiousness)  + ',' + \
		str(learning_example.state.emotional_stability)  + ',' + \
		str(learning_example.state.openness_experience)  + ',' + \
		str(learning_example.state.activity_level)  + ',' + \
		str(learning_example.state.programme_state)  + ',' + \
		str(learning_example.state.normalised_session_time)  + ',' + \
		str(learning_example.state.normalised_programme_time)  + ',' + \
		str(learning_example.state.programme_action_progress)  + ',' + \
		str(learning_example.state.programme_action_duration)  + ',' + \
		str(learning_example.state.relative_speed_average)  + ',' + \
		str(learning_example.state.relative_speed_best)  + ',' + \
		str(learning_example.state.normalised_time_since_last_action)  + ',' + \
		str(learning_example.state.relative_heart_rate)  + ',' + \
		str(learning_example.state.au12)  + ',' + \
		str(learning_example.state.au25)  + ',' + \
		str(learning_example.state.session_mood)

		with open('new_style_instance_collection.csv', 'a') as logfile:
			logfile.write(action_record + "\n")
			logfile.flush()

		#added if to prevent learning updates in autonomous & control sessions @ end of study
		# note that at the beginning of the study, we *also* learnt from control
		if self.session_condition == "SUPERVISED":
			self.style_learner.add_new_instance(training_data,label,reward)
		self.last_style_state_seen = learning_example.state.state_id

	def on_action_example(self, learning_example):
		if learning_example.validated_action.validation == ValidatedC25KAction.UNPROMPTED or learning_example.validated_action.validation == ValidatedC25KAction.VALIDATED:
			reward = 1
		elif learning_example.validated_action.validation == ValidatedC25KAction.PASSIVE_ACCEPTED:
			reward = 0
		elif learning_example.validated_action.validation == ValidatedC25KAction.REFUSED:
			reward = -1

		reduced_learner_example_state = []
		reduced_learner_example_state.append(learning_example.state.self_attitude)
		reduced_learner_example_state.append(learning_example.state.expert_attitude)
		reduced_learner_example_state.append(learning_example.state.extraversion)
		reduced_learner_example_state.append(learning_example.state.agreeableness)
		reduced_learner_example_state.append(learning_example.state.conscientiousness)
		reduced_learner_example_state.append(learning_example.state.emotional_stability)
		reduced_learner_example_state.append(learning_example.state.openness_experience)
		reduced_learner_example_state.append(learning_example.state.activity_level)
		reduced_learner_example_state.append(learning_example.state.programme_state)
		reduced_learner_example_state.append(learning_example.state.normalised_session_time)
		reduced_learner_example_state.append(learning_example.state.normalised_programme_time)
		reduced_learner_example_state.append(learning_example.state.programme_action_progress)
		reduced_learner_example_state.append(learning_example.state.programme_action_duration)
		reduced_learner_example_state.append(learning_example.state.relative_speed_average)
		reduced_learner_example_state.append(learning_example.state.relative_speed_best)
		reduced_learner_example_state.append(learning_example.state.normalised_time_since_last_action)
		reduced_learner_example_state.append(learning_example.state.relative_heart_rate)
		# reduced_learner_example_state.append(learning_example.state.au12)
		# reduced_learner_example_state.append(learning_example.state.au25)
		reduced_learner_example_state.append(learning_example.state.session_mood)

		training_data = reduced_learner_example_state
		abstracted_actionclass = self.abstract_action(learning_example.validated_action.action.value)
		label = self.action_learner_dict[abstracted_actionclass]

		print("learner: saw an example actionclass of type: ")
		print(abstracted_actionclass)

		self.time_of_last_action = rospy.get_time()
		self.last_actionclass_seen = abstracted_actionclass
		self.last_actionclass_style_seen = learning_example.validated_action.style.value

		#turn this off on switch to autonomy! 
		if self.session_condition == "SUPERVISED":
			self.action_learner.add_new_instance(training_data,label,reward)
		self.last_action_state_seen = learning_example.state.state_id

		action_record = learning_example.validated_action.action_id  + ',' + \
		str(abstracted_actionclass)  + ',' + \
		str(learning_example.validated_action.validation)  + ',' + \
		str(learning_example.state.self_attitude)  + ',' + \
		str(learning_example.state.expert_attitude)  + ',' + \
		str(learning_example.state.extraversion)  + ',' + \
		str(learning_example.state.agreeableness)  + ',' + \
		str(learning_example.state.conscientiousness)  + ',' + \
		str(learning_example.state.emotional_stability)  + ',' + \
		str(learning_example.state.openness_experience)  + ',' + \
		str(learning_example.state.activity_level)  + ',' + \
		str(learning_example.state.programme_state)  + ',' + \
		str(learning_example.state.normalised_session_time)  + ',' + \
		str(learning_example.state.normalised_programme_time)  + ',' + \
		str(learning_example.state.programme_action_progress)  + ',' + \
		str(learning_example.state.programme_action_duration)  + ',' + \
		str(learning_example.state.relative_speed_average)  + ',' + \
		str(learning_example.state.relative_speed_best)  + ',' + \
		str(learning_example.state.normalised_time_since_last_action)  + ',' + \
		str(learning_example.state.relative_heart_rate)  + ',' + \
		str(learning_example.state.au12)  + ',' + \
		str(learning_example.state.au25)  + ',' + \
		str(learning_example.state.session_mood)


		with open('new_action_instance_collection.csv', 'a') as logfile:
			logfile.write(action_record + "\n")
			logfile.flush()
		
	def abstract_action(self, action):
		abstracted_action = ""

		if action == C25KActionType.MAINTAIN or action == C25KActionType.SPEEDUP or action == C25KActionType.SPEEDDOWN:
			abstracted_action = TASK
		
		elif action == C25KActionType.TIME:
			abstracted_action = TIME
		
		elif action == C25KActionType.CHALLENGE or action == C25KActionType.HUMOUR or action == C25KActionType.SYMPATHISE: 
			abstracted_action = SOCIAL
		
		elif action == C25KActionType.PRAISE:
			abstracted_action = REWARD
		
		elif action == C25KActionType.ANIMATION:
			abstracted_action = ANIMATION
		
		elif action == C25KActionType.GET_CLOSER:
			abstracted_action = GET_CLOSER
		
		elif action == C25KActionType.BACK_OFF:
			abstracted_action = BACK_OFF
		
		elif action == C25KActionType.CHECKPRE:
			abstracted_action = CHECKPRE
		
		if abstracted_action == "":
			print("failed to abstract action!")

		return abstracted_action

	def on_state(self, state):
		#want to put this in lock to prevent weird overwriting
		with self.lock:
			self.state = copy.copy(state) 
			self.learner_input = [] #feature selection of state features used by learner to predict action
			self.learner_input.append(state.self_attitude)
			self.learner_input.append(state.expert_attitude)
			self.learner_input.append(state.extraversion)
			self.learner_input.append(state.agreeableness)
			self.learner_input.append(state.conscientiousness)
			self.learner_input.append(state.emotional_stability)
			self.learner_input.append(state.openness_experience)
			self.learner_input.append(state.activity_level)
			self.learner_input.append(state.programme_state)
			self.learner_input.append(state.normalised_session_time)
			self.learner_input.append(state.normalised_programme_time)
			self.learner_input.append(state.programme_action_progress)
			self.learner_input.append(state.programme_action_duration)
			self.learner_input.append(state.relative_speed_average)
			self.learner_input.append(state.relative_speed_best)
			self.learner_input.append(state.normalised_time_since_last_action)
			self.learner_input.append(state.relative_heart_rate)
			# self.learner_input.append(state.au12)
			# self.learner_input.append(state.au25)
			self.learner_input.append(state.session_mood)

		self.main_loop()

	def on_session_condition(self, condition):
		self.session_condition = condition.data		

	def on_session_time(self, signal):
		'''
		Just used to detected when warm up is complete
		'''
		self.warmup_complete_flag = 1
		self.time_subscriber.unregister()

	def on_session_complete(self, string):
		if self.session_condition == "SUPERVISED":
			#self.style_learner.retrain()
			#self.action_learner.retrain()
			self.style_learner.save_model()
			self.action_learner.save_model()	

		print("learner: session complete - shutting down node")

if __name__ == '__main__':
	rospy.init_node("learner", disable_signals=True)
	# style_learner = MLPLearner("STYLE", (40,20,20), 0.5)
	# action_learner = MLPLearner("ACTION", (40,20,20), 0.5)
	style_learner = KNNLearner("STYLE")
	action_learner = KNNLearner("ACTION")
	learner = C25KLearner(style_learner, action_learner)
	learner.style_learner.load_model()
	learner.action_learner.load_model()
	rospy.spin()
	# import pdb
	# pdb.set_trace()
	# rate = rospy.Rate(0.5)
	# while not rospy.is_shutdown():
	# 	rate.sleep()
	# 	if learner.warmup_complete_flag == 1 and len(learner.learner_input) != 0:
	# 		rate.sleep()
	# 		learner.main_loop()	
	# 		