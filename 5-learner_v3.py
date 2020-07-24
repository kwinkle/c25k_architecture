#!/usr/bin/env python

import rospy
import uuid
import pandas as pd
import numpy as np
import random
import csv
import sys
import copy

from std_msgs.msg import String, Float32
from c25k_msgs.msg import C25KAction, C25KActionType, C25KActionStyle, C25KTraining, C25KState, ValidatedC25KAction
from Queue import Queue

class DemonstratedStyle():
	def __init__(self):
		self.style = C25KActionStyle()
		self.state_rewards = [] #list of state reward pairs captured at each instance of that action 

class DemonstratedActionClass():
	def __init__(self):
		self.actionclass = ""
		self.state_rewards = [] #list of state reward pairs captured at each instance of that action 

#constants for action classes
TIME = "TIME"
SOCIAL = "SOCIAL"
TASK = "TASK"
REWARD = "REWARD"
ANIMATION = "ANIMATION"
GET_CLOSER = "GET_CLOSER"
BACK_OFF = "BACK_OFF" 
CHECK_PRE = "CHECK_PRE"

class Learner():

	def __init__(self):

		rospy.Subscriber('session_condition', String, self.on_session_condition)
		self.time_subscriber = rospy.Subscriber('session_time', Float32, self.on_session_time)

		self.warmup_complete_flag = 0
		self.session_condition = ""

		self.state = C25KState()
		self.learner_input = []
		self.actionclasses_seen = []
		self.actionclasses_seen_values = [] 
		self.styles_seen = []
		self.styles_seen_values = []
		self.last_style_learning_state_id = ""
		self.last_action_learning_state_id = ""

		self.latest_style_suggestion = C25KActionStyle()
		self.latest_style_suggestion.value = C25KActionStyle.NEUTRAL

		self.last_suggested_actionclass = ""
		self.last_suggested_actionstyle = ""
		self.last_action_time = 0

		self.action_suggestions = 0

		#TO-DO: threshold needs to be saved/loaded from a store
		#self.threshold = 0.8

		with open('learner_threshold.csv') as csvDataFile:
			threshold_reader = csv.reader(csvDataFile)
			for value in threshold_reader:
				threshold_str = value
				
		threshold_str = str(threshold_str)
		threshold_str = threshold_str.replace("[","")
		threshold_str = threshold_str.replace("'","")
		threshold_str = threshold_str.replace("]","")
		self.threshold = float(threshold_str)

		print "learner threshold recovered from csv is: "
		print self.threshold

		self.threshold_factor_decrease = 10
		self.threshold_factor_increase = 5

		rospy.Subscriber('state', C25KState, self.on_state)

		rospy.Subscriber('action_learning_examples', C25KTraining, self.on_action_example)
		rospy.Subscriber('style_learning_examples', C25KTraining, self.on_style_example)

		self.suggested_action_pub = rospy.Publisher('suggested_actions', C25KAction, queue_size = 1) 
		self.suggested_style_pub = rospy.Publisher('suggested_style', C25KAction, queue_size = 1)

		rospy.Subscriber('session_complete', String, self.on_session_complete)

		number_examples = 0
		with open('style_instance_collection.csv') as csvDataFile:
			style_instances = csv.reader(csvDataFile)
			for example in style_instances:
				if example[0] in self.styles_seen_values:
					example_state_string = example[2:22]
					# print "example_state_string is: "
					# print example_state_string
					example_state_list = []
					for string in example_state_string:
						if string[0] == "[":
							replace_str=string[1:]
							example_state_list.append(float(replace_str))
						elif string[-1] == "]":
							replace_str=string[:-1]
							example_state_list.append(float(replace_str))
						else:
							example_state_list.append(float(string))
					#print example_state_list
					self.styles_seen[self.styles_seen_values.index(example[0])].state_rewards.append([example_state_list,float(example[22])])
					number_examples += 1
				else:
					new_style = DemonstratedStyle()
					new_style.style.value = example[0]
					example_state_string = example[2:22]
					example_state_list = []
					for string in example_state_string:
						if string[0] == "[":
							replace_str=string[1:]
							example_state_list.append(float(replace_str))
						elif string[-1] == "]":
							replace_str=string[:-1]
							example_state_list.append(float(replace_str))
						else:
							example_state_list.append(float(string))
					#print([example_state_list,float(example[22])])
					new_style.state_rewards.append([example_state_list,float(example[22])])
					self.styles_seen.append(new_style)
					self.styles_seen_values.append(example[0])
					number_examples += 1

		print "loaded style_instance_collection, styles_seen_values looks like: "
		print self.styles_seen_values
		print type(self.styles_seen)
		print "number of examples loaded is: " + str(number_examples)
		
		number_examples = 0
		with open('action_instance_collection.csv') as csvDataFile:
			action_instances = csv.reader(csvDataFile)
			for example in action_instances:
				if example[0] in self.actionclasses_seen_values:
					example_state_string = example[2:22]
					example_state_list = []
					for string in example_state_string:
						if string[0] == "[":
							replace_str=string[1:]
							example_state_list.append(float(replace_str))
						elif string[-1] == "]":
							replace_str=string[:-1]
							example_state_list.append(float(replace_str))
						else:
							example_state_list.append(float(string))
					#print example_state_list
					self.actionclasses_seen[self.actionclasses_seen_values.index(example[0])].state_rewards.append([example_state_list,float(example[22])])
					number_examples += 1
				else:
					new_class = DemonstratedActionClass()
					new_class.actionclass = example[0]
					example_state_string = example[2:22]
					example_state_list = []
					for string in example_state_string:
						if string[0] == "[":
							replace_str=string[1:]
							example_state_list.append(float(replace_str))
						elif string[-1] == "]":
							replace_str=string[:-1]
							example_state_list.append(float(replace_str))
						else:
							example_state_list.append(float(string))
					new_class.state_rewards.append([example_state_list,float(example[22])])
					self.actionclasses_seen.append(new_class)
					self.actionclasses_seen_values.append(example[0])
					number_examples += 1

		print "loaded action_instance_collection, actionclasses_seen_values looks like: "
		print self.actionclasses_seen_values
		print "number of examples loaded is: " + str(number_examples)


	def on_session_time(self, signal):
		'''
		Just used to detected when warm up is complete
		'''
		self.warmup_complete_flag = 1
		self.time_subscriber.unregister()

	def on_session_condition(self, condition):
		self.session_condition = condition.data

	def main_loop(self):
			state = C25KState()
			state = copy.copy(self.state) #to prevent updates midway through loop
			learner_input = copy.copy(self.learner_input) #to prevent updates midway through loop

			#don't suggest if in control condition (assuming this node is always launched in order to learn)
			if self.session_condition != "CONTROL":

			#style predictor
				if len(self.styles_seen) > 0 and self.last_style_learning_state_id != self.state.state_id:
					print "----------style predictor----------"
					style_predicted_rewards = []
					for style in self.styles_seen:
						state_similarity_values = []
						state_reward_values = []
						state_feature_list = []
						for state_rewards in style.state_rewards: #for each state previously seen for this action
							index = 0
							state_feature_list = state_rewards[0]
							feature_similarity_values = []
							# print "input state is: "
							# print self.learner_input
							# print "comparing to: "
							# print state_feature_list
							for state_feature in state_feature_list: #compute similarity between state features
								# print "comparing"
								# print state_feature
								# print "and"
								#print self.learner_input[index]
								featre_similarity = (self.learner_input[index] - state_feature)*(self.learner_input[index] - state_feature)
								feature_similarity_values.append(featre_similarity)
								index += 1
							state_similarity = 1.0 - sum(feature_similarity_values)/len(feature_similarity_values)
							# print "state similarity is: " 
							# print state_similarity
							state_similarity_values.append(state_similarity)
							state_reward_values.append(state_rewards[1])					
						# print "similarity of each previously seen state is: "
						# print state_similarity_values
						# print "reward associated with those states was: "
						# print state_reward_values
						#import pdb; pdb.set_trace()
						most_similar_index = state_similarity_values.index(max(state_similarity_values))
						most_similar_state_reward = style.state_rewards[most_similar_index]
						reward_most_similar = most_similar_state_reward[1]
						expected_reward = max(state_similarity_values)*reward_most_similar 
						style_predicted_rewards.append([style.style, expected_reward])
					# print "style_predicted_rewards looks like"
					# print style_predicted_rewards
					rewards = []
					for style_reward_pair in style_predicted_rewards:
						rewards.append(style_reward_pair[1])
					highest_reward = max(rewards)
					highest_reward_index = rewards.index(highest_reward)
					best_pair = style_predicted_rewards[highest_reward_index]
					best_suggested_style = best_pair[0]
					print "best_suggested_style is: "
					print best_suggested_style.value 
					print "with predicted reward: "
					print highest_reward

					#only create & log the style update if it's a new style
					if self.latest_style_suggestion.value != best_suggested_style.value:
						suggested_style_action = C25KAction()
						suggested_style_action.action.value = C25KActionType.STYLE_UPDATE
						suggested_style_action.style = best_suggested_style
						duration = 0
						suggested_style_action.trigger_state_id = state.state_id
						suggested_style_action.trigger_timestamp = state.timestamp
						suggested_style_action.action_id = str(uuid.uuid4())
						will_suggest = "FALSE"

						if highest_reward > self.threshold: 
							print "publishing style suggestion"
							self.suggested_style_pub.publish(suggested_style_action)
							self.latest_style_suggestion.value = suggested_style_action.style.value 
							will_suggest = "TRUE" 

						#data logging - includes current threshold for posthoc comparison to reward
						style_output_record = str(suggested_style_action.trigger_timestamp) + ',' + \
						str(suggested_style_action.trigger_state_id) + ',' + \
						str(suggested_style_action.action.value) + ',' + \
						str(suggested_style_action.style.value) + ',' + \
						str(highest_reward) + ',' + \
						str(self.threshold) + ',' + \
						will_suggest + ',' + \
						str(suggested_style_action.action_id)

						with open('learner_output_logger.csv', 'a') as logfile:
							logfile.write(style_output_record + "\n")
							logfile.flush()

						print "------------------------------------------------------"

				#action class + instance predictor
				if len(self.actionclasses_seen) > 0  and self.last_action_learning_state_id != self.state.state_id:
					print "----------action class + instance predictor----------"
					actionclass_predicted_rewards = []
					for actionclass in self.actionclasses_seen:
						# print "for action class: " + actionclass.actionclass 
						state_similarity_values = []
						state_reward_values = []
						state_feature_list = []
						for state_rewards in actionclass.state_rewards: #for each state/reward oair previously seen for this action
							index = 0
							state_feature_list = state_rewards[0] #state
							feature_similarity_values = []
							# print "input state is: "
							# print self.learner_input
							# print "comparing to: "
							#print state_feature_list
							for state_feature in state_feature_list: #compute similarity between state features
								featre_similarity = (self.learner_input[index] - state_feature)*(self.learner_input[index] - state_feature)
								feature_similarity_values.append(featre_similarity)
								index += 1
							state_similarity = 1.0 - sum(feature_similarity_values)/len(feature_similarity_values)
							# print "state similarity is: " 
							# print state_similarity
							state_similarity_values.append(state_similarity)
							state_reward_values.append(state_rewards[1])	
						# print "similarity of each previously seen state is: "
						# print state_similarity_values
						# print "reward associated with those states was: "
						# print state_reward_values
						most_similar_index = state_similarity_values.index(max(state_similarity_values))
						most_similar_state_reward = actionclass.state_rewards[most_similar_index]
						reward_most_similar = most_similar_state_reward[1]
						expected_reward = max(state_similarity_values)*reward_most_similar 
						actionclass_predicted_rewards.append([actionclass.actionclass, expected_reward])
					rewards = []
					for actionclass_reward_pair in actionclass_predicted_rewards:
						rewards.append(actionclass_reward_pair[1])
					highest_reward = max(rewards)
					highest_reward_index = rewards.index(highest_reward)
					best_pair = actionclass_predicted_rewards[highest_reward_index]
					best_suggested_actionclass = best_pair[0]
					print "best_suggested_actionclass is: "
					print best_suggested_actionclass 
					print "with predicted reward: "
					print highest_reward
					print "will apply the style of: "
					print self.latest_style_suggestion.value 

					suggested_action_valid_flag = 0
					suggested_action = C25KAction()
					suggested_action.style = self.latest_style_suggestion
					suggested_action.duration = 0
					suggested_action.trigger_state_id = state.state_id
					suggested_action.trigger_timestamp = state.timestamp
					suggested_action.action_id = str(uuid.uuid4())
					will_suggest = "FALSE"

					# print "self.latest_style_suggestion is: "
					# print self.latest_style_suggestion

					random_choice = random.randint(0,1)

					if best_suggested_actionclass == TIME:
						if self.latest_style_suggestion.value == C25KActionStyle.CHALLENGING:
							suggested_action.action.value = C25KActionType.TIME
							suggested_action_valid_flag = 1
						elif self.latest_style_suggestion.value == C25KActionStyle.SYMPATHETIC:
							suggested_action.action.value = C25KActionType.TIME
							suggested_action_valid_flag = 1
						elif self.latest_style_suggestion.value == C25KActionStyle.POSITIVE:
							suggested_action.action.value = C25KActionType.TIME
							suggested_action_valid_flag = 1
						else:
							print "mismatch between suggested action and suggested style: suggesting nothing"
					
					elif best_suggested_actionclass == SOCIAL:
						if self.latest_style_suggestion.value == C25KActionStyle.POSITIVE:
							suggested_action.action.value = C25KActionType.HUMOUR
							suggested_action_valid_flag = 1
						elif self.latest_style_suggestion.value == C25KActionStyle.CHALLENGING:
							suggested_action.action.value = C25KActionType.CHALLENGE
							suggested_action_valid_flag = 1
						elif self.latest_style_suggestion.value == C25KActionStyle.SYMPATHETIC:
							if random_choice == 0:
								suggested_action.action.value = C25KActionType.SYMPATHISE
							else:
								suggested_action.action.value = C25KActionType.CHALLENGE
							suggested_action_valid_flag = 1
						else:
							print "mismatch between suggested action and suggested style: suggesting nothing"
					
					elif best_suggested_actionclass == REWARD:
						suggested_action.action.value = C25KActionType.PRAISE
						if self.latest_style_suggestion.value == C25KActionStyle.POSITIVE:
							suggested_action_valid_flag = 1
						elif self.latest_style_suggestion.value == C25KActionStyle.SYMPATHETIC:
							suggested_action_valid_flag = 1
						else: 
							print "mismatch between suggested action and suggested style: suggesting nothing"
					
					elif best_suggested_actionclass == TASK:
						if self.latest_style_suggestion.value == C25KActionStyle.POSITIVE:
							suggested_action.action.value = C25KActionType.MAINTAIN
							suggested_action_valid_flag = 1
						elif self.latest_style_suggestion.value == C25KActionStyle.CHALLENGING:
							suggested_action.action.value = C25KActionType.SPEEDUP
							suggested_action_valid_flag = 1
						elif self.latest_style_suggestion.value == C25KActionStyle.SYMPATHETIC:
							suggested_action.action.value = C25KActionType.SPEEDDOWN
							suggested_action_valid_flag = 1
					
					elif best_suggested_actionclass == ANIMATION:
						suggested_action.action.value = C25KActionType.ANIMATION
						suggested_action_valid_flag = 1
					
					elif best_suggested_actionclass == GET_CLOSER:
						suggested_action.action.value = C25KActionType.GET_CLOSER
						suggested_action_valid_flag = 1
					
					elif best_suggested_actionclass == BACK_OFF:
						suggested_action.action.value = C25KActionType.BACK_OFF
						suggested_action_valid_flag = 1

					elif best_suggested_actionclass == CHECK_PRE:
						suggested_action.action.value = C25KActionType.CHECKPRE
						suggested_action_valid_flag = 1

					else:
						print "suggested action class not recognised"

					print "I would suggest a: "
					print self.latest_style_suggestion.value + " " + suggested_action.action.value

					if highest_reward > self.threshold and suggested_action_valid_flag == 1:
						if best_suggested_actionclass == self.last_suggested_actionclass and suggested_action.style.value == self.last_suggested_actionstyle:
							print "I'm suggesting the same thing again!"
							if rospy.get_time() - self.last_action_time > 10:
								print "but it has been a while - will suggest"
								print suggested_action.action.value + suggested_action.style.value 
								self.last_suggested_actionclass = best_suggested_actionclass
								self.last_suggested_actionstyle = suggested_action.style.value 
								self.last_action_time = rospy.get_time()
								will_suggest = "TRUE" 
								if self.action_suggestions == 1:
									print "publishing action suggestion: "
									print suggested_action.action.value + suggested_action.style.value 
									self.suggested_action_pub.publish(suggested_action)
								else:
									print "action suggestions are turned off - not publishing"
							else:
								print "it's too soon dammit!"
								will_suggest = "FALSE"
						else:
							self.last_suggested_actionclass = best_suggested_actionclass
							self.last_suggested_actionstyle = suggested_action.style.value 
							will_suggest = "TRUE" 

							if self.action_suggestions == 1:				
								print "publishing action suggestion: "
								print suggested_action.action.value + suggested_action.style.value 
								self.suggested_action_pub.publish(suggested_action)
							else: 
								print "action suggestions are turned off - not publishing"
	
					action_output_record = str(suggested_action.trigger_timestamp) + ',' + \
					str(suggested_action.trigger_state_id) + ',' + \
					str(suggested_action.action.value) + ',' + \
					str(suggested_action.style.value) + ',' + \
					str(highest_reward) + ',' + \
					str(self.threshold) + ',' + \
					will_suggest + ',' + \
					str(suggested_action.action_id)

					#data logging
					with open('learner_output_logger.csv', 'a') as logfile:
						logfile.write(action_output_record + "\n")
						logfile.flush()		

					print "------------------------------------------------------"		

	def on_style_example(self, learning_example):
		print "got a style learning example with validation type: "
		print learning_example.validated_action.validation
		#TBC: reward logic for tablet actions 
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
		reduced_learner_example_state.append(learning_example.state.au12)
		reduced_learner_example_state.append(learning_example.state.au25)
		reduced_learner_example_state.append(learning_example.state.session_mood)

		self.last_style_learning_state_id = learning_example.state.state_id

		if learning_example.validated_action.style.value in self.styles_seen_values:
			for style in self.styles_seen:
				if style.style == learning_example.validated_action.style:
					#similarity between this example state and closest previous instance of this style
					for state_rewards in style.state_rewards: #for each state previously seen for this style
						index = 0
						state_feature_list = state_rewards[0]
						feature_similarity_values = []
						for state_feature in state_feature_list: #compute similarity between state features
							featre_similarity = (reduced_learner_example_state[index] - state_feature)*(reduced_learner_example_state[index] - state_feature)
							feature_similarity_values.append(featre_similarity)
							index += 1
							state_similarity = 1.0 - sum(feature_similarity_values)/len(feature_similarity_values)

					if state_similarity < self.threshold:
						if reward > 0:
							self.threshold = self.threshold - (self.threshold-state_similarity)/self.threshold_factor_decrease
						elif reward <0:
							self.threshold = self.threshold + (similarity-self.threshold)/self.threshold_factor_increase

						print "updated learner threshold, now at: " + str(self.threshold)	

					#append to instance collection
					style.state_rewards.append([reduced_learner_example_state, reward])

		else:
			#print "it's a new example"
			new_style = DemonstratedStyle()
			new_style.style = learning_example.validated_action.style
			new_style.state_rewards.append([reduced_learner_example_state,reward])
			self.styles_seen.append(new_style)	
			self.styles_seen_values.append(learning_example.validated_action.style.value)

		action_record = learning_example.validated_action.action_id  + ',' + \
		learning_example.validated_action.style.value  + ',' + \
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


		with open('testing_style_instances.csv', 'a') as logfile:
			logfile.write(action_record + "\n")
			logfile.flush()

	def on_action_example(self, learning_example):
		print "got a learning example: "
		print learning_example.validated_action.action.value
		print "with validation type: "
		print learning_example.validated_action.validation
		#TBC: reward logic for tablet actions 
		if learning_example.validated_action.validation == ValidatedC25KAction.UNPROMPTED or learning_example.validated_action.validation == ValidatedC25KAction.VALIDATED:
			reward = 1
		elif learning_example.validated_action.validation == ValidatedC25KAction.PASSIVE_ACCEPTED:
			reward = 0
		elif learning_example.validated_action.validation == ValidatedC25KAction.REFUSED:
			reward = -1

		#DE-BUGGING - reward is sometimes not getting triggered?

		#style = learning_example.validated_action.style.value 
		reduced_learner_example_state = []
		reduced_learner_example_state.append(learning_example.state.programme_state)
		reduced_learner_example_state.append(learning_example.state.normalised_session_time)
		reduced_learner_example_state.append(learning_example.state.normalised_programme_time)
		reduced_learner_example_state.append(learning_example.state.programme_action_progress)
		reduced_learner_example_state.append(learning_example.state.programme_action_duration)
		reduced_learner_example_state.append(learning_example.state.relative_speed_average)
		reduced_learner_example_state.append(learning_example.state.relative_speed_best)
		reduced_learner_example_state.append(learning_example.state.normalised_time_since_last_action)
		reduced_learner_example_state.append(learning_example.state.relative_heart_rate)
		reduced_learner_example_state.append(learning_example.state.session_mood)
		reduced_learner_example_state.append(learning_example.state.au12)
		reduced_learner_example_state.append(learning_example.state.au25)
		reduced_learner_example_state.append(learning_example.state.self_attitude)
		reduced_learner_example_state.append(learning_example.state.expert_attitude)
		reduced_learner_example_state.append(learning_example.state.activity_level)
		reduced_learner_example_state.append(learning_example.state.extraversion)
		reduced_learner_example_state.append(learning_example.state.agreeableness)
		reduced_learner_example_state.append(learning_example.state.conscientiousness)
		reduced_learner_example_state.append(learning_example.state.emotional_stability)
		reduced_learner_example_state.append(learning_example.state.openness_experience)	

		self.last_action_learning_state_id = learning_example.state.state_id	

		actionclass = self.abstract_action(learning_example)

		if actionclass in self.actionclasses_seen_values:
			for prev_seen_class in self.actionclasses_seen:
				if prev_seen_class.actionclass == actionclass:
					#similarity between this example state and closest previous instance of this actionclass
					for state_rewards in prev_seen_class.state_rewards: #for each state previously seen for this style
						index = 0
						state_feature_list = state_rewards[0]
						feature_similarity_values = []
						for state_feature in state_feature_list: #compute similarity between state features
							featre_similarity = (reduced_learner_example_state[index] - state_feature)*(reduced_learner_example_state[index] - state_feature)
							feature_similarity_values.append(featre_similarity)
							index += 1
							state_similarity = 1.0 - sum(feature_similarity_values)/len(feature_similarity_values)

					if state_similarity < self.threshold:
						if reward > 0:
							self.threshold = self.threshold - (self.threshold-state_similarity)/self.threshold_factor_decrease
						elif reward <0:
							self.threshold = self.threshold + (similarity-self.threshold)/self.threshold_factor_increase

						print "updated learner threshold, now at: " + str(self.threshold)

					#append to instance collection
					prev_seen_class.state_rewards.append([reduced_learner_example_state, reward])
					print "adding a new instance to a prev. seen class"
		else:
			#print "it's a new example"
			new_class = DemonstratedActionClass()
			new_class.actionclass = actionclass
			new_class.state_rewards.append([reduced_learner_example_state,reward])
			self.actionclasses_seen.append(new_class)	
			self.actionclasses_seen_values.append(actionclass)

		action_record = learning_example.validated_action.action_id  + ',' + \
		actionclass  + ',' + \
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

		with open('testing_action_instances.csv', 'a') as logfile:
			logfile.write(action_record + "\n")
			logfile.flush()


	def abstract_action(self, learning_example):
		abstracted_action = ""
		print "validated action value is: " + learning_example.validated_action.action.value
		if learning_example.validated_action.action.value == C25KActionType.MAINTAIN or learning_example.validated_action.action.value == C25KActionType.SPEEDUP or \
		learning_example.validated_action.action.value == C25KActionType.SPEEDDOWN:
			abstracted_action = TASK
		elif learning_example.validated_action.action.value == C25KActionType.TIME:
			abstracted_action = TIME
		elif learning_example.validated_action.action.value == C25KActionType.CHALLENGE or C25KActionType.HUMOUR or learning_example.validated_action.action.value == C25KActionType.SYMPATHISE: 
			abstracted_action = SOCIAL
		elif learning_example.validated_action.action.value == C25KActionType.PRAISE:
			abstracted_action = REWARD
		elif learning_example.validated_action.action.value == C25KActionType.ANIMATION:
			abstracted_action = ANIMATION
		elif learning_example.validated_action.action.value == C25KActionType.GET_CLOSER:
			abstracted_action = GET_CLOSER
		elif learning_example.validated_action.action.value == C25KActionType.BACK_OFF:
			abstracted_action = BACK_OFF
		elif learning_example.validated_action.action.value == C25KActionType.CHECKPRE:
			abstracted_action = CHECK_PRE
		if abstracted_action == "":
			print "failed to abstract action!"

		return abstracted_action


	def on_state(self, state):
		self.state = state 
		self.learner_input = [] #feature selection of state features used by learner to predict action
		self.learner_input.append(state.programme_state)
		self.learner_input.append(state.normalised_session_time)
		self.learner_input.append(state.normalised_programme_time)
		self.learner_input.append(state.programme_action_progress)
		self.learner_input.append(state.programme_action_duration)
		self.learner_input.append(state.relative_speed_average)
		self.learner_input.append(state.relative_speed_best)
		self.learner_input.append(state.normalised_time_since_last_action)
		self.learner_input.append(state.relative_heart_rate)
		self.learner_input.append(state.session_mood)
		self.learner_input.append(state.au12)
		self.learner_input.append(state.au25)
		self.learner_input.append(state.self_attitude)
		self.learner_input.append(state.expert_attitude)
		self.learner_input.append(state.activity_level)
		self.learner_input.append(state.extraversion)
		self.learner_input.append(state.agreeableness)
		self.learner_input.append(state.conscientiousness)
		self.learner_input.append(state.emotional_stability)
		self.learner_input.append(state.openness_experience)
		# #TO-DO: float encoded motivator

	def on_session_complete(self, string):
		with open('learner_threshold.csv', 'w') as logfile:
				logfile.write(str(self.threshold))
				logfile.flush()		

		print "session complete - shutting down node"
		rospy.signal_shutdown("session complete")


if __name__ == '__main__':
	rospy.init_node("learner", disable_signals=True)
	learner = Learner()
	rate = rospy.Rate(0.5)
	while not rospy.is_shutdown():
		rate.sleep()
		if learner.warmup_complete_flag == 1:
			rate.sleep()
			learner.main_loop()	