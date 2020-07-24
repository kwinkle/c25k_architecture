#!/usr/bin/env python

import rospy
import c25k_msgs
import time
import uuid
import copy

from std_msgs.msg import Int32, String, Float32
from c25k_msgs.msg import C25KAction, ValidatedC25KAction, C25KActionType, C25KActionStyle, C25KUserInfo, C25KState, C25KTraining
from Queue import PriorityQueue

SUPERVISED="SUPERVISED"
CONTROL="CONTROL"
AUTONOMOUS="AUTONOMOUS"

TIME = "TIME"
SOCIAL = "SOCIAL"
TASK = "TASK"
REWARD = "REWARD"
ANIMATION = "ANIMATION"
GET_CLOSER = "GET_CLOSER"
BACK_OFF = "BACK_OFF" 
CHECKPRE = "CHECKPRE"
NONE = "NONE"

class Action_Moderator():
	def __init__(self):
		rospy.init_node('action_moderator', disable_signals=True)

		rospy.Subscriber('session_condition', String, self.on_session_condition)
		rospy.Subscriber('programme_actions', C25KAction, self.on_programme_action) #session-fixed exercise instructions from database
		rospy.Subscriber('suggested_actions', C25KAction, self.on_suggested_action) #learner suggested actions 
		rospy.Subscriber('validated_actions', ValidatedC25KAction, self.on_validated_action) #actions coming from tablet or control script 
		rospy.Subscriber('suggested_style', C25KAction, self.on_suggested_style) #learner suggested style/mood
		rospy.Subscriber('state', C25KState, self.on_state)
		rospy.Subscriber('robot_busy', String, self.on_robot_busy)
		rospy.Subscriber('robot_free', String, self.on_robot_free)
		rospy.Subscriber('session_complete', String, self.on_session_complete)

		self.is_running = False
		
		self.state = C25KState()
		
		self.suggested_style = C25KActionStyle()
		self.suggested_style.value = C25KActionStyle.NEUTRAL

		self.time_subscriber = rospy.Subscriber('session_time', Float32, self.on_session_time)

		self.proposed_action_pub = rospy.Publisher("unvalidated_actions", C25KAction, queue_size=1) #to tablet for validation
		self.action_execution_pub = rospy.Publisher("execute_action", ValidatedC25KAction, queue_size=1) #to behaviour manager - may want to replace with an action client for response?
		#self.learning_actions_pub = rospy.Publisher('learning_actions', ValidatedC25KAction, queue_size=1) #go to feedback moderator
		self.action_learner_examples_pub =  rospy.Publisher('action_learning_examples', C25KTraining, queue_size=1) #go direct to learner
		self.style_learner_examples_pub =  rospy.Publisher('style_learning_examples', C25KTraining, queue_size=1) #go direct to learner

		# self.unvalidated_action_queue = PriorityQueue() 
		# self.validated_action_queue = PriorityQueue() 
		# self.control_action_queue = PriorityQueue()

		self.queued_action_suggestion = C25KAction()
		self.new_action_suggestion = False
		self.queued_validated_action = ValidatedC25KAction()
		self.new_validated_action = False

		self.robot_busy = False
		self.session_condition = ""
		self.waiting_on_response_flag = 0 

		self.time_of_last_execution = 0
		self.time_since_executed_action = 0

		self.last_executed_action_class = ""
		self.last_executed_action_style = ""

		self.tablet_free_time = rospy.get_time()


		self.run()
		
	def run(self):
		rate = rospy.Rate(50)

		while True:			
			rate.sleep()

			# print "waiting on response flag is: "
			# print self.waiting_on_response_flag

			if not self.is_running:
				continue
			# print "unvalidated queue size is: " + str(self.unvalidated_action_queue.qsize())
			# print "validated queue size is: " + str(self.validated_action_queue.qsize())
			
			if self.session_condition != CONTROL:
				# print "main action_moderator loop"
				# print "waiting on response flag: " + str(self.waiting_on_response_flag)
				# print "new_action_suggestion: " + str(self.new_action_suggestion)
				
				#if got a new action suggestion and the tablet is free
				if self.new_action_suggestion and self.waiting_on_response_flag == 0: 
					time_since_tablet_free = rospy.get_time() - self.tablet_free_time
					#print "action_moderator: time since tablet free is " + str(time_since_tablet_free)
					if self.queued_action_suggestion.action.value in [C25KActionType.RUN, C25KActionType.WALK] or time_since_tablet_free >= 30:
						#fetch queued action suggestion
						proposed_action = self.queued_action_suggestion
						self.proposed_action_pub.publish(proposed_action) #goes to the tablet for checking
						print "action _moderator publishing this proposed action to tablet: " + proposed_action.action.value + proposed_action.style.value  
						self.waiting_on_response_flag = 1 #prevent further suggestions until this one has been addressed
						self.new_action_suggestion = False

				#if got a new validated action and the robot is not busy
				if self.new_validated_action and not self.robot_busy:
					#fetch the queued validated action
					validated_action = self.queued_validated_action 
					print "action_moderator publishing this validated action to behaviour manager: " + str(validated_action.action.value) + str(validated_action.style.value) + str(validated_action.duration)
					self.action_execution_pub.publish(validated_action) 
					self.new_validated_action = False

			else:
				if self.new_validated_action:
					control_action = self.queued_validated_action
					print "action_moderator publishing this validated control action to behaviour manager: " + str(control_action.action.value) + str(control_action.style.value) + str(control_action.duration)
					self.action_execution_pub.publish(control_action)
					self.new_validated_action = False

	def on_session_time(self, signal):
		'''
		Just used to detected when warm up is complete
		'''
		self.is_running = True
		self.time_subscriber.unregister()

	def on_programme_action(self, action):
		#programme actions need to be given trigger timestamps on receipt here
		#print "got programme action: " + action.action.value
		if self.session_condition != CONTROL:
			action.style.value = self.suggested_style.value #apply latest learner suggested style
			action.trigger_timestamp = rospy.get_time()
			action.trigger_state_id = self.state.state_id
			print "action_moderator: got new suggested programme action - overwriting any queued action suggestion"
			print "style of suggested programme action: "
			print action.style.value
			self.queued_action_suggestion = action #overwrite any stored suggestion with this programme action
			self.new_action_suggestion = True
		
		else:
			prog_action = ValidatedC25KAction() 
			prog_action.action = action.action
			prog_action.style = action.style 
			prog_action.duration = action.duration
			prog_action.validation = ValidatedC25KAction.CONTROL
			prog_action.trigger_timestamp = rospy.get_time()
			prog_action.trigger_state_id = self.state.state_id
			prog_action.action_id = action.action_id
			time.sleep(10) #wait for the same amount of time it would take to put action through tablet 
			print "action_moderator: got new suggested programme action - overwriting any queued action suggestion"
			self.queued_action_suggestion = action #overwrite any stored suggestion with this programme action
			self.new_action_suggestion = True
			self.on_validated_action(prog_action) #replicates learning generation and logging as if come from tablet
			

	def on_suggested_action(self, action):
		#suggested_class = self.abstract_action(action.action.value) #TBC whether this is needed for additional logic around not repeating same suggestion

		if self.session_condition != CONTROL:
			#if queued action is not new then can just overwrite
			if not self.new_action_suggestion:
				self.queued_action_suggestion = action
				self.new_action_suggestion = True
				print "action_moderator: replacing old/non-existent queued action"
			else:
				#only update queued action if queued action is not a programme action 
				if not self.queued_action_suggestion.action.value in [C25KActionType.WALK, C25KActionType.RUN]:
					self.queued_action_suggestion = action
					self.new_action_suggestion = True
					print "action_moderator: got new engagement suggestion action - overwriting stored engagement suggestion"
	
	def on_suggested_style(self, action):
		#print "got suggested style"
		if self.session_condition != CONTROL:
			if self.suggested_style != action.style: 
				self.suggested_style = action.style
				print "action_moderator: suggested_style is " 
				print self.suggested_style
				print self.suggested_style.value
				style_update = ValidatedC25KAction()
				style_update.action.value = C25KActionType.STYLE_UPDATE
				style_update.validation = ValidatedC25KAction.STYLE_UPDATE
				style_update.style = action.style
				style_update.trigger_state_id = action.trigger_state_id
				style_update.trigger_timestamp = action.trigger_timestamp
				style_update.action_id =str(uuid.uuid4())

				#syle updates automatically assumed valid
				self.on_validated_action(style_update) 

	def on_validated_action(self, validated_action):
		state = copy.copy(self.state) #to prevent updates midway through loop

		if validated_action.action.value in [C25KActionType.RUN, C25KActionType.WALK]:
			if validated_action.validation != ValidatedC25KAction.REFUSED: #styled programme suggestions may yield a refused before the final choice
				self.waiting_on_response_flag = 0  
				self.tablet_free_time = rospy.get_time()
				print "action_moderator: received final response to styled programme suggestion"
		
		elif validated_action.action.value != C25KActionType.STYLE_UPDATE and validated_action.validation != ValidatedC25KAction.UNPROMPTED:
			self.waiting_on_response_flag = 0 #this implies response to an engagement suggestion
			self.tablet_free_time = rospy.get_time()
			print "action_moderator: received response to learner engagement action suggestion"

		if validated_action.validation == ValidatedC25KAction.UNPROMPTED: #unprompted actions will not yet have been assigned an ID or trigger stamps
			validated_action.trigger_timestamp = state.timestamp 
			validated_action.trigger_state_id = state.state_id
			validated_action.action_id = str(uuid.uuid4())

		validated_action.validation_timestamp = state.timestamp #all actions are given validation time/state_id regardless of origin/condition
		validated_action.validation_state_id = state.state_id #although really 'validation' is only in the case of learner suggestions

		print "action_moderator: " + str(rospy.get_time())+ ": got validated action " + validated_action.action.value + " (validation timestamp: " + str(state.timestamp) + ")"
		print "with style " + validated_action.style.value
		print "with validation state " + validated_action.validation
		
		if self.session_condition != CONTROL: 
			#if new validated action was not a refused one
			if validated_action.validation != ValidatedC25KAction.REFUSED:
				#if no new validated action currently queued then take whatever comes here
				if not self.new_validated_action:
					self.queued_validated_action = validated_action
					self.new_validated_action = True
				else:
					#if new validated action is a programme action, replace whatever was waiting
					if validated_action.action.value in [C25KActionType.WALK, C25KActionType.RUN]:
						self.queued_validated_action = validated_action
						self.new_validated_action = True
					else: 
						#only want to overwrtite queued action if queued action is not a programme action
						if not self.queued_validated_action.action.value in [C25KActionType.WALK, C25KActionType.RUN]:
							#only overwrite engagement actions with newer engagement actions (not style updates)
							if self.queued_validated_action.action.value != C25KActionType.STYLE_UPDATE:
								if validated_action.action.value != C25KActionType.STYLE_UPDATE:
									self.queued_validated_action = validated_action
									self.new_validated_action = True
							else: #queued validated action is a style update
								self.queued_validated_action = validated_action
								self.new_validated_action = True

			#learn styles from all actions except automatically verified style updates, 'style-free actions', or first forced neutral programme action
			if validated_action.action.value != C25KActionType.STYLE_UPDATE and validated_action.action.value != C25KActionType.GET_CLOSER and \
			validated_action.action.value != C25KActionType.BACK_OFF and validated_action.validation != ValidatedC25KAction.CONTROL:			
				style_learning_example = C25KTraining()
				style_learning_example.validated_action = validated_action
				style_learning_example.state = state
				print "action_moderator: publishing a style learning action"
				self.style_learner_examples_pub.publish(style_learning_example)
			
			if not validated_action.action.value in [C25KActionType.WALK, C25KActionType.RUN, C25KActionType.STYLE_UPDATE]:
					action_learning_example = C25KTraining()
					action_learning_example.validated_action = validated_action
					action_learning_example.state = state
					print "action_moderator: publishing an action learning example"
					self.action_learner_examples_pub.publish(action_learning_example) #generate additional action learning examples

		else:
			if validated_action.validation == ValidatedC25KAction.CONTROL:
				#replicate above logic 
				#if no new validated action currently queued then take whatever comes here
				if self.new_validated_action == False:
					self.queued_validated_action = validated_action
					self.new_validated_action = True
				else:
					#if new validated action is a programme action, replace whatever was waiting
					if validated_action.action.value in [C25KActionType.WALK, C25KActionType.RUN]:
						self.queued_validated_action = validated_action
						self.new_validated_action = True
					else: 
						#if queued action is not a programme action, then rewrite with new validated action
						if not self.queued_validated_action.action.value in [C25KActionType.WALK, C25KActionType.RUN]:
							self.queued_validated_action = validated_action
							self.new_validated_action = True

			elif validated_action.validation == ValidatedC25KAction.UNPROMPTED: #these are supervisor actions still coming from the tablet
				style_learning_example = C25KTraining()
				style_learning_example.validated_action = validated_action
				style_learning_example.state = state
				print "action_moderator: publishing a style learning action"
				self.style_learner_examples_pub.publish(style_learning_example) #assumes all actions generate a style learning example
				action_learning_example = C25KTraining()
				action_learning_example.validated_action = validated_action
				action_learning_example.state = state
				print "action_moderator: publishing an action learning example"
				self.action_learner_examples_pub.publish(action_learning_example) #generate additional action learning example

		#action_record = [user_id, session_number, action_id, time of validation, state id at validation, action, style, duration (if programme acttion) validation status, time of trigger (if learner suggestion),... 
		#...state id at trigger (if learner suggestion)]
		action_record = str(state.user_id) + ',' + \
		str(state.session_number) + ',' + \
		str(validated_action.action_id)  + ',' + \
		str(validated_action.validation_timestamp) + ',' + \
		str(validated_action.validation_state_id) + ',' + \
		str(validated_action.action.value) + ',' + \
		str(validated_action.style.value) + ',' + \
		str(validated_action.duration) + ',' + \
		str(validated_action.validation) + ',' + \
		str(validated_action.trigger_timestamp) + ',' + \
		str(validated_action.trigger_state_id)

		#data logging
		with open('action_logger.csv', 'a') as logfile:
			logfile.write(action_record + "\n")
			logfile.flush()

	def on_session_condition(self, session_condition):
		self.session_condition = session_condition.data
		print "session_condition is: " + self.session_condition

	def on_state(self, state):
		self.state = state

	def on_session_complete(self, string):
		print "session complete - shutting down node"
		rospy.signal_shutdown("session complete")

	
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
			print "failed to abstract action!"

		return abstracted_action

	def on_robot_busy(self, signal):
		self.robot_busy = True

	def on_robot_free(self, signal):
		self.robot_busy = False


if __name__ == '__main__':
	action_moderator = Action_Moderator()
	rospy.spin()