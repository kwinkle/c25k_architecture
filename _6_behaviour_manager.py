#!/usr/bin/env python

import rospy
import sys
import c25k_msgs
import time
import random
import csv
import copy

from std_msgs.msg import String, Time, Int8, Float32, Int32
from naoqi import ALProxy
from c25k_msgs.msg import C25KAction, ValidatedC25KAction, C25KActionType, C25KActionStyle, C25KState, C25KUserInfo
from user_class import User

from walk import Walk
from run import Run 
from timeaction2 import TimeAction 
from maintain import Maintain
from challenge import Challenge
from sympathise import Sympathise 
from praise import Praise
from speedup import SpeedUp
from speeddown import SpeedDown 
from checkpre import CheckPRE
from humour import Humour 
from pre_response import Pre_Response


class BehaviourManager():

	def __init__(self, ip, port):
		self.pip = ip
		self.pport = port
		self.connectNaoQi()

		self.last_action = C25KActionType()

		self.current_style = C25KActionStyle()
		self.current_style.value = C25KActionStyle.NEUTRAL 

		#TO-DO: extend dictionary for any additional low level style/mood behaviour settings
		self.style_dictionary = { #low level values (style 'settings') for style/mood update function, currently only eye LED colours
		 	C25KActionStyle.NEUTRAL: "white",
		 	C25KActionStyle.CHALLENGING: "yellow",
		 	C25KActionStyle.POSITIVE: "green",
		 	C25KActionStyle.SYMPATHETIC: "blue"
		}

		rospy.Subscriber('execute_action', ValidatedC25KAction, self.on_execute_action)
		rospy.Subscriber('extra_dialogue', String, self.extra_dialogue)


		self.robot_busy_pub = rospy.Publisher('robot_busy', String, queue_size=1)
		self.robot_free_pub = rospy.Publisher('robot_free', String, queue_size=1)
		self.subtitles_pub = rospy.Publisher('subtitles', String, queue_size=1) 

		rospy.Subscriber('session_mood', String, self.on_mood)

		rospy.Subscriber('state', C25KState, self.on_state)

		rospy.Subscriber('reset', String, self.on_reset)

		self.time_subscriber = rospy.Subscriber('session_time', Float32, self.on_session_time)

		rospy.Subscriber('emergency_stop', String, self.on_emergency_stop)

		rospy.Subscriber('animation', String, self.on_animation)

		rospy.Subscriber('user_info', C25KUserInfo, self.on_user_info) 

		#rospy.Subscriber('session_complete', String, self.on_session_complete)

		#add subscribers to user info here for behaviour logger during warmup 
		#probably suggests should start analysing state even during warmup

		self.state=C25KState()

		self.warmup_complete_flag = 0
		self.e_stop_flag = 0
		self.session_complete_flag = 0

		self.time_last_speech = None
		self.speech_task_id = None

		self.user = User()

		self.start()

	def connectNaoQi(self):
		rospy.loginfo("Connecting to NaoQi at %s:%d", self.pip, self.pport)
		#self.animspeechProxy = ALProxy("ALAnimatedSpeech", self.pip, self.pport)
		self.animspeechProxy = ALProxy("ALTextToSpeech", self.pip, self.pport)
		self.ttsProxy = ALProxy("ALTextToSpeech", self.pip, self.pport)
		self.robotPostureProxy = ALProxy("ALRobotPosture", self.pip, self.pport)
		#TO-DO: set-up extra proxies here for low level behaviours: e.g. eyes...
		self.eyeLedsProxy = ALProxy("ALLeds", self.pip, self.pport)
		self.ttsProxy.setParameter('speed', 75)
		self.basicAwarenessProxy = ALProxy("ALBasicAwareness", self.pip, self.pport)
		self.autonomousLifeProxy = ALProxy("ALAutonomousLife", self.pip, self.pport)
		self.backgroundMovementProxy = ALProxy("ALBackgroundMovement", self.pip, self.pport)
		self.motionProxy = ALProxy("ALMotion", self.pip, self.pport)
		self.animationProxy = ALProxy("ALAnimationPlayer", self.pip, self.pport)

		if self.animspeechProxy is None:
			sys.exit(1)

	def on_execute_action(self, validated_action):
		state = copy.copy(self.state) #to prevent updates midway through updates

		self.robot_busy_pub.publish('BUSY')
		print "behaviour_manager: robot is busy"

		print "behaviour_manager: received validated action: " + validated_action.action.value + validated_action.style.value

		#execute low level style behaviours before executing action speech
		if self.current_style != validated_action.style:
			self.style_update(validated_action.style) 

			#TO-DO: should probably change action.value below to actually be style update as getting duplicates in the logger
			#if/when implementing this change then need to go through logger & rename all previous instances, can use string of
			#"no speech-style update" to find

			action_record =  str(state.user_id) + ',' + \
			str(state.session_number) + ',' + \
			str(state.state_id) + ',' + \
			str(state.timestamp) + ',' + \
			str(validated_action.action_id)  + ',' + \
			str(validated_action.action.value) + ',' + \
			str(validated_action.style.value) + ',' + \
			str(validated_action.duration) + ',' + \
			str(validated_action.validation) + ',' + \
			str("(no speech - style update)")
			
			#data logging
			with open('behaviour_logger.csv', 'a') as logfile:
				logfile.write(action_record + "\n")
				logfile.flush()

		if validated_action.action.value not in [C25KActionType.GET_CLOSER, C25KActionType.BACK_OFF]:
			self.robotPostureProxy.goToPosture("StandInit", 0.5) 
			self.backgroundMovementProxy.setEnabled(True) #reset pose & background movement to reset any back_off / get_closer 

		#execute speech content of action 
		if validated_action.action.value == C25KActionType.WALK: 
			if validated_action.style.value == C25KActionStyle.POSITIVE:
				speech = self.walk.positive(validated_action.duration)
			elif validated_action.style.value == C25KActionStyle.SYMPATHETIC:
				speech = self.walk.sympathetic(validated_action.duration)
			elif validated_action.style.value == C25KActionStyle.CHALLENGING:
				speech = self.walk.challenging(validated_action.duration)
			else: 
				speech = self.walk.neutral(validated_action.duration)
			
			#self.did_an_action_pub.publish("DID-AN-ACTION")

			action_record =  str(state.user_id) + ',' + \
			str(state.session_number) + ',' + \
			str(state.state_id) + ',' + \
			str(state.timestamp) + ',' + \
			str(validated_action.action_id)  + ',' + \
			str(validated_action.action.value) + ',' + \
			str(validated_action.style.value) + ',' + \
			str(validated_action.duration) + ',' + \
			str(validated_action.validation) + ',' + \
			str(speech)
			
			#data logging
			with open('behaviour_logger.csv', 'a') as logfile:
				logfile.write(action_record + "\n")
				logfile.flush()

		if validated_action.action.value == C25KActionType.RUN: 
			if validated_action.style.value == C25KActionStyle.POSITIVE:
				speech = self.run.positive(validated_action.duration)
			elif validated_action.style.value == C25KActionStyle.SYMPATHETIC:
				speech = self.run.sympathetic(validated_action.duration)
			elif validated_action.style.value == C25KActionStyle.CHALLENGING:
				speech = self.run.challenging(validated_action.duration)
			else:
				speech = self.run.neutral(validated_action.duration) 
			
			#self.did_an_action_pub.publish("DID-AN-ACTION")

			# action_record =  str(state.user_id) + ',' + \
			# str(state.session_number) + ',' + \
			# str(state.state_id) + ',' + \
			# str(state.timestamp) + ',' + \
			# str(validated_action.action_id)  + ',' + \
			# str(validated_action.action.value) + ',' + \
			# str(validated_action.style.value) + ',' + \
			# str(validated_action.duration) + ',' + \
			# str(validated_action.validation) + ',' + \
			# str(speech)
			
			#data logging
			# with open('behaviour_logger.csv', 'a') as logfile:
			# 	logfile.write(action_record + "\n")
			# 	logfile.flush()

		#TBC: old timing actions were forcedly specific based on progress through run, and specifically referenced time remaining
		# may want to just check if want something like that as an action...? see timeaction.py

		# if validated_action.action.value == C25KActionType.TIME: 
		# 	if self.state.progamme_action_completion < 0.5: #less than half way through action, state how much done already
		# 		if validated_action.style.value == C25KActionStyle.POSITIVE:
		# 			self.time.positive_past(self.state.time_on_programme_action)
		# 		elif validated_action.style.value == C25KActionStyle.CHALLENGING:
		# 			self.time.challenging_past(self.state.time_on_programme_action)
		# 		elif validated_action.style.value == C25KActionStyle.SYMPATHETIC:
		# 			self.time.sympathetic_past(self.state.time_on_programme_action)
		# 		else: 
		# 			self.time.neutral_past(self.state.time_on_programme_action)

		# 	elif self.state.progamme_action_completion > 0.5: #more than half way through action, state how long left
		# 		if validated_action.style.value == C25KActionStyle.POSITIVE:
		# 			self.time.positive_remaining(self.state.time_on_programme_action)
		# 		elif validated_action.style.value == C25KActionStyle.CHALLENGING:
		# 			self.time.challenging_remaining(self.state.time_on_programme_action)
		# 		elif validated_action.style.value == C25KActionStyle.SYMPATHETIC:
		# 			self.time.sympathetic_remaining(self.state.time_on_programme_action)
		# 		else: 
		# 			self.time.neutral_remaining(self.state.time_on_programme_action)
		# 	else: #exactly half way - say so 
		# 		if validated_action.style.value == C25KActionStyle.POSITIVE:
		# 			self.time.positive_half(self.state.time_on_programme_action)
		# 		elif validated_action.style.value == C25KActionStyle.CHALLENGING:
		# 			self.time.challenging_half(self.state.time_on_programme_action)
		# 		elif validated_action.style.value == C25KActionStyle.SYMPATHETIC:
		# 			self.time.sympathetic_half(self.state.time_on_programme_action)
		# 		else: 
		# 			self.time.neutral_half(self.state.time_on_programme_action)

		if validated_action.action.value == C25KActionType.TIME: 
			if validated_action.style.value == C25KActionStyle.SYMPATHETIC:
				speech = self.time.sympathetic()
				#self.did_an_action_pub.publish("DID-AN-ACTION")
			elif validated_action.style.value == C25KActionStyle.CHALLENGING:
				speech = self.time.challenging()
				#self.did_an_action_pub.publish("DID-AN-ACTION")
			elif validated_action.style.value == C25KActionStyle.POSITIVE:
				speech = self.time.positive()
				#self.did_an_action_pub.publish("DID-AN-ACTION")

			action_record =  str(state.user_id) + ',' + \
			str(state.session_number) + ',' + \
			str(state.state_id) + ',' + \
			str(state.timestamp) + ',' + \
			str(validated_action.action_id)  + ',' + \
			str(validated_action.action.value) + ',' + \
			str(validated_action.style.value) + ',' + \
			str(validated_action.duration) + ',' + \
			str(validated_action.validation) + ',' + \
			str(speech)
			
			#data logging
			with open('behaviour_logger.csv', 'a') as logfile:
				logfile.write(action_record + "\n")
				logfile.flush()

		if validated_action.action.value == C25KActionType.SPEEDUP:
			if validated_action.style.value == C25KActionStyle.SYMPATHETIC:
				speech = self.speedup.sympathetic()
				#self.did_an_action_pub.publish("DID-AN-ACTION")
			elif validated_action.style.value == C25KActionStyle.CHALLENGING:
				speech = self.speedup.challenging()
				#self.did_an_action_pub.publish("DID-AN-ACTION")

			action_record =  str(state.user_id) + ',' + \
			str(state.session_number) + ',' + \
			str(state.state_id) + ',' + \
			str(state.timestamp) + ',' + \
			str(validated_action.action_id)  + ',' + \
			str(validated_action.action.value) + ',' + \
			str(validated_action.style.value) + ',' + \
			str(validated_action.duration) + ',' + \
			str(validated_action.validation) + ',' + \
			str(speech)
			
			#data logging
			with open('behaviour_logger.csv', 'a') as logfile:
				logfile.write(action_record + "\n")
				logfile.flush()

		if validated_action.action.value == C25KActionType.CHALLENGE:
			if validated_action.style.value == C25KActionStyle.SYMPATHETIC:
				speech = self.challenge.sympathetic()
				#self.did_an_action_pub.publish("DID-AN-ACTION")
			elif validated_action.style.value == C25KActionStyle.CHALLENGING:
				speech = self.challenge.challenging()
				#self.did_an_action_pub.publish("DID-AN-ACTION")
			# else:
			# 	self.challenge.neutral()

			action_record =  str(state.user_id) + ',' + \
			str(state.session_number) + ',' + \
			str(state.state_id) + ',' + \
			str(state.timestamp) + ',' + \
			str(validated_action.action_id)  + ',' + \
			str(validated_action.action.value) + ',' + \
			str(validated_action.style.value) + ',' + \
			str(validated_action.duration) + ',' + \
			str(validated_action.validation) + ',' + \
			str(speech)
			
			#data logging
			with open('behaviour_logger.csv', 'a') as logfile:
				logfile.write(action_record + "\n")
				logfile.flush()

		if validated_action.action.value == C25KActionType.PRAISE:
			if validated_action.style.value == C25KActionStyle.SYMPATHETIC:
				speech = self.praise.sympathetic()
				#self.did_an_action_pub.publish("DID-AN-ACTION")
			elif validated_action.style.value == C25KActionStyle.POSITIVE:
				speech = self.praise.positive()
				#self.did_an_action_pub.publish("DID-AN-ACTION")
			# else:
			# 	self.praise.neutral()

			action_record =  str(state.user_id) + ',' + \
			str(state.session_number) + ',' + \
			str(state.state_id) + ',' + \
			str(state.timestamp) + ',' + \
			str(validated_action.action_id)  + ',' + \
			str(validated_action.action.value) + ',' + \
			str(validated_action.style.value) + ',' + \
			str(validated_action.duration) + ',' + \
			str(validated_action.validation) + ',' + \
			str(speech)
			
			#data logging
			with open('behaviour_logger.csv', 'a') as logfile:
				logfile.write(action_record + "\n")
				logfile.flush()

		if validated_action.action.value == C25KActionType.SYMPATHISE:
			speech = self.sympathise.sympathetic()
			#self.did_an_action_pub.publish("DID-AN-ACTION")

			action_record =  str(state.user_id) + ',' + \
			str(state.session_number) + ',' + \
			str(state.state_id) + ',' + \
			str(state.timestamp) + ',' + \
			str(validated_action.action_id)  + ',' + \
			str(validated_action.action.value) + ',' + \
			str(validated_action.style.value) + ',' + \
			str(validated_action.duration) + ',' + \
			str(validated_action.validation) + ',' + \
			str(speech)
			
			#data logging
			with open('behaviour_logger.csv', 'a') as logfile:
				logfile.write(action_record + "\n")
				logfile.flush()

		if validated_action.action.value == C25KActionType.MAINTAIN: 
			speech = self.maintain.positive()
			#self.did_an_action_pub.publish("DID-AN-ACTION")

			action_record =  str(state.user_id) + ',' + \
			str(state.session_number) + ',' + \
			str(state.state_id) + ',' + \
			str(state.timestamp) + ',' + \
			str(validated_action.action_id)  + ',' + \
			str(validated_action.action.value) + ',' + \
			str(validated_action.style.value) + ',' + \
			str(validated_action.duration) + ',' + \
			str(validated_action.validation) + ',' + \
			str(speech)
			
			#data logging
			with open('behaviour_logger.csv', 'a') as logfile:
				logfile.write(action_record + "\n")
				logfile.flush()

		if validated_action.action.value == C25KActionType.SPEEDDOWN:
			speech = self.speeddown.sympathetic()
			#self.did_an_action_pub.publish("DID-AN-ACTION")

			action_record =  str(state.user_id) + ',' + \
			str(state.session_number) + ',' + \
			str(state.state_id) + ',' + \
			str(state.timestamp) + ',' + \
			str(validated_action.action_id)  + ',' + \
			str(validated_action.action.value) + ',' + \
			str(validated_action.style.value) + ',' + \
			str(validated_action.duration) + ',' + \
			str(validated_action.validation) + ',' + \
			str(speech)
			
			#data logging
			with open('behaviour_logger.csv', 'a') as logfile:
				logfile.write(action_record + "\n")
				logfile.flush()

		if validated_action.action.value == C25KActionType.CHECKPRE:
			speech = self.checkpre.sympathetic()
			#self.did_an_action_pub.publish("DID-AN-ACTION")

			action_record =  str(state.user_id) + ',' + \
			str(state.session_number) + ',' + \
			str(state.state_id) + ',' + \
			str(state.timestamp) + ',' + \
			str(validated_action.action_id)  + ',' + \
			str(validated_action.action.value) + ',' + \
			str(validated_action.style.value) + ',' + \
			str(validated_action.duration) + ',' + \
			str(validated_action.validation) + ',' + \
			str(speech)
			
			#data logging
			with open('behaviour_logger.csv', 'a') as logfile:
				logfile.write(action_record + "\n")
				logfile.flush()

		if validated_action.action.value == C25KActionType.HUMOUR:
			speech = self.humour.positive()
			#self.did_an_action_pub.publish("DID-AN-ACTION")

			action_record =  str(state.user_id) + ',' + \
			str(state.session_number) + ',' + \
			str(state.state_id) + ',' + \
			str(state.timestamp) + ',' + \
			str(validated_action.action_id)  + ',' + \
			str(validated_action.action.value) + ',' + \
			str(validated_action.style.value) + ',' + \
			str(validated_action.duration) + ',' + \
			str(validated_action.validation) + ',' + \
			str(speech)
			
			#data logging
			with open('behaviour_logger.csv', 'a') as logfile:
				logfile.write(action_record + "\n")
				logfile.flush()

		if validated_action.action.value == C25KActionType.GET_CLOSER:
			#SET AND CHECK AGAINST A MAXIMUM VALUE HERE
			current_angle = self.motionProxy.getAngles("HipPitch", False)
			#print "current_angle is: " + str(current_angle)
			self.backgroundMovementProxy.setEnabled(False)
			self.motionProxy.changeAngles("HipPitch",-0.1,0.8)
			#self.did_an_action_pub.publish("DID-AN-ACTION")

			action_record =  str(state.user_id) + ',' + \
			str(state.session_number) + ',' + \
			str(state.state_id) + ',' + \
			str(state.timestamp) + ',' + \
			str(validated_action.action_id)  + ',' + \
			str(validated_action.action.value) + ',' + \
			str(validated_action.style.value) + ',' + \
			str(validated_action.duration) + ',' + \
			str(validated_action.validation) + ',' + \
			"(no action - get closer)"
			
			#data logging
			with open('behaviour_logger.csv', 'a') as logfile:
				logfile.write(action_record + "\n")
				logfile.flush()

		if validated_action.action.value == C25KActionType.BACK_OFF:
			#TO-DO: SET AND CHECK AGAINST A MAXIMUM VALUE HERE
			current_angle = self.motionProxy.getAngles("HipPitch", False)
			#print "current_angle is: " + str(current_angle)
			self.backgroundMovementProxy.setEnabled(False)
			self.motionProxy.changeAngles("HipPitch",0.1,0.8)
			#self.did_an_action_pub.publish("DID-AN-ACTION")

			action_record =  str(state.user_id) + ',' + \
			str(state.session_number) + ',' + \
			str(state.state_id) + ',' + \
			str(state.timestamp) + ',' + \
			str(validated_action.action_id)  + ',' + \
			str(validated_action.action.value) + ',' + \
			str(validated_action.style.value) + ',' + \
			str(validated_action.duration) + ',' + \
			str(validated_action.validation) + ',' + \
			"(no action - back off)"
			
			#data logging
			with open('behaviour_logger.csv', 'a') as logfile:
				logfile.write(action_record + "\n")
				logfile.flush()

		if validated_action.action.value == C25KActionType.ANIMATION:
			choice = random.randint(0,1)
			if choice == 0:
				animation = "animations/Stand/Emotions/Positive/Hysterical_1"
			else:
				animation = "animations/Stand/Gestures/Excited_1"
			self.animationProxy.run(animation)
			#self.did_an_action_pub.publish("DID-AN-ACTION")

			action_record =  str(state.user_id) + ',' + \
			str(state.session_number) + ',' + \
			str(state.state_id) + ',' + \
			str(state.timestamp) + ',' + \
			str(validated_action.action_id)  + ',' + \
			str(validated_action.action.value) + ',' + \
			str(validated_action.style.value) + ',' + \
			str(validated_action.duration) + ',' + \
			str(validated_action.validation) + ',' + \
			animation 
			
			#data logging
			with open('behaviour_logger.csv', 'a') as logfile:
				logfile.write(action_record + "\n")
				logfile.flush()

		self.last_action = validated_action.action
		print "behaviour_manager: robot is free" 
		self.robot_free_pub.publish('FREE')


	def style_update(self, style):
		#import pdb;pdb.set_trace()
		self.eyeLedsProxy.fadeRGB("FaceLeds",self.style_dictionary[style.value],0.5)
		#TO-DO: extend for altering any other lower level behaviours e.g. voice pitch, speed...?

	def extra_dialogue(self, dialogue):
		self.robot_busy_pub.publish('BUSY')
		print "behaviour_manager: robot is busy"
		self.subtitles_pub.publish(dialogue.data)
		#self.speech_task_id = self.animspeechProxy.say(dialogue.data)
		self.animspeechProxy.say(dialogue.data)
		#self.did_an_action_pub.publish("DID-AN-ACTION") #this could be worked in to a 'robot busy' checking mechanism

		print "behaviour_manager: robot is free" 
		self.robot_free_pub.publish('FREE')


		dialogue_str = dialogue.data
		dialogue_str = dialogue_str.replace(", ","-")

		action_record =  self.user.user_id + ',' + \
		str(self.user.session_number) + ',' + \
		"(no state-id on warmup)" + ',' + \
		"(no state-timestamp on warmup)" + ',' + \
		"(no action-id)"  + ',' + \
		"(extra dialogue)" + ',' + \
		"(no style)" + ',' + \
		str(0) + ',' + \
		"(no validation)" + ',' + \
		dialogue_str 
		
		#data logging
		with open('behaviour_logger.csv', 'a') as logfile:
			logfile.write(action_record + "\n")
			logfile.flush()

		if "Goodbye" in dialogue.data:
			self.session_complete_flag = 1
			self.on_emergency_stop("signal") #testing

	def on_mood(self, mood):
		if self.warmup_complete_flag == 1:
			if mood.data == "NEUTRAL":
				self.pre_response.neutral()
			elif mood.data == "POSITIVE":
				self.pre_response.positive()
			else:
				self.pre_response.negative()

	def on_session_time(self, signal):
		'''
		Just used to detected when warm up is complete
		'''
		self.warmup_complete_flag = 1
		self.time_subscriber.unregister()

	def on_state(self, state):
		self.state = state

	def start(self):
		#self.autonomousLifeProxy.setState("disabled") #currently have robot set *not* to be alive by default when turned on 
		self.robotPostureProxy.goToPosture("StandInit", 0.8)
		self.backgroundMovementProxy.setEnabled(True) #this is only current source of background behaviour 
		start_style = C25KActionStyle()
		start_style.value = C25KActionStyle.NEUTRAL
		self.style_update(start_style)

	#if want something more complex as background behaviour then could do something like:
	# def move_head(self):
	# 	if self.state.time_since_last_action > 10 and rospy.get_time() - time_since_moved_head > 10:
	# 		print "moving head just because"
	# 		rate.sleep()
	# 		names = "HeadYaw"
	# 		angles = 0.3
	# 		fractionMaxSpeed = 0.1
	# 		self.motionProxy.setAngles(names, angles, fractionMaxSpeed)
	# 		time.sleep(0.5)
	# 		angles = 0.0
	# 		self.motionProxy.setAngles(names, angles, fractionMaxSpeed)
	# 		time.sleep(0.5)
	# 		angles = 0.1
	# 		self.motionProxy.setAngles(names, angles, fractionMaxSpeed)
	# 		time_since_moved_head = 0
	# 	else:
	# 		time_since_moved_head = rospy.get_time()

	def reset_pose(self):
		if self.speech_task_id is not None and self.e_stop_flag == 0:

			if self.animspeechProxy.isRunning(self.speech_task_id):
				self.time_last_speech = rospy.get_time()

			if not self.animspeechProxy.isRunning(self.speech_task_id) and \
			   self.time_last_speech is not None and \
			   rospy.get_time() > self.time_last_speech + 0.5 and \
			   self.last_action.value not in [C25KActionType.GET_CLOSER, C25KActionType.BACK_OFF]:

				self.robotPostureProxy.goToPosture("StandInit", 0.5)
				self.time_last_speech = None

				print "reset pose triggered and reset to stand"

	def on_emergency_stop(self, signal):
		self.e_stop_flag = 1
		self.ttsProxy.stopAll()
		self.robotPostureProxy.goToPosture("Crouch", 0.5)
		self.backgroundMovementProxy.setEnabled(False)
		self.eyeLedsProxy.fadeRGB("FaceLeds",self.style_dictionary["NEUTRAL"],0.5)

	def on_reset(self, signal):
		self.e_stop_flag = 0
		self.start()

	def on_animation(self, animation):
		self.animationProxy.run(animation.data)

	def on_user_info(self, user_info):
		self.user_id = user_info.user_id
		self.user.set_user_id(user_info.user_id)
		self.user.set_from_csv()

		self.maintain = Maintain(self.animspeechProxy, self.user.name, self.user.user_id)
		self.walk = Walk(self.animspeechProxy)
		self.run = Run(self.animspeechProxy)
		self.time = TimeAction(self.animspeechProxy)
		self.challenge = Challenge(self.animspeechProxy, self.user.name, self.user.user_id)
		self.sympathise = Sympathise(self.animspeechProxy, self.user.user_id)
		self.praise = Praise(self.animspeechProxy, self.user.user_id)
		self.speedup = SpeedUp(self.animspeechProxy)
		self.speeddown = SpeedDown(self.animspeechProxy)
		self.checkpre = CheckPRE(self.animspeechProxy, self.user.name, self.user.user_id)
		self.humour = Humour(self.animspeechProxy, self.user_id)
		self.pre_response = Pre_Response(self.animspeechProxy)

	# def on_session_complete(self, string):
	# 	print "behaviour_manager: session complete - shutting down node"
	# 	rospy.signal_shutdown("session complete")


if __name__ == "__main__":

	#ip="164.11.72.125" #TO-DO: fetch this from wherever is appropriate
	ip="pepper.local"
	rospy.init_node("behaviour_manager")
	behaviour_manager = BehaviourManager(ip, port=9559)
	# print "about to call tts: robot busy"
	# behaviour_manager.animspeechProxy.say("I'm busy now saying this sentence just to test out whether this is blocking or not")
	# print "finished the tts call: robot free"
	#rospy.spin()

	rate = rospy.Rate(2)
	while not rospy.is_shutdown():
		rate.sleep()
		behaviour_manager.reset_pose()
		#behaviour_manager.move_head() #if wanted to do something more complex for background movement...
		if behaviour_manager.session_complete_flag == 1:
			behaviour_manager.on_emergency_stop("signal")
			rospy.signal_shutdown("session complete")
