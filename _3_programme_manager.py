#!/usr/bin/env python

import rospy
import csv
import uuid

from std_msgs.msg import Float32, String, Int32 
from c25k_msgs.msg import C25KAction, C25KUserInfo, ValidatedC25KAction 

#programme manager handles all timing related to the exercise programme, i.e. issuing programme actions at the necessary times (allowing for time on tablet etc
#and monitoring time spent/remaining on those actions. Also identifies end of the session once the last programme action has been completed 

class Programme_Manager():
	def __init__(self):
		rospy.init_node('programme_manager', disable_signals=True)

		self.time_subscriber = rospy.Subscriber('session_time', Float32, self.on_session_time)
		self.just_starting = True # this flag makes it possible to ' special case' the first time message, to have special behaviou at begining of session

		self.action_pub = rospy.Publisher('programme_actions', C25KAction, queue_size = 1)
		self.condition_pub = rospy.Publisher('session_condition', String, queue_size = 1)
		self.action_execution_pub = rospy.Publisher("execute_action", ValidatedC25KAction, queue_size=1)
		self.programme_state_pub = rospy.Publisher('programme_state', String, queue_size=1)
		
		#self.programme_action_time_pub = rospy.Publisher('programme_action_time', Float32, queue_size=1)
		self.programme_action_duration_pub = rospy.Publisher('programme_action_duration', Int32, queue_size=1)
		self.programme_action_progress_pub = rospy.Publisher('programme_action_progress', Float32, queue_size=1)

		self.time_spent_on_programme_action = 0.0
		self.time_spent_prog_action_pub = rospy.Publisher('time_spent_prog_action', Float32, queue_size=1)
		self.time_remaining_on_programme_action = 0.0
		self.time_remaining_prog_action_pub = rospy.Publisher('time_remaining_prog_action', Float32, queue_size=1)
		self.programme_action_progress = 0.0
		self.programme_action_start_time = 0.0
		self.programme_action_duration = 0

		self.session_complete_pub = rospy.Publisher('session_complete', String, queue_size=1)
		self.session_complete_flag = 0

		rospy.Subscriber('user_info', C25KUserInfo, self.on_user_info) 
		
		self.user_id = ""

		rate = rospy.Rate(10)

		while True:
			rate.sleep()

	def on_user_info(self, user_info):
		self.user_id = user_info.user_id
		self.session = user_info.session_number 

		#print self.user_id

		self.actionlist=""
		self.actionlist2=[]
		self.actionlist3=[] #multiple lists are just for conversions & seperations of CSV data

		if self.user_id in ["LB","JF","JW"]:

			with open('programme_database.csv') as csvDataFile2:
				programme_database = csv.reader(csvDataFile2)
				for session in programme_database:
					if int(session[1]) == self.session:
						self.actionlist = session[3] 
						self.condition_pub.publish(session[0])
						#for testing autonomous condition can just publish "AUTONOMOUS" here instead of pulling from csv

		elif self.user_id == "DP":
			with open('programme_database_DP.csv') as csvDataFile2:
				programme_database = csv.reader(csvDataFile2)
				for session in programme_database:
					if int(session[1]) == self.session:
						self.actionlist = session[3] 
						self.condition_pub.publish(session[0])			

		elif self.user_id == "GB":
			with open('programme_database_GB.csv') as csvDataFile2:
				programme_database = csv.reader(csvDataFile2)
				for session in programme_database:
					if int(session[1]) == self.session:
						self.actionlist = session[3] 
						self.condition_pub.publish(session[0])			

		elif self.user_id == "FB":
			with open('programme_database_FB.csv') as csvDataFile2:
				programme_database = csv.reader(csvDataFile2)
				for session in programme_database:
					if int(session[1]) == self.session:
						self.actionlist = session[3] 
						self.condition_pub.publish(session[0])								

		elif self.user_id in ["MR","PT","DB"]:
			with open('programme_database_PT.csv') as csvDataFile2:
				programme_database = csv.reader(csvDataFile2)
				for session in programme_database:
					if int(session[1]) == self.session:
						self.actionlist = session[3] 
						self.condition_pub.publish(session[0])		

		else:
			with open('programme_database_2.csv') as csvDataFile2:
				programme_database = csv.reader(csvDataFile2)
				for session in programme_database:
					if int(session[1]) == self.session:
						self.actionlist = session[3] 
						self.condition_pub.publish(session[0])
						#for testing autonomous condition can just publish "AUTONOMOUS" here instead of pulling from csv

		self.actionlist = self.actionlist.split("*") #split list of instructions
		for string in self.actionlist:
			self.actionlist2.append(string.split("-")) #split instructions into time, action, style & duration
		for string in self.actionlist2:
			self.actionlist3.append([int(string[0]),string[1],string[2],int(string[3])]) #[time,action,style,duration]
		#print self.actionlist3

		action = self.actionlist3[0]
		self.programme_action_duration = float(action[3])

	def on_session_time(self, time):

		# first time? -> call on_warmup_complete
		if self.just_starting:
			self.on_warmup_complete()
			self.just_starting = False
			return

		for action in self.actionlist3:

			#TO-DO(?): Don asked for an automatic, challenging 10 sec reminder towards end of run (I feel it's an unneccessary complication....)

			if action[0] == (round(time.data)+12): #set sensible buffer for publishing programme actions ahead of time to allow for tablet [10] + speech/getting up to speed [2]
				nextaction = C25KAction()		
				nextaction.action.value = action[1]
				nextaction.style.value = action[2]
				nextaction.duration = int(action[3])
				nextaction.action_id = str(uuid.uuid4())
				print "programme_manager publishing next action: " + nextaction.action.value + nextaction.style.value + str(nextaction.duration)
				self.action_pub.publish(nextaction)

			if action[0] == (round(time.data)): #assume behaviour aligns perfectly to programme manager in terms of participant getting up to speed etc. 
				#print "participant should now be in " + action[1]
				self.programme_state_pub.publish(action[1]) #publish action state change
				self.programme_action_start_time = rospy.get_time()
				self.programme_action_duration = float(action[3])
				self.actionlist3.remove(action) #this action has been published already and executed already, therefore can be removed from list for future checking

		#print "prog action duration is: " + str(self.programme_action_duration)
		self.programme_action_duration_pub.publish(self.programme_action_duration)
		self.time_spent_on_programme_action = rospy.get_time() - self.programme_action_start_time
		self.time_spent_prog_action_pub.publish(self.time_spent_on_programme_action)
		self.time_remaining_on_prog_action = self.programme_action_duration - self.time_spent_on_programme_action
		#print "time remaining on that action is: " + str(self.time_remaining_on_prog_action)
		self.time_remaining_prog_action_pub.publish(self.time_remaining_on_prog_action)
		self.programme_action_progress = self.time_remaining_on_prog_action/self.programme_action_duration 
		self.programme_action_progress_pub.publish(self.programme_action_progress) #this is the important, normalised signal for learner 

		if len(self.actionlist3) == 0 and self.time_remaining_on_prog_action <= 0 and self.session_complete_flag == 0: #indicates session is finished (last action completed)
			#print "session is complete"
			self.session_complete_pub.publish("COMPLETE")
			print "programme_manager: session complete - shutting down node"
			rospy.signal_shutdown("session complete")
			#self.session_complete_flag = 1

	def on_warmup_complete(self):
		#execute first action as timer will not trigger for that action (time = 0)
		print "programme manager: triggering first session action: "
		action = self.actionlist3[0]
		nextaction = ValidatedC25KAction()		
		nextaction.action.value = action[1] 
		nextaction.style.value = action[2]
		nextaction.duration = int(action[3])
		nextaction.validation = ValidatedC25KAction.CONTROL #this is just a fix to prevent it going to teacher tablet
		nextaction.action_id = str(uuid.uuid4())
		print nextaction.action.value + nextaction.style.value + str(nextaction.duration)
		self.action_execution_pub.publish(nextaction)
		#print "participant should now be in " + action[1]
		self.programme_state_pub.publish(action[1])
		self.programme_action_start_time = rospy.get_time()
		self.programme_action_duration = int(action[3])
		self.actionlist3.remove(action)


if __name__ == '__main__':
	programme_manager = Programme_Manager()
	rospy.spin()
