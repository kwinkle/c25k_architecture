#!/usr/bin/env python

import pandas as pd
import rospy
import csv
import random
import time
from std_msgs.msg import String, Float32
from c25k_msgs.msg import C25KUserInfo, C25KState
#from naoqi import ALProxy
from user_class import User

class EndSession():
	def __init__(self, disable_signals=True):
		self.user = User()
		rospy.Subscriber('user_info', C25KUserInfo, self.on_user_info)

		# self.user.set_user_id("K1") #just for testing purposes (remeber to remove!)
		# self.user.set_from_csv()

		self.state = C25KState()
		rospy.Subscriber('state', C25KState, self.on_state)

		rospy.Subscriber('session_complete', String, self.on_start)
		self.dialogue_pub = rospy.Publisher('extra_dialogue', String, queue_size=1)

		rospy.Subscriber('emergency_stop', String, self.on_estop)

		self.estop_flag = 0

		self.end_dict = {
		 0: "Ok bring the treadmill to a stop when you're ready. FINISHED!! You did really well today, definitely looks like I'm going to have to test you next time. Until then " + self.user.name + " rest, recover and look after yourself! Goodbye.",
		 1: "Ok " + self.user.name + " that's the end of our session for today. Bring the treadmill to a stop when you're ready. I look forward to seeing you again next time!. Goodbye!",
		 2: "Run Complete!! Great work today "+ self.user.name + " you are finally free! Bring it down to zero when you're ready. I'll be seeing you soon, keep well! Goodbye.",
		 3: "Ok, bring it down to a stop when you're ready. Nice work " + self.user.name + ". We're done for today, but don't forget to cool down properly and have a good stretching session; feel free to talk to my friend Don for some useful stretches. I'll catch you soon, Goodbye.",
		 4: "And we are done!! Catch your breath, cool down and keep up all this great work! I'll see you soon. Goodbye.",
		 5: "Ok, bring it down to a stop when you're ready. Another great session today! Don't forget to have a good stretch out and fuel that body with something worthwhile, you've earnt it. Goodbye.",
		 6: "Done! Bring it down to zero when you're ready. That's all for today's session, I look forward to working you hard again, till next time "+ self.user.name + " Goodbye.", 
		 7: "Ok bring it down to zero when you're ready " + self.user.name + " that's it for today, you're flying through these runs! I hope you're still having as much fun as I am! I look forward to seeing you next time! Rest well. Goodbye.",
		 8: "Run completed! You survived!! Bring it down to zero when you're ready. I knew you could do it, now rest up and stretch out that body; I think you've earnt some quality relaxation time! Till next time " + self.user.name + " Goodbye.",
		 9: "Finished! Bring the treadmill to a stop when you're ready. Nice work today " + self.user.name + ". Hope you had a good session. Can't wait until next time! Goodbye."
		}

	def on_start(self, signal):
		time.sleep(5)

		if self.estop_flag == 0:
			random_choice = random.randint(0,9)
			self.dialogue_pub.publish(self.end_dict[random_choice])

			# print "Ok " + self.user.name + " that's the end of our session for today. I look forward to seeing you again next time!"
			# self.dialogue_pub.publish("Ok " + self.user.name + " that's the end of our session for today. I look forward to seeing you again next time. Goodbye!")

			#update all user info for csv here 
			self.user.session_number += 1
			self.user.programme_time = self.state.programme_time
			self.user.run_speed_pb = self.state.user_run_pb
			self.user.walk_speed_pb = self.state.user_walk_pb

			#re-calculate averages from state log 

			df = pd.read_csv('state_logger.csv', names=['time', 'state_id', 'user_id', 'session_condition','self_attitude', 'expert_attitude', 'extraversion', 'agreeableness', \
				'conscientiousness', 'emotional_stability', 'openness_experience', 'activity_level', 'session_number', 'programme_state', 'task_success', \
				'session_time', 'session_time_remaining', 'normalised_session_time', 'programme_time', 'normalised_programme_time', 'time_spent_prog_action', \
				'time_remaining_prog_action', 'programme_action_progress', 'programme_action_duration', 'current_speed', 'relative_speed_average', 'relative_speed_best', \
				'time_since_last_action', 'normalised_time_since_last_action', 'heart_rate', 'relative_heart_rate', 'au12', 'au25','session_mood', 'user_run_pb', 'user_walk_pb', \
				'user_run_avg','user_walk_avg'])

			#print df.head()

			#walk average
			relevant_df = df.loc[(df['user_id']==self.user.user_id) & (df['programme_state']==0.5) & (df['task_success']==1) & (df["current_speed"]!=0) & (df["programme_action_progress"]<=0.8)].copy()
			self.user.walk_speed_avg = relevant_df["current_speed"].mean(skipna=True)
			print "end_session: new average walk speed is: " + str(self.user.walk_speed_avg)

			print relevant_df.head()

			#run average
			relevant_df = df.loc[(df['user_id']==self.user.user_id) & (df['programme_state']==1) & (df['task_success']==1) & (df["current_speed"]!=0) & (df["programme_action_progress"]<=0.8)].copy()
			self.user.run_speed_avg = relevant_df["current_speed"].mean(skipna=True)
			print "end_session: new average run speed is: " + str(self.user.run_speed_avg)

			#print relevant_df.head()

			updated_record = [self.user.name, self.user.user_id, self.user.session_number, self.user.self_attitude, self.user.expert_attitude, self.user.extraversion, \
			self.user.agreeableness, self.user.conscientiousness, self.user.emotional_stability, self.user.openness_experience, self.user.activity_level, self.user.programme_time, \
			self.user.walk_speed_pb, self.user.walk_speed_avg, self.user.run_speed_pb, self.user.run_speed_avg, self.user.resting_heart_rate, self.user.motivator, \
			self.user.age]
			
			self.user.update_csv(updated_record)

		print "end_session: session complete - shutting down node"
		rospy.signal_shutdown("session complete")

	def on_user_info(self, user_info):
		self.user.set_user_id(user_info.user_id)
		self.user.set_from_csv()
		#print self.user.user_id

	def on_state(self, state):
		self.state = state

	def on_estop(self, signal):
		self.estop_flag = 1

if __name__ == "__main__":
	rospy.init_node('end')
	warmup = EndSession()
	rospy.spin()
