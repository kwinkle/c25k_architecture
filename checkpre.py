import random
import rospy
import pandas as pd
from std_msgs.msg import String

class CheckPRE:

	def __init__(self, animspeechProxy, name, user_id):
		self.animspeechProxy = animspeechProxy
		self.checkmood_pub = rospy.Publisher('check_mood', String, queue_size=1)
		self.subtitles_pub = rospy.Publisher('subtitles', String, queue_size=1) 

		self.user_id = user_id
		self.name = name

		self.sympathetic_dict = {
			5: "How're you feeling?",
			1: "What are those legs saying?",
			2: "Hey " + name + " how are you holding up?", 
			3: "How's your body feeling?",
			4: "How are you holding up " + name + " ?",
			"How're you feeling?": 5,
			"What are those legs saying?": 1,
			"Hey " + name + " how are you holding up?": 2, 
			"How's your body feeling?": 3,
			"How are you holding up " + name + " ?": 4
		}


	def sympathetic(self):
		df = pd.read_csv('behaviour_logger.csv', names=['user_id','session_number','state_id','timestamp','action_id','action', \
			'style','duration','validation','speech'])

		relevant_df = df.loc[(df['user_id']==self.user_id) & (df['action']=="CHECKPRE") & (df['style']=="SYMPATHETIC") & (df['speech']!="(no speech - style update)")].copy()

		if relevant_df.empty:
			choice = 1
		else:
			print relevant_df.head()

			last_utterance = relevant_df['speech'].iloc[-1]
			last_utterance = last_utterance.replace("-",", ")

			print last_utterance

			index = self.sympathetic_dict[last_utterance]

			if index == 5:
				choice = 1
			else:
				choice = index + 1

		self.subtitles_pub.publish(self.sympathetic_dict[choice])
		self.animspeechProxy.say(self.sympathetic_dict[choice])
		self.checkmood_pub.publish(self.sympathetic_dict[choice])
		ret_str = self.sympathetic_dict[choice]
		ret_str = ret_str.replace(", ","-")
		return ret_str