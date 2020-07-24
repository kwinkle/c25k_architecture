import random
import rospy
import pandas as pd
from std_msgs.msg import String

class Sympathise:

	def __init__(self, animspeechProxy, user_id):
		self.animspeechProxy = animspeechProxy
		self.subtitles_pub = rospy.Publisher('subtitles', String, queue_size=1)

		self.user_id = user_id

		self.sympathetic_dict = {
			5: "Run as hard as you need. To me, self-love is all I want you to leave here with.",
			1: "Remember you and your body are working together. Not fighting each other.",
			2: "This training should be an achievement of what your body can do, not a punishment for what you ate.",
			3: "It doesn't matter how you finish, even a bad run has its benefits.",
			4: "Remember why you're here. Remember why you're doing this",
			"Run as hard as you need. To me, self-love is all I want you to leave here with.": 5,
			"Remember you and your body are working together. Not fighting each other.": 1,
			"This training should be an achievement of what your body can do, not a punishment for what you ate.": 2,
			"It doesn't matter how you finish, even a bad run has its benefits.": 3,
			"Remember why you're here. Remember why you're doing this": 4
		}


	def sympathetic(self):

		df = pd.read_csv('behaviour_logger.csv', names=['user_id','session_number','state_id','timestamp','action_id','action', \
			'style','duration','validation','speech'])

		relevant_df = df.loc[(df['user_id']==self.user_id) & (df['action']=="SYMPATHISE") & (df['style']=="SYMPATHETIC") & (df['speech']!="(no speech - style update)")].copy()

		if relevant_df.empty:
			choice = 1
		else:
			#print relevant_df.head()

			last_utterance = relevant_df['speech'].iloc[-1]
			last_utterance = last_utterance.replace("-",", ")

			#print last_utterance

			index = self.sympathetic_dict.get(last_utterance)

			if index:
				if index == 5:
					choice = 1
				else:
					choice = index + 1
			else:
				choice = 1


		self.subtitles_pub.publish(self.sympathetic_dict[choice])
		self.animspeechProxy.say(self.sympathetic_dict[choice])
		ret_str = self.sympathetic_dict[choice]
		ret_str = ret_str.replace(", ","-")
		return ret_str