import random
import rospy
import pandas as pd
from std_msgs.msg import String

class Maintain:

	def __init__(self, animspeechProxy, name, user_id):
		self.animspeechProxy = animspeechProxy
		self.subtitles_pub = rospy.Publisher('subtitles', String, queue_size=1) 

		self.user_id = user_id

		self.positive_dict = {
			7: "Feel the rhythm of your body and go with it",
			1: "Focus on your breathing",
			2: "Relax your shoulders, allow your arms to swing naturally with the motion",
			3: "Great work, now keep that pace!",
			4: "Don't forget to listen to what your body's saying!",
			5: name + ", " + "take your mind where it needs to go and just keep this intensity up.",
			6: name + ", " + "just find the flow of movement within the body.",
			"Feel the rhythm of your body and go with it": 7,
			"Focus on your breathing": 1,
			"Relax your shoulders, allow your arms to swing naturally with the motion": 2,
			"Great work, now keep that pace!": 3,
			"Don't forget to listen to what your body's saying!": 4,
			name + ", " + "take your mind where it needs to go and just keep this intensity up.": 5,
			name + ", " + "just find the flow of movement within the body.": 6
		}

	def positive(self):

		df = pd.read_csv('behaviour_logger.csv', names=['user_id','session_number','state_id','timestamp','action_id','action', \
			'style','duration','validation','speech'])

		relevant_df = df.loc[(df['user_id']==self.user_id) & (df['action']=="MAINTAIN") & (df['style']=="POSITIVE") & (df['speech']!="(no speech - style update)")].copy()

		if relevant_df.empty:
			choice = 1
		else: 
			#print relevant_df.head()

			last_utterance = relevant_df['speech'].iloc[-1]
			last_utterance = last_utterance.replace("-",", ")

			#print last_utterance

			index = self.positive_dict.get(last_utterance)

			if index:
				if index == 7:
					choice = 1
				else:
					choice = index + 1
			else:
				choice = 1
	
		self.subtitles_pub.publish(self.positive_dict[choice])
		self.animspeechProxy.say(self.positive_dict[choice])
		ret_str = self.positive_dict[choice]
		ret_str = ret_str.replace(", ","-")
		return ret_str
