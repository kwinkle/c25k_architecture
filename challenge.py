import random
import rospy
import pandas as pd
from std_msgs.msg import String

class Challenge:

	def __init__(self, animspeechProxy, name, user_id):
		self.animspeechProxy = animspeechProxy
		self.subtitles_pub = rospy.Publisher('subtitles', String, queue_size=1) 

		self.user_id = user_id

		self.sympathetic_dict = {
			5: "I know this is going to be hard, but you're going to show me what you've got!",
			1: "Zone out, listen to your body and find that rhythm.",
			2: "No stress. Clear your mind and just run.",
			3: "I'm afraid it doesn't get easier, you only get tougher.",
			4: "We're here to make each day and every run worth it.",
			"I know this is going to be hard, but you're going to show me what you've got!": 5,
			"Zone out, listen to your body and find that rhythm.": 1,
			"No stress. Clear your mind and just run.": 2,
			"I'm afraid it doesn't get easier, you only get tougher.": 3,
			"We're here to make each day and every run worth it.": 4
		}

		self.challenging_dict = {
			9: "You WILL leave here only wishing you ran just that little bit harder. Make it count.",
			1: "Come on " + name + " show me what you're made of!!",
			2: "Keep up the intensity. We die today to be stronger tomorrow.",
			3: "Suffering builds character. You are tough!",
			4: "GIVE ME YOUR ENERGY!!",
			5: "What are you saving your energy for anyway?",
			6: "To change your body you MUST give it reason to adapt! There's no such thing as an easy challenge!",
			7: name + ", " + "getting tough yet? I'm here to show you what you are truly capable of.",
			8: "The only thing I want from you " + name + " is your effort!",
			"You WILL leave here only wishing you ran just that little bit harder. Make it count.": 9,
			"Come on " + name + " show me what you're made of!!": 1,
			"Keep up the intensity. We die today to be stronger tomorrow.": 2,
			"Suffering builds character. You are tough!": 3,
			"GIVE ME YOUR ENERGY!!": 4,
			"What are you saving your energy for anyway?": 5,
			"To change your body you MUST give it reason to adapt! There's no such thing as an easy challenge!": 6,
			name + ", " + "getting tough yet? I'm here to show you what you are truly capable of.": 7,
			"The only thing I want from you " + name + " is your effort!": 8
		}


	def sympathetic(self):

		df = pd.read_csv('behaviour_logger.csv', names=['user_id','session_number','state_id','timestamp','action_id','action', \
			'style','duration','validation','speech'])

		relevant_df = df.loc[(df['user_id']==self.user_id) & (df['action']=="CHALLENGE") & (df['style']=="SYMPATHETIC") & (df['speech']!="(no speech - style update)")].copy()

		if relevant_df.empty:
			choice = 1
		else:
			print relevant_df.head()

			last_utterance = relevant_df['speech'].iloc[-1]
			last_utterance = last_utterance.replace("-",", ")

			print last_utterance

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

	def challenging(self):

		df = pd.read_csv('behaviour_logger.csv', names=['user_id','session_number','state_id','timestamp','action_id','action', \
			'style','duration','validation','speech'])

		relevant_df = df.loc[(df['user_id']==self.user_id) & (df['action']=="CHALLENGE") & (df['style']=="CHALLENGING") & (df['speech']!="(no speech - style update)")].copy()

		if relevant_df.empty:
			choice = 1
		else:
			print relevant_df.head()

			last_utterance = relevant_df['speech'].iloc[-1]
			last_utterance = last_utterance.replace("-",", ")

			print last_utterance

			index = self.challenging_dict.get(last_utterance)

			if index:
				if index == 9:
					choice = 1
				else:
					choice = index + 1
			else:
				choice = 1

		self.subtitles_pub.publish(self.challenging_dict[choice])
		self.animspeechProxy.say(self.challenging_dict[choice])
		ret_str = self.challenging_dict[choice]
		ret_str = ret_str.replace(", ","-")
		return ret_str