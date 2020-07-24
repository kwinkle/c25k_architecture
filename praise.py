import random
import rospy
import pandas as pd
from std_msgs.msg import String

class Praise:

	def __init__(self, animspeechProxy, user_id):
		self.animspeechProxy = animspeechProxy
		self.subtitles_pub = rospy.Publisher('subtitles', String, queue_size=1) 

		self.user_id = user_id

		# self.neutral_dict = { #need to think here whether neutral choice is consistent in terms of social agency for fair comparison against supervised (i.e. the I'm impressed/I know...)
		# 	0: "You're giving great effort",
		# 	1: "You're doing a good job"
		# }

		self.sympathetic_dict = {
			5: "Ah! I knew you were stronger than you realised!!",
			1: "I know this is hard but you've got it",
			2: "Don't forget success isn't some one time event, it's making small victories for yourself, day in day out.",
			3: "Damn, You're making me proud!",
			4: "You're doing well, keep it up",
			"Ah! I knew you were stronger than you realised!!": 5,
			"I know this is hard but you've got it": 1,
			"Don't forget success isn't some one time event, it's making small victories for yourself, day in day out.": 2,
			"Damn, You're making me proud!": 3,
			"You're doing well, keep it up": 4
		}

		self.positive_dict = {
			8: "I'm impressed, you're doing great!",
			1: "You've got this!",
			2: "You're doing really well, keep this up.",
			3: "You're nailing this",
			4: "You'll be destroying 5k runs in no time!!",
			#5: "I wish your past self could see you now!",
			5: "Loving this effort! You are stronger than you realise",
			6: "Damn, you're making this look easy!",
			7: "Your evolutionary ancestors would be proud of you",
			"I'm impressed, you're doing great!": 8,
			"You've got this!": 1,
			"You're doing really well, keep this up.": 2,
			"You're nailing this": 3,
			"You'll be destroying 5k runs in no time!!": 4,
			#5: "I wish your past self could see you now!",
			"Loving this effort! You are stronger than you realise": 5,
			"Damn, you're making this look easy!": 6,
			"Your evolutionary ancestors would be proud of you": 7
		}


	def sympathetic(self):
		df = pd.read_csv('behaviour_logger.csv', names=['user_id','session_number','state_id','timestamp','action_id','action', \
			'style','duration','validation','speech'])

		relevant_df = df.loc[(df['user_id']==self.user_id) & (df['action']=="PRAISE") & (df['style']=="SYMPATHETIC") & (df['speech']!="(no speech - style update)")].copy()

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

	def positive(self):
		df = pd.read_csv('behaviour_logger.csv', names=['user_id','session_number','state_id','timestamp','action_id','action', \
			'style','duration','validation','speech'])

		relevant_df = df.loc[(df['user_id']==self.user_id) & (df['action']=="PRAISE") & (df['style']=="POSITIVE") & (df['speech']!="(no speech - style update)")].copy()

		if relevant_df.empty:
			choice = 1
		else:
			#print relevant_df.head()

			last_utterance = relevant_df['speech'].iloc[-1]
			last_utterance = last_utterance.replace("-",", ")

			#print last_utterance

			index = self.positive_dict.get(last_utterance)

			if index:
				if index == 8:
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