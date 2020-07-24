import random
import rospy
import pandas as pd
from std_msgs.msg import String

class Humour:

	def __init__(self, animspeechProxy, user_id):
		self.animspeechProxy = animspeechProxy
		self.subtitles_pub = rospy.Publisher('subtitles', String, queue_size=1) 

		self.user_id = user_id

		# self.neutral_dict = { #need to think here whether neutral choice is consistent in terms of social agency for fair comparison against supervised (i.e. the I'm impressed/I know...)
		# 	0: "You're giving great effort",
		# 	1: "You're doing a good job"
		# }

		self.positive_dict = {
			8: "Just watching you is making my legs hurt, oh wait",
			1: "I heard your ancestors ate 5K runs for breakfast!",
			2: "You can call me Terminator because I'm going to make you run for your life!",
			3: "Run! Run like you're escaping the robotic revolution!",
			4: "The estimated speed of a T-rex is 27 miles per hour. You know, just saying",
			5: "Running is just putting one leg in front of the other, how hard could that be?",
			6: "I wish I had legs",
			7: "Damn, if I only had legs I'd join you",
			"Just watching you is making my legs hurt, oh wait": 8,
			"I heard your ancestors ate 5K runs for breakfast!": 1,
			"You can call me Terminator because I'm going to make you run for your life!": 2,
			"Run! Run like you're escaping the robotic revolution!": 3,
			"The estimated speed of a T-rex is 27 miles per hour. You know, just saying": 4,
			"Running is just putting one leg in front of the other, how hard could that be?": 5,
			"I wish I had legs": 6,
			"Damn, if I only had legs I'd join you": 7
		}

		# self.neutral_lastchoice = 0
		# self.neutral_choice = 0

		self.positive_lastchoice = 0
		self.positive_choice = 0

	# def neutral(self):
	# 	while self.neutral_choice == self.neutral_lastchoice:
	# 		self.neutral_choice = random.randint(0,1)
	# 	self.animspeechProxy.say(self.neutral_dict[self.neutral_choice])
	# 	self.neutral_lastchoice = self.neutral_choice

	def positive(self):
		df = pd.read_csv('behaviour_logger.csv', names=['user_id','session_number','state_id','timestamp','action_id','action', \
			'style','duration','validation','speech'])

		relevant_df = df.loc[(df['user_id']==self.user_id) & (df['action']=="HUMOUR") & (df['style']=="POSITIVE") & (df['speech']!="(no speech - style update)")].copy()

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