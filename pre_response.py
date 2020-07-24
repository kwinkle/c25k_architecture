import random
import rospy
from std_msgs.msg import String

class Pre_Response:
	def __init__(self, animspeechProxy):
		self.animspeechProxy = animspeechProxy
		self.subtitles_pub = rospy.Publisher('subtitles', String, queue_size=1)

		self.neutral_dict = {
			0: "As long as you're not dying",
			1: "Good, I don't want to scare you off!",
			2: "I hope you're not regretting signing up now!",
			3: "I promise you'll survive"
		}

		self.positive_dict = {
			0: "That's good to hear",
			1: "Good, I'm glad!",
			2: "Ideal",
			3: "Awesome!"
		}

		self.negative_dict = { #need a second level here
			0: "Feel free to bring the speed down if you need to",
			1: "Ok, I hear what you're saying"
		}

		self.neutral_lastchoice = 0
		self.neutral_choice = 0

		self.positive_lastchoice = 0
		self.positive_choice = 0

		self.negative_lastchoice = 0
		self.negative_choice = 0	

	def neutral(self):
		while self.neutral_choice == self.neutral_lastchoice:
			self.neutral_choice = random.randint(0,3)
		self.subtitles_pub.publish(self.neutral_dict[self.neutral_choice])
		self.animspeechProxy.say(self.neutral_dict[self.neutral_choice])
		self.neutral_lastchoice = self.neutral_choice

	def positive(self):
		while self.positive_choice == self.positive_lastchoice:
			self.positive_choice = random.randint(0,3)
		self.subtitles_pub.publish(self.positive_dict[self.positive_choice])
		self.animspeechProxy.say(self.positive_dict[self.positive_choice])
		self.positive_lastchoice = self.positive_choice		

	def negative(self):
		while self.negative_choice == self.negative_lastchoice:
			self.negative_choice = random.randint(0,1)
		self.subtitles_pub.publish(self.negative_dict[self.negative_choice])
		self.animspeechProxy.say(self.negative_dict[self.negative_choice])
		self.negative_lastchoice = self.negative_choice	