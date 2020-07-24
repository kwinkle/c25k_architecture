import random
import rospy
from std_msgs.msg import String

class TimeAction:

	def __init__(self, animspeechProxy):
		self.animspeechProxy = animspeechProxy
		self.subtitles_pub = rospy.Publisher('subtitles', String, queue_size=1)


		self.challenging_dict = {
			0: "You will NOT give up!",
			1: "COME ON, YOU GOT THIS!!",
			2: "Give me everything you have left!",
			3: "With every step you take seconds tick by",
			4: "Don't you dare give up on me now, just a litte longer!"
		}

		self.sympathetic_dict = {
			0: "Just a little bit further now and then we can take our active rest.",
			1: "Not long now!",
			2: "Almost there, I know you've got this!",
			3: "Hold on, you're almost done!"
		}


		self.positive_dict = {
			0: "This is great work, keep this pace up!",
			1: "You are flying through this run!!"
		}


		self.positive_lastchoice = 0
		self.positive_choice = 0

		self.challenging_lastchoice = 0
		self.challenging_choice = 0

		self.sympathetic_lastchoice = 0
		self.sympathetic_choice = 0

	def challenging(self):
		while self.challenging_choice == self.challenging_lastchoice:
			self.challenging_choice = random.randint(0,4)
		self.subtitles_pub.publish(self.challenging_dict[self.challenging_choice])
		self.animspeechProxy.say(self.challenging_dict[self.challenging_choice])
		self.challenging_lastchoice = self.challenging_choice	
		return str(self.challenging_dict[self.challenging_choice])	

	def sympathetic(self):
		while self.sympathetic_choice == self.sympathetic_lastchoice:
			self.sympathetic_choice = random.randint(0,3)
		self.subtitles_pub.publish(self.sympathetic_dict[self.sympathetic_choice])
		self.animspeechProxy.say(self.sympathetic_dict[self.sympathetic_choice])
		self.sympathetic_lastchoice = self.sympathetic_choice
		return str(self.sympathetic_dict[self.sympathetic_choice])

	def positive(self):
		while self.positive_choice == self.positive_lastchoice:
			self.positive_choice = random.randint(0,1)
		self.subtitles_pub.publish(self.positive_dict[self.positive_choice])
		self.animspeechProxy.say(self.positive_dict[self.positive_choice])
		self.positive_lastchoice = self.positive_choice
		return str(self.positive_dict[self.positive_choice])

