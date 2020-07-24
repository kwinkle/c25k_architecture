import random
import rospy
from std_msgs.msg import String

class SpeedUp:

	def __init__(self, animspeechProxy):
		self.animspeechProxy = animspeechProxy
		self.subtitles_pub = rospy.Publisher('subtitles', String, queue_size=1)

		self.neutral_dict = {
			0: "Can you push a bit harder? Maybe turn up the speed.", #TO-DO: refer by name, how to reference prev. performance?
			1: "You might want to try turning the speed up"
		}

		self.sympathetic_dict = {
			0: "I know this is tough but you can do it, can you turn up the speed a little more?",
			1: "This is hard work but you're doing great! I think you can give me more. Maybe turn the speed up a bit"
		}

		self.challenging_dict = {
			0: "Come on you can work harder than this! Could you turn the speed up?",
			1: "I know you can push harder than this, can you turn the speed up?"
		}

		self.neutral_lastchoice = 0
		self.neutral_choice = 0

		self.sympathetic_lastchoice = 0
		self.sympathetic_choice = 0

		self.challenging_lastchoice = 0
		self.challenging_choice = 0

	def neutral(self):
		while self.neutral_choice == self.neutral_lastchoice:
			self.neutral_choice = random.randint(0,1)
		self.subtitles_pub.publish(self.neutral_dict[self.neutral_choice])
		self.animspeechProxy.say(self.neutral_dict[self.neutral_choice])
		self.neutral_lastchoice = self.neutral_choice
		return str(self.neutral_dict[self.neutral_choice])

	def sympathetic(self):
		while self.sympathetic_choice == self.sympathetic_lastchoice:
			self.sympathetic_choice = random.randint(0,1)
		self.subtitles_pub.publish(self.sympathetic_dict[self.sympathetic_choice])
		self.animspeechProxy.say(self.sympathetic_dict[self.sympathetic_choice])
		self.sympathetic_lastchoice = self.sympathetic_choice
		return str(self.sympathetic_dict[self.sympathetic_choice])

	def challenging(self):
		while self.challenging_choice == self.challenging_lastchoice:
			self.challenging_choice = random.randint(0,1)
		self.subtitles_pub.publish(self.challenging_dict[self.challenging_choice])
		self.animspeechProxy.say(self.challenging_dict[self.challenging_choice])
		self.challenging_lastchoice = self.challenging_choice
		return str(self.challenging_dict[self.challenging_choice])