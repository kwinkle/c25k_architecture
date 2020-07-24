import random
import rospy
from std_msgs.msg import String

class SpeedDown:

	def __init__(self, animspeechProxy):
		self.animspeechProxy = animspeechProxy
		self.subtitles_pub = rospy.Publisher('subtitles', String, queue_size=1)
		
		self.sympathetic_dict = {
			0: "Don't forget to pace yourself!",
			1: "You 'might' want to slow down a little bit",
			2: "Woah there, let's not burn out too fast...",
			3: "Don't die on me just yet, you could bring the speed down a little",
			4: "Remember you have to survive until the end of the run."
		}	

		self.sympathetic_lastchoice = 0
		self.sympathetic_choice = 0

	def sympathetic(self):
		while self.sympathetic_choice == self.sympathetic_lastchoice:
			self.sympathetic_choice = random.randint(0,1)
		self.subtitles_pub.publish(self.sympathetic_dict[self.sympathetic_choice])
		self.animspeechProxy.say(self.sympathetic_dict[self.sympathetic_choice])
		self.sympathetic_lastchoice = self.sympathetic_choice
		return str(self.sympathetic_dict[self.sympathetic_choice])