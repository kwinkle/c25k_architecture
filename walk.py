import random
import rospy
from std_msgs.msg import String

class Walk: #TO-DO probably want to change use of seconds to minutes once up to certain session (simple if switch)

	def __init__(self, animspeechProxy):
		self.animspeechProxy = animspeechProxy
		self.subtitles_pub = rospy.Publisher('subtitles', String, queue_size=1)

		self.neutral_dict = {
			0: "Ok, start thinking about bringing it back down to a walk. Slow down now and we'll walk for ",
			1: "Get ready for another rest now, ok, let's switch back to walking for ",
			2: "Time to think about slowing it down now. Bring it down to a walk for "
		}

		self.sympathetic_dict = {
			0: "Phew! Time for a break, slow down and walk for ",
			1: "Ok, I know that was hard, let's switch back to walking for ",
			2: "That seemed tough! Slow down now and walk for "
		}

		self.positive_dict = {
			0: "You did it! Now it's time to walk for ",
			1: "Great job! Now let's switch back to walking for ",
			2: "Well done! Slow down now and walk for "
		}

		self.challenging_dict = {
			0: "Now give me a good pace walk for ",
			1: "Keep it strong now with a fast walk for ",		
			2: "Ok now I want you to walk at a solid place for "
		}

		self.neutral_lastchoice = 0
		self.neutral_choice = 0

		self.sympathetic_lastchoice = 0
		self.sympathetic_choice = 0

		self.positive_lastchoice = 0
		self.positive_choice = 0

		self.challenging_lastchoice = 0
		self.challenging_choice = 0

	def neutral(self, duration):
		seconds =  duration % 60
		mins = (duration - seconds)/60

		while self.neutral_choice == self.neutral_lastchoice:
			self.neutral_choice = random.randint(0,2)
		self.subtitles_pub.publish(self.neutral_dict[self.neutral_choice]+str(mins)+" minute "+str(seconds))
		self.animspeechProxy.say(self.neutral_dict[self.neutral_choice]+str(mins)+" minute "+str(seconds))
		# id = self.animspeechProxy.post.say(self.neutral_dict[self.neutral_choice]+str(duration)+"seconds")
		# self.animspeechProxy.wait(id,0)
		self.neutral_lastchoice = self.neutral_choice
		return str(self.neutral_dict[self.neutral_choice]+str(duration)+" minute "+str(seconds))

	def sympathetic(self,duration):
		seconds =  duration % 60
		mins = (duration - seconds)/60
		while self.sympathetic_choice == self.sympathetic_lastchoice:
			self.sympathetic_choice = random.randint(0,2)
		self.subtitles_pub.publish(self.sympathetic_dict[self.sympathetic_choice]+str(mins)+" minute "+str(seconds))
		self.animspeechProxy.say(self.sympathetic_dict[self.sympathetic_choice]+str(mins)+" minute "+str(seconds))
		# id = self.animspeechProxy.post.say(self.sympathetic_dict[self.sympathetic_choice]+str(duration)+"seconds")
		# self.animspeechProxy.wait(id,0)
		self.sympathetic_lastchoice = self.sympathetic_choice
		return str(self.sympathetic_dict[self.sympathetic_choice]+str(mins)+" minute "+str(seconds))

	def positive(self,duration):
		seconds =  duration % 60
		mins = (duration - seconds)/60
		while self.positive_choice == self.positive_lastchoice:
			self.positive_choice = random.randint(0,2)
		self.subtitles_pub.publish(self.positive_dict[self.positive_choice]+str(mins)+" minute "+str(seconds))
		self.animspeechProxy.say(self.positive_dict[self.positive_choice]+str(mins)+" minute "+str(seconds))
		# id = self.animspeechProxy.post.say(self.positive_dict[self.positive_choice]+str(duration)+"seconds")
		# self.animspeechProxy.wait(id,0)
		self.positive_lastchoice = self.positive_choice
		return str(self.positive_dict[self.positive_choice]+str(mins)+" minute "+str(seconds))

	def challenging(self, duration):
		seconds =  duration % 60
		mins = (duration - seconds)/60
		while self.challenging_choice == self.challenging_lastchoice:
			self.challenging_choice = random.randint(0,2)
		self.subtitles_pub.publish(self.challenging_dict[self.challenging_choice]+str(mins)+" minute "+str(seconds))
		self.animspeechProxy.say(self.challenging_dict[self.challenging_choice]+str(mins)+" minute "+str(seconds))
		# id = self.animspeechProxy.post.say(self.challenging_dict[self.challenging_choice]+str(duration)+"seconds")
		# self.animspeechProxy.wait(id,0)
		self.challenging_lastchoice = self.challenging_choice
		return str(self.challenging_dict[self.challenging_choice]+str(mins)+" minute "+str(seconds))
