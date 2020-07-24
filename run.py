import random
import rospy
from std_msgs.msg import String

class Run: 

	def __init__(self, animspeechProxy):
		self.animspeechProxy = animspeechProxy
		self.subtitles_pub = rospy.Publisher('subtitles', String, queue_size=1)

		self.neutral_dict = {
			0: "Now it's time to run for ",
			1: "Ok, let's switch to running for ",
			2: "Speed up now and run for "
		}

		self.sympathetic_dict = {
			0: "Ok now I know you can do this, next is a run for ",
			1: "Come on you've got this, next up is a run for ",
			2: "Get ready for the next run. You can do it. We're going to go for "
		}

		self.positive_dict = {
			0: "Good stuff, time to switch up to running. This one is for ",
			1: "Let's keep this momentum now into our next run, which is for ",
			2: "Keep up this effort, now it's time to run for "
		}

		self.challenging_dict = {
			0: "Right I want to see you push hard on this next run for ",
			1: "Give me everything you've got on this next run, which will be ",
			2: "Come on now let's push hard on this next run for "
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
		self.subtitles_pub.publish(self.neutral_dict[self.neutral_choice]+str(mins)+" minute "+str(seconds) + ". In 3, 2, 1, go!")
		self.animspeechProxy.say(self.neutral_dict[self.neutral_choice]+str(mins)+" minute "+str(seconds) + ". In 3, 2, 1, go!")
		# id = self.animspeechProxy.post.say(self.neutral_dict[self.neutral_choice]+str(duration)+"seconds. " + "In 3, 2, 1, go!")
		# self.animspeechProxy.wait(id,0)
		self.neutral_lastchoice = self.neutral_choice
		return str(self.neutral_dict[self.neutral_choice]+str(mins)+" minute "+str(seconds) + ". In 3, 2, 1, go!")

	def sympathetic(self, duration): 
		seconds =  duration % 60
		mins = (duration - seconds)/60

		while self.sympathetic_choice == self.sympathetic_lastchoice:
			self.sympathetic_choice = random.randint(0,2)
		self.subtitles_pub.publish(self.sympathetic_dict[self.sympathetic_choice]+str(mins)+" minute "+str(seconds) + ". In 3, 2, 1, go!")
		self.animspeechProxy.say(self.sympathetic_dict[self.sympathetic_choice]+str(mins)+" minute "+str(seconds) + ". In 3, 2, 1, go!")
		# id = self.animspeechProxy.post.say(self.sympathetic_dict[self.sympathetic_choice]+str(duration)+"seconds. " + "In 3, 2, 1, go!")
		# self.animspeechProxy.wait(id,0)
		self.sympathetic_lastchoice = self.sympathetic_choice
		return str(self.sympathetic_dict[self.sympathetic_choice]+str(duration)+str(mins)+" minute "+str(seconds) + ". In 3, 2, 1, go!")

	def positive(self, duration): 
		seconds =  duration % 60
		mins = (duration - seconds)/60

		while self.positive_choice == self.positive_lastchoice:
			self.positive_choice = random.randint(0,2)
		self.subtitles_pub.publish(self.positive_dict[self.positive_choice]+str(mins)+" minute "+str(seconds) + ". In 3, 2, 1, go!")
		self.animspeechProxy.say(self.positive_dict[self.positive_choice]+str(mins)+" minute "+str(seconds) + ". In 3, 2, 1, go!")
		# id = self.animspeechProxy.post.say(self.positive_dict[self.positive_choice]+str(duration)+"seconds. " + "In 3, 2, 1, go!")
		# self.animspeechProxy.wait(id,0)
		self.positive_lastchoice = self.positive_choice
		return str(self.positive_dict[self.positive_choice]+str(duration)+str(mins)+" minute "+str(seconds) + ". In 3, 2, 1, go!")

	def challenging(self, duration):
		seconds =  duration % 60
		mins = (duration - seconds)/60

		while self.challenging_choice == self.challenging_lastchoice:
			self.challenging_choice = random.randint(0,2)
		self.subtitles_pub.publish(self.challenging_dict[self.challenging_choice]+str(mins)+" minute "+str(seconds) + ". In 3, 2, 1, go!")
		self.animspeechProxy.say(self.challenging_dict[self.challenging_choice]+str(mins)+" minute "+str(seconds) + ". In 3, 2, 1, go!")
		# id = self.animspeechProxy.say(self.challenging_dict[self.challenging_choice]+str(duration)+"seconds. " + "In 3, 2, 1, go!")
		# self.animspeechProxy.wait(id,0)
		self.challenging_lastchoice = self.challenging_choice	
		return str(self.challenging_dict[self.challenging_choice]+str(mins)+" minute "+str(seconds) + ". In 3, 2, 1, go!")