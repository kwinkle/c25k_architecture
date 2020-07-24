#!/usr/bin/env python

import rospy
import csv
import random
from std_msgs.msg import String, Float32
from std_srvs.srv import Empty
from c25k_msgs.msg import C25KUserInfo
#from naoqi import ALProxy
from user_class import User

class WarmUp():
	def __init__(self):

		rospy.Subscriber('user_start', String, self.on_start)
		rospy.Subscriber('warmup_time', Float32, self.on_time)
		self.mood_subscriber = rospy.Subscriber('session_mood', String, self.on_mood)
		self.dialogue_pub = rospy.Publisher('extra_dialogue', String, queue_size=1)
		self.checkmood_pub = rospy.Publisher('check_mood', String, queue_size=1)
		self.user_info_pub = rospy.Publisher('user_info', C25KUserInfo, queue_size=1)
		self.warmup_complete_pub = rospy.Publisher('warmup_complete', String, queue_size=1)
		self.programme_state_pub = rospy.Publisher('programme_state', String, queue_size=1)


		rospy.wait_for_service('warmup_complete')
		self.trigger_warmup_complete = rospy.ServiceProxy('warmup_complete', Empty)

		self.user = User()
		self.time = 0

		self.neutral_dict = {
			0: "Ok, I hope this session will leave you feeling more positive!", #TO-DO: get Don's advice on these 
			1: "I see, well let's work out and get you feeling positive!",
			2: "Ok, well let's get positive with this workout!"
		}

		self.positive_dict = {
			0: "I'm so glad to hear that, let's have a good session too!",
			1: "Excellent, this will be a good session then!",
			2: "It's great that you're feeling good!"
		}

		self.negative_dict = {
			0: "I'm sorry to hear that, I hope this session will improve things",
			1: "Oh no. Come on, let's change that with this workout",
			2: "Oh, not good. I'll try and make this session as enjoyable as possible for you."
		}

		self.warmup_dict1 = {
			0: "For all of your sessions in this first week of the programme we will be alternating between 60 seconds of running and 90 seconds of walking. But don't worry, I'll tell you when it's time to run and when it's time to walk.",
			1: "Today we will again be alternating between 60 seconds of running and 90 seconds of walking. Just like last time, I'll tell you when it's time to run and when it's time to walk.",
			2: "Just like last time, today we will be alternating between 60 seconds of running and 90 seconds of walking. But don't worry, I'll tell you when it's time to run and when it's time to walk.",
			3: "Well done for getting to Week 2! This week we'll be alternating between 90 seconds of running and 2 minutes of walking for about 20 minutes. Like last week, I'll tell you when to run and when to walk.",
			4: "Today we'll again be alternating between 90 seconds of running and 2 minutes of walking for about 20 minutes. As always, I'll tell you when to run and when to walk.",
			5: "Just like last time, today we will be alternating between 90 seconds of running and 2 minutes of walking. But don't worry, I'll tell you when it's time to run and when it's time to walk.",
			6: "Today we move on to Week 3 of the programme - well done for getting this far! In this week's sessions we'll be doing two repetitions of the following: 90 seconds of running, then 90 seconds of walking, then 3 minutes of running, then 3 minutes of walking.",
			7: "Today we'll again be doing two repetitions of the following: 90 seconds of running, then 90 seconds of walking, then 3 minutes of running, then 3 minutes of walking. I know it's a bit complicated, but I'll be guiding you all the way through so don't worry about that.",
			8: "Just like last time, today we'll again be doing two repetitions of the following: 90 seconds of running, then 90 seconds of walking, then 3 minutes of running, then 3 minutes of walking. As always I'll keep time and tell you when to run and when to walk.",
			9: "We're now on to Week 4 of the programme, and we're getting serious this week with some longer runs. This week we'll be doing 3 minutes of running and 90 seconds of walking, then 5 minutes running, 2 and a half minutes walking, 3 minutes running, 90 seconds walking, then a final 5 minutes running. Don't worry though, I'll tell you when to run and when to walk.", 
			10: "Today we'll again be doing 3 minutes of running and 90 seconds of walking, then 5 minutes running, 2 and a half minutes walking, 3 minutes running, 90 seconds walking, then a final 5 minutes running. As always, I'll be here to tell you when to run and when to walk. ",
			11: "Just like last time, today we'll again be doing 3 minutes of running and 90 seconds of walking, then 5 minutes running, 2 and a half minutes walking, 3 minutes running, 90 seconds walking, then a final 5 minutes running. And of course, I'll tell you when to run and when to walk, so don't worry about that. ",
			12: "So we're up to Week 5 now , and fast approaching half way through the programme! Well done on sticking with it. This week we'll really move things up a level, and surge forwards with some real progress. Today you'll run for 5 minutes, walk for 3, run for 5, walk for 3 and finish by running for another 5. ",
			13: "Now that we're in Week 5 you won't be repeating runs so much any more, so this session is different to last time. Today you've got 8 minutes of running, then 5 minutes of walking then 8 minutes of running, so it will be important to pace yourself properly.",
			14: "For the final session of Week 5 we're going to take a significant step forward in your training. Today you're going to run for 20 minutes with no stopping or walking. I know this might sound like quite a jump, but have faith in the plan. You've done all the hard work to prepare yourself for this. Once you're done, we'll do a 5 minute cool down walk.",
			15: "This is the first session of Week 6 and again we have 3 different runs this week. Today we'll be consolidating our progress with 5 minutes of running, 3 minutes walking, 8 minutes running, 3 minutes walking and then 5 minutes running, As always, I'll be here to let you know when to swap between walking and running.",
			16: "For today's session we'll be running for 10 minutes, walking for 3 and then running for another 10. As always, make sure your warmup pace is nice and brisk to get you ready for that run.",
			17: "Today we'll be running for 25 minutes non stop. You can definitely do this - you've done all the training you need in previous weeks. You just need to keep going with the technique you've used to get this far. We'll also do a 5 minute cool down walk at the end.",
			18: "We're up to Week 7 of the programme now, which requires you to repeat the same run 3 times. From now on it's all running after the warm up, then a 5 minute cool down at the end. Today you'll be running for 25 minutes non-stop.",
			19: "Once again today we'll be going for a 25 minute run. Up until now the Couch to 5k programme has given you rests in between runs, so I know these longer runs might seem relentless but it's just a process of readjustment.",
			20: "Ok so once again for the final session of Week 7 today is another 25 minute run once you're done with the warm up.",
			21: "Today is the first session of Week 8 - we're nearly finished now! Today we're going to run for a bit longer - 28 minutes in total once you're done warming up.",
			22: "We've got another 28 minute run today - I'm sure you're feeling confident about running on a regular basis now.",
			23: "So for the final session of this week it's another long run - with all the longer runs you've been doing over the last few weeks I hope you're feeling a real sense of achievement!",
			24: "We're up to Week 9 - the final week of the programme! This week you'll finally reach your goal and running 30 minutes, covering about 5k. ",
			25: "Once you've warmed up it'll be another 30 minute run today. Don't forget that next session will be our last one so make it count!",
			26: "And here we are on our final session! Well done for getting all the way through the programme. I hope you're feeling a real sense of achievement. Just one more 30 minute run to go today and we'll be done, although I hope this won't be the end of running for you!"

	}

		self.warmup_dict2 = {
			0: "How you hold your head is key to overall posture, which determines how efficiently you run. Look straight ahead naturally and let your gaze guide you.",
			1: "Don't forget to keep a nice neutral head position, there's nothing interesting on the floor.",
			2: "Relax your shoulders, they should be low and loose, not high and tight.",
			3: "Don't hold any tension in your shoulders, shake them out, draw them back and allow them to move freely.",
			4: "Your shoulders should remain level and not dip side-to-side while you run in order to maintain an efficient running technique and posture.",
			5: "Your arms should be relaxed at roughly 90 degree angles and should swing alternatively back and forth, not across the body.",
			6: "When setting your warm up speed, it should be brisk enough to get your heart beating faster whilst still leaving you able to hold a conversation.",
			7: "If your elbows are pointing outwards, that means your arms are crossing your body, which slows you down costing both energy and momentum.",
			8: "The ideal running posture is decribed as 'running tall' meaning you stretch yourself up to your full height with your back comfortably straight. If you start to slouch during the run, just take a deep breath, feel yourself naturally straighten and as you exhale simply maintain that upright position.",
			9: "While you're running, you want to lean slightly into the run versus running completely upright. That lean should come from your bodies natural shift in centre of gravity at the hips between your left and right legs moving forward.",
			10: "Your knee should be in line with the middle of your foot so that when your foot strikes the ground, it's right under your knee.",
			11: "Everyone's stride and gait is different, which is natural. The easiest way to think about your foot placement is to think about your shin being as close to perpendicular as possible when the foot hits the ground.",
			12: "While running Keep your ankle flexed as your foot rolls forward to create more force to push-off from. As you roll onto your toes, try to spring off the ground. You should feel your calf muscles propelling you forward on each step."

		}


	def on_start(self, userID):

		#print userID.data 

		self.user.set_user_id(userID.data)
		self.user.set_from_csv()

		#print self.user.name 

		user_info = C25KUserInfo()
		user_info.name = self.user.name
		user_info.user_id = self.user.user_id
		user_info.session_number = self.user.session_number
		user_info.walk_speed_pb = self.user.walk_speed_pb
		user_info.walk_speed_avg = self.user.walk_speed_avg
		user_info.run_speed_pb = self.user.run_speed_pb
		user_info.run_speed_avg = self.user.run_speed_avg
		user_info.resting_heart_rate = self.user.resting_heart_rate
		user_info.motivator = self.user.motivator
		user_info.heart_effort_threshold = self.user.heart_effort_threshold

		#TBC: if we can be sure that the other nodes will already be launched before the user selects their ID then can get rid of this extra
		#publishing step and just trigger everyting from user_id on user_start and then .set_from_csv()

		self.user_info_pub.publish(user_info)
		self.programme_state_pub.publish("WARMUP")

		self.hello_dict = {
			0: "Hello " + self.user.name + ", it's good to see you", 
			1: "Hi " + self.user.name + ", nice to see you", 
		}
		choice = random.randint(0,1) 
		print self.hello_dict[choice] + ". Today we're going to do Session " + str(self.user.session_number) + " of the Couch to 5K programme. Let's start the warm up.\
		Bring the treadmill up to a comfortable walking speed."
		self.dialogue_pub.publish(self.hello_dict[choice] + ". Today we're going to do Session " + str(self.user.session_number) + " of the Couch to 5K programme. Let's start the warm up.\
		Bring the treadmill up to a comfortable walking speed.") 


	def on_time(self, time):
		self.time = int(time.data)

		if self.time < 345:
			self.programme_state_pub.publish("WARMUP")

		#test these full-time timing points
		if self.time == 20:

			#print "So how are you doing today? Please let me know via the tablet."
			self.dialogue_pub.publish("So how are you doing today? Please let me know via the tablet.")
			self.checkmood_pub.publish("How are you doing today?") #use this to trigger correct tablet state change & display check pre 

		if self.time == 100:
			self.dialogue_pub.publish(self.warmup_dict1[self.user.session_number-1])

		if self.time == 180: #180: #40:
			#print "Ok we are half way through the warm up now." 
			self.dialogue_pub.publish("Ok we are half way through the warm up now.")

		if self.time == 250:
			choice = random.randint(0,12) 
			self.dialogue_pub.publish(self.warmup_dict2[choice])

		if self.time == 330: #330: #50:
			#print "Right, you should be nice and warm now, think about getting ready to start running."
			self.dialogue_pub.publish("Right, you should be nice and warm now, think about getting ready to start running.")

		if self.time == 345: #345: ##55 
			self.warmup_complete_pub.publish("WARMUP_COMPLETE")
			print("warm-up complete - shutting down node")
			self.trigger_warmup_complete() # tell the clock that the warm up is complete -- session time will start being published

			rospy.signal_shutdown("warmup complete")
			#self.mood_subscriber.unregister()


	def on_mood(self, mood):
		choice = random.randint(0,2)
		if mood.data == "NEUTRAL":
			print self.neutral_dict[choice]
			self.dialogue_pub.publish(self.neutral_dict[choice])
		elif mood.data == "POSITIVE":
			print self.positive_dict[choice]
			self.dialogue_pub.publish(self.positive_dict[choice])
		else: 
			print self.negative_dict[choice]
			self.dialogue_pub.publish(self.negative_dict[choice])

if __name__ == "__main__":
	rospy.init_node('warmup', disable_signals=True)
	warmup = WarmUp()
	rospy.spin()