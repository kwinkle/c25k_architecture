#!/usr/bin/env python

import csv
import rospy
import c25k_msgs

from std_msgs.msg import String
from c25k_msgs.msg import C25KActionType, C25KActionStyle, ValidatedC25KAction, C25KTraining, C25KState
from Queue import Queue

#import learner

class Feedback_Moderator():
	def __init__(self):
		rospy.init_node('feedback_moderator')

		rospy.Subscriber('learning_actions', ValidatedC25KAction, self.on_validated_action)
		rospy.Subscriber('selected_features', String, self.on_feature_selection) #TO-DO: appropriate message type - 0/1 mask vector matching C25KState
		rospy.Subscriber('state', C25KState, self.on_state) 

		self.style_learning_pub = rospy.Publisher('style_learning_examples', C25KTraining, queue_size = 1)
		self.action_learning_pub = rospy.Publisher('action_learning_examples', C25KTraining, queue_size = 1)

		#TO-DO: Dictionary of feature masks for each action type (if it's appropriate to have different masks types)

		self.state = C25KState() 

	def on_state(self, state):
		self.state = state
		#TBC: any additional feature selection here? or complex logic on dealing with refusals? if not, can probably collapse this into the learner itself...

	def on_learning_action(self, learning_action):
		#all actions go through this, regardless of origin, so need to add if logic to avoid learning from e.g. control actions...
		learning_instance = C25KTraining()

		if validated_action.action in [C25KActionType.WALK, C25KActionType.RUN]: #programme actions teach style/mood
			##TO DO!! ADD SOMETHING HERE TO DISTINGUISH BETWEEN VALIDATION = 'REFUSED' ACTIONS GENERATED WHEN THE TEACHER CHOOSES A DIFFERENT STYLE TO THAT SUGGESTED BY THE LEARNER
			##AND ACTUALLY VALIDATED ACTIONS WHICH HAVE THE CORRECT STYLE (WHETHER IT WAS SUGGESTED BY LEARNER OR BY TEACHER DOESN'T MATTER...)
			learning_instance.type = ValidatedC25KAction.type.STYLE 
			learning_instance.action = validated_action.action
			learning_instance.style = validated_action.style #
			#learning_instance.learning_state = self.state #for learning purposes, take the latest state for which this action was deemed valid ##THIS IS NOW DONE IN THE ACTION MODERATOR!##
			print "publishing programme related style learning instance to learner"

		else: #all other actions are engagement actions and can teach action and (in some cases) style 
			#WHAT DO WE WANT TO GIVE LEARNER - STATE THAT TRIGGERED THIS SUGGESTION (IN CASE OF SUGGESTION) OR STATE AT WHICH IT WAS VALIDATED...?
			#I GUESS YOU TAKE LATEST AND THEN IT CAN BE FED BACK IN TO LEARNER AS A LEARNING EXAMPLE (EVEN THOUGH IT WAS SELF-SUGGESTED - INTRINSIC REINFORCEMENT/REWARD LOOP...?)
			if validated_action.validation != ValidatedC25KAction.validation.REFUSED:
				learning_instance.type = ValidatedC25KAction.type.ACTION
				learning_instance.action = validated_action.action
				learning_instance.style = validated_action.style 
				print "publishing encouragement action learning instance to learner"
				self.action_learning_pub(learning_instance)
				
				if ValidatedC25KAction.type.action in [C25KActionType.CHALLENGE]: #TO-DO: add any engagement actions from which we can infer style/mood too
					learning_instance.type = ValidatedC25KAction.type.STYLE
					print "publishing encouragement action related style learning instance to learner"
					self.style_learning_pub(learning_instance) 
			else:
			#TBC: a refused action should also be fed back to learner via reward mechanism?
				pass

if __name__ == "__main__":
	rospy.init_node("feedback_moderator")
	rospy.spin()