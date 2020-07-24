import pandas as pd
import numpy as np
import csv

from knnlearner import KNNLearner
from mlplearner import MLPLearner
from _5_c25klearner import C25KLearner
from c25k_msgs.msg import C25KAction, C25KActionType, C25KActionStyle, C25KTraining, C25KState, ValidatedC25KAction

ACTION="ACTION"
STYLE="STYLE"

TIME = "TIME"
SOCIAL = "SOCIAL"
TASK = "TASK"
REWARD = "REWARD"
ANIMATION = "ANIMATION"
GET_CLOSER = "GET_CLOSER"
BACK_OFF = "BACK_OFF" 
CHECKPRE = "CHECKPRE"
NONE = "NONE"

class LearnerTestBed():

	def __init__(self, learner, learner_type):

		self.test_inputs = []
		self.test_labels = []
		self.test_rewards = []
		self.number_tests = 0

		self.train_inputs = []
		self.train_labels = []
		self.train_rewards = []
		self.number_examples=0

		self.learner=learner
		self.learner_type = learner_type

		self.style_dict = {
			0: C25KActionStyle.NEUTRAL,
			1: C25KActionStyle.CHALLENGING,
			2: C25KActionStyle.POSITIVE,
			3: C25KActionStyle.SYMPATHETIC,
			4: NONE,
			C25KActionStyle.NEUTRAL: 0,
	        C25KActionStyle.CHALLENGING: 1,
			C25KActionStyle.POSITIVE: 2,
			C25KActionStyle.SYMPATHETIC: 3,
			NONE: 4
		}

		self.action_dict = {
			0: NONE,
			1: TIME,
			2: TASK,
			3: REWARD,
			4: ANIMATION,
			5: GET_CLOSER,
			6: BACK_OFF,
			7: CHECKPRE,
			8: SOCIAL,
			NONE: 0,
			TIME: 1,
			TASK: 2,
			REWARD: 3,
			ANIMATION: 4,
			GET_CLOSER: 5,
			BACK_OFF: 6,
			CHECKPRE: 7,
			SOCIAL: 8
		}


	def prepare_training_data(self, training_data):

		stored_data = pd.read_csv(training_data, names=['action-id','label','s1','s2','s3','s4','s5','s6','s7','s8','s9','s10', \
		's11','s12','s13','s14','s15','s16','s17','s18','reward']) 

		stored_data = stored_data.dropna(axis=0) #TBC whether want to do something cleverer than this
		
		full_df = stored_data[['label','s1','s2','s3','s4','s5','s6','s7','s8','s9','s10', \
		's11','s12','s13','s14','s15','s16','s17','s18','reward']]

		test_df = full_df.sample(frac=0.3)

		test_array = test_df.values
		#print test_array.shape
		self.test_labels = test_array[:,[0]]
		if self.learner_type == ACTION:
			self.test_labels = np.vectorize(self.action_dict.get)(self.test_labels)
		elif self.learner_type == STYLE:
			self.test_labels = np.vectorize(self.style_dict.get)(self.test_labels)
		self.test_labels = self.test_labels.tolist()
		self.number_tests = len(self.test_labels)

		self.test_inputs = test_array[:,[1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18]]
		self.test_inputs = self.test_inputs.tolist()
		
		self.test_rewards = test_array[:,[19]]
		self.test_rewards = self.test_rewards.tolist()
		
		train_df = full_df.drop(test_df.index)
		train_array = train_df.values
		#print train_array.shape

		self.train_labels = train_array[:,[0]]
		if self.learner_type == ACTION:
			self.train_labels = np.vectorize(self.action_dict.get)(self.train_labels)
		elif self.learner_type == STYLE:
			self.train_labels = np.vectorize(self.style_dict.get)(self.train_labels)
		self.train_labels = self.train_labels.tolist()
		#print self.train_labels

		self.number_examples = len(self.train_labels)

		self.train_inputs = train_array[:,[1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18]]
		self.train_inputs = self.train_inputs.tolist()

		self.train_rewards = train_array[:,[19]]
		self.train_rewards = self.train_rewards.tolist()


	def train_model(self):
		counter = 0
		# example_input = self.train_inputs[1]
		# print type(example_input)
		# print example_input

		# example_label = self.train_labels[1][0]
		# #example_label = example_label[0]
		# print type(example_label)
		# print example_label

		# example_reward = self.train_rewards[1][0]
		# #example_reward=example_reward[0]
		# print type(example_reward)
		# print example_reward

		while counter < self.number_examples:
			example_input = self.train_inputs[counter]
			example_label = self.train_labels[counter][0]
			example_reward = self.train_rewards[counter][0]

			if self.learner_type == ACTION:
				self.learner.action_learner.add_new_instance(example_input,example_label,example_reward)
			elif self.learner_type == STYLE:
				self.learner.style_learner.add_new_instance(example_input,example_label,example_reward)

			counter+=1


	def predict(self):
		predicted_output = []
		actual_output = []
		counter = 0

		while counter < self.number_tests:
			test_input = self.test_inputs[counter]
			if self.learner_type == ACTION:
				predicted_output.append(self.learner.action_learner.predict(test_input))
			elif self.learner_type == STYLE:
				predicted_output.append(self.learner.style_learner.predict(test_input))
			actual_output.append(self.test_labels[counter][0])
			counter+=1

		index = 0
		performance = []

		while index < len(predicted_output):
			# print "prediction was: "
			# print predicted_output[index]
			# print "actual output was: "
			# print actual_output[index]

			if predicted_output[index] == actual_output[index]:
				performance.append(1) 
			else:
				if predicted_output[index] == 0: #i.e. couldn't predict anything
					performance.append(0)
				else:
					performance.append(-1) #i.e. predicted something different
			index+=1

		print "Number of successful predictions: "
		print performance.count(1)

		print "Number of times no prediction returned: "
		print performance.count(0)

		print "Number of times prediction returned != actual output: "
		print performance.count(-1)


if __name__ == '__main__':
	test_type = STYLE
	# style_knnlearner = MLPLearner(STYLE,(40,20,20), 0.5)
	# action_knnlearner = MLPLearner(ACTION,(40,20,20), 0.5)
	style_knnlearner = KNNLearner(STYLE)
	action_knnlearner = KNNLearner(ACTION)	
	learner = C25KLearner(style_knnlearner,action_knnlearner)
	testbed = LearnerTestBed(learner, test_type)

	# if test_type == STYLE:
	# 	testbed.learner.style_learner.load_model()
	# elif test_type == ACTION:
	# 	testbed.learner.action_learner.load_model()

	if test_type == ACTION:
		testbed.prepare_training_data('new_action_instances_noface.csv')
	elif test_type == STYLE:
		testbed.prepare_training_data('new_style_instances_noface.csv')

	testbed.train_model()

	# if test_type == ACTION:
	# 	testbed.learner.action_learner.retrain()
	# elif test_type == STYLE:
	# 	testbed.learner.style_learner.retrain()

	#testbed.predict()

	if test_type == STYLE:
		testbed.learner.style_learner.save_model()
	elif test_type == ACTION:
		testbed.learner.action_learner.save_model()
