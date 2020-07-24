import pandas as pd
import numpy as np
import csv

from knnlearner import KNNLearner
from mlplearner import MLPLearner
from itertools import permutations 
from _5_c25klearner import C25KLearner
from c25k_msgs.msg import C25KAction, C25KActionType, C25KActionStyle, C25KTraining, C25KState, ValidatedC25KAction

from sklearn.metrics import accuracy_score
from sklearn.metrics import f1_score
from sklearn.metrics import recall_score
from sklearn.metrics import precision_score

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

		self.predicted_output = []

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

		#print train_array.shape
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
		
		train_df = full_df#.drop(test_df.index) #use all data when actually generating initial models
		train_array = train_df.values

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


	def user_split_testing(self, training_data):

		stored_data = pd.read_csv(training_data, names=['user_id', 'action-id','label','s1','s2','s3','s4','s5','s6','s7','s8','s9','s10', \
		's11','s12','s13','s14','s15','s16','s17','s18','s19','s20','reward']) 

		print stored_data.head()

		stored_data = stored_data.dropna(axis=0) #TBC whether want to do something cleverer than this
		
		full_df = stored_data[['label','s1','s2','s3','s4','s5','s6','s7','s8','s9','s10', \
		's11','s12','s13','s14','s15','s16','s17','s18','s19','s20','reward']]

		user_list = ["GB","MR","JF","JW","DP","DB","MB","PT","LB","FB"]

		perm = permutations(user_list,3)

		trial = 1
		list_scores = []

		for subset in perm:
			train_users = ["GB","MR","JF","JW","DP","DB","MB","PT","LB","FB"]
			test_users = subset
			print test_users

			for user in test_users:
				print user
				train_users.remove(user)

			print train_users
			test_df = stored_data[(stored_data['user_id']==test_users[0]) | (stored_data['user_id']==test_users[1]) | (stored_data['user_id']==test_users[2])]

			test_array_df = test_df[['label','s1','s2','s3','s4','s5','s6','s7','s8','s9','s10', \
			's11','s12','s13','s14','s15','s16','s17','s18','s19','s20','reward']]
			test_array = test_array_df.values

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
			
			train_df = stored_data.drop(test_df.index)
			train_df = train_df[['label','s1','s2','s3','s4','s5','s6','s7','s8','s9','s10', \
			's11','s12','s13','s14','s15','s16','s17','s18','s19','s20','reward']]
			train_array = train_df.values

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

			if test_type == STYLE:
				self.learner.style_learner.add_new_instances(self.train_inputs, self.train_labels, self.train_rewards)
			elif test_type == ACTION:
				self.learner.action_learner.add_new_instances(self.train_inputs, self.train_labels, self.train_rewards)

			if test_type == STYLE:
				self.predict()
				# scores = testbed.learner.style_learner.clf.score(self.test_inputs,self.test_labels)
				# print  "Scores on test: " + str(trial) 
				# print scores
				# list_scores.append(scores)
			elif test_type == ACTION:
				self.predict()
				# scores = testbed.learner.action_learner.clf.score(self.test_inputs,self.test_labels)
				# print "Scores on test: " + str(trial) 
				# print scores
				# list_scores.append([trial, scores])

			trial+=1 

		print "Overall list of scores:"
		print list_scores

	def train_model(self):
		counter = 0
		while counter < self.number_examples:
			example_input = self.train_inputs[counter]
			example_label = self.train_labels[counter][0]
			example_reward = self.train_rewards[counter][0]

			if self.learner_type == ACTION:
				self.learner.action_learner.add_new_instance(example_input,example_label,example_reward)
			elif self.learner_type == STYLE:
				self.learner.style_learner.add_new_instance(example_input,example_label,example_reward)

			counter+=1

	def train_model_array(self): 
		if self.learner_type == ACTION:
			self.learner.action_learner.add_new_instances(self.train_inputs,self.train_labels,self.train_rewards)
			# print "CLF Score: "
			# print self.learner.action_learner.clf.score(self.test_inputs,self.test_labels)
			#self.learner.action_learner.save_model()
		elif self.learner_type == STYLE:
			self.learner.style_learner.add_new_instances(self.train_inputs,self.train_labels,self.train_rewards)
			# print "CLF Score: "
			# print self.learner.style_learner.clf.score(self.test_inputs,self.test_labels)
			#self.learner.style_learner.save_model()


	def predict(self):
		actual_output = []
		counter = 0
		correct_probabilities=[]
		incorrect_probabilities=[]
		probability=[]

		while counter < self.number_tests:
			test_input = self.test_inputs[counter]
			if self.learner_type == ACTION:
				# output_prob = self.learner.action_learner.predict_proba_testing(test_input)
				# predicted_output.append(output_prob[0])
				self.predicted_output.append(self.learner.action_learner.predict_proba(test_input)) #testing for bugs linked to predict_proba
				# probability.append(output_prob[1])
			elif self.learner_type == STYLE:
				# output_prob = self.learner.style_learner.predict_proba_testing(test_input)
				# predicted_output.append(output_prob[0])
				self.predicted_output.append(self.learner.style_learner.predict_proba(test_input))
				#probability.append(output_prob[1])
			actual_output.append(self.test_labels[counter][0])
			counter+=1

		index = 0
		performance = []
		action_action_counter = 0

		while index < len(self.predicted_output):
			# print "prediction was: "
			# print predicted_output[index]
			# print "actual output was: "
			# print actual_output[index]

			if self.predicted_output[index] == actual_output[index]:
				performance.append(1) 
				if self.predicted_output[index] != 0:
					action_action_counter +=1
				#correct_probabilities.append(probability[index])
			else:
				if self.predicted_output[index] == 0: #i.e. couldn't predict anything
					performance.append(0)
				else:
					performance.append(-1) #i.e. predicted something different
					#incorrect_probabilities.append(probability[index])
			index+=1

		print "Number of successful predictions: "
		print performance.count(1)

		print "Number of times no prediction returned: "
		print performance.count(0)

		print "Number of times prediction returned != actual output: "
		print performance.count(-1)

		record = str(performance.count(1)) + ',' + \
		str(performance.count(0)) + ',' + \
		str(performance.count(-1))

		#data logging
		# with open('performance_logger.csv', 'a') as logfile:
		# 	logfile.write(record + "\n")
		# 	logfile.flush()

		# print "Number of actual action predictions: "
		# print action_action_counter

		# print "Average probability for correct predictions: "
		# print sum(correct_probabilities)/len(correct_probabilities)

		# print "Average probability for incorrect predictions: "
		# print sum(incorrect_probabilities)/len(incorrect_probabilities)

	def metrics(self):
		self.predict()
		accuracy = accuracy_score(self.test_labels, self.predicted_output)
		f_score = f1_score(self.test_labels, self.predicted_output, average='micro')
		recall = recall_score(self.test_labels, self.predicted_output, average='micro')
		precision = precision_score(self.test_labels, self.predicted_output, average='micro')

		print("accuracy: " + str(accuracy))
		print("f_score: " + str(f_score))
		print("recall: " + str(recall))
		print("precision: " + str(precision))

		record = str(accuracy) + ',' + \
		str(f_score) + ',' + \
		str(recall) + ',' + \
		str(precision)

		# #data logging
		# with open('metrics_logger.csv', 'a') as logfile:
		# 	logfile.write(record + "\n")
		# 	logfile.flush()		

if __name__ == '__main__':
	test_type = STYLE
	# style_learner = MLPLearner(STYLE, (40,20,20), 0.5)
	# action_learner = MLPLearner(ACTION, (40,20,20), 0.5)
	style_learner = KNNLearner(STYLE)
	action_learner = KNNLearner(ACTION)	
	learner = C25KLearner(style_learner,action_learner)
	testbed = LearnerTestBed(learner, test_type)

	if test_type == STYLE:
		testbed.learner.style_learner.load_model()
	elif test_type == ACTION:
		testbed.learner.action_learner.load_model()

	# import pdb
	# pdb.set_trace()

	# if test_type == ACTION:
	# 	testbed.user_split_testing('user_action_instances.csv')
	# elif test_type == STYLE:
	# 	testbed.user_split_testing('user_style_instances.csv')

	if test_type == ACTION:
		testbed.prepare_training_data('new_action_instances_noface.csv')
	elif test_type == STYLE:
		testbed.prepare_training_data('new_style_instances_noface.csv')

	testbed.train_model()
	
	# #testbed.metrics()

	# # if test_type == STYLE:
	# # 	print(testbed.learner.style_learner.cross_val_scores())
	# # elif test_type == ACTION:
	# # 	print(testbed.learner.action_learner.cross_val_scores())

	if test_type == STYLE:
		testbed.learner.style_learner.save_model()
	elif test_type == ACTION:
		testbed.learner.action_learner.save_model()