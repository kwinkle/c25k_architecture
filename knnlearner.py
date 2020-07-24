#!/usr/bin/env python

import datetime
import os
import numpy as np
import threading

from scipy.spatial import distance

class KNNLearner():

	def __init__(self, learner_type):

		self.threshold = 0.85
		self.training_data = np.empty([1,18])
		self.rewards = np.empty([1,1])
		self.labels = np.empty([1,1])

		self.threshold_factor_decrease = 10
		self.threshold_factor_increase = 5

		self.train_first_example = True
		self.label_first_example = True
		self.reward_first_example = True

		self.type = learner_type
		self.lock = threading.Lock()


	def load_model(self): # , model_folder):

		models = os.listdir("./knnmodels/"+self.type+"/")
		models.sort(reverse=True)
		training_data_file = models[0]
		threshold_file = models[1]
		rewards_file = models[2]
		labels_file = models[3]

		self.training_data = np.load("./knnmodels/"+self.type+"/"+training_data_file)
		print("loading training data array from: " + "./knnmodels/"+self.type+"/"+training_data_file)
		
		self.labels = np.load("./knnmodels/"+self.type+"/"+labels_file)
		print("loading labels data array from: " + "./knnmodels/"+self.type+"/"+labels_file)
		
		self.rewards = np.load("./knnmodels/"+self.type+"/"+rewards_file)
		print("loading rewards data array from: " + "./knnmodels/"+self.type+"/"+rewards_file)
		
		#self.threshold = np.load("./knnmodels/"+self.type+"/"+threshold_file)
		#print("loading threshold array from: " + "./knnmodels/"+self.type+"/"+threshold_file)

		# import pdb
		# pdb.set_trace()

		self.train_first_example = False
		self.label_first_example = False
		self.reward_first_example = False

		print("size of training data for " + self.type + ": ")
		print(self.training_data.shape)

	def save_model(self):
		date_label = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")

		np.save("./knnmodels/"+self.type+"/"+date_label+"-training_data.npy",self.training_data)
		np.save("./knnmodels/"+self.type+"/"+date_label+"-labels.npy",self.labels)
		np.save("./knnmodels/"+self.type+"/"+date_label+"-rewards.npy",self.rewards)
		np.save("./knnmodels/"+self.type+"/"+date_label+"-threshold.npy",self.threshold)

		print("saving model")

	def update_threshold(self, input_data, label, reward):
		#locate relevant stored training_data
		relevant_indices = np.argwhere(self.labels == label)
		relevant_array = self.training_data[relevant_indices[0]]
		similarity = np.array([1]) - (np.linalg.norm(input_data-self.training_data,axis=1,keepdims=True)/18)
		max_similarity = np.amax(similarity)

		# print "max similarity is: "
		# print max_similarity

		# import pdb
		# pdb.set_trace()

		if reward > 0 and max_similarity < self.threshold:
			self.threshold = self.threshold - ((self.threshold - max_similarity)/self.threshold_factor_decrease)
			print("new threshold is: ")
			print(self.threshold)
		elif reward < 0 and max_similarity > self.threshold:
			self.threshold = self.threshold + ((max_similarity - self.threshold)/self.threshold_factor_increase)
			print("new threshold is: ")
			print(self.threshold)

	def add_new_instances(self,input_data, labels, rewards):
		#logic on re-labelling -ve reward labels to be labels of do nothing (0) and passive accepts (0) to be +ve examples
		self.training_data = np.asarray(input_data)
		self.labels = np.asarray(labels)
		self.rewards = np.asarray(rewards)			
		

	def add_new_instance(self, input_data, label, reward):

		#THIS NEEDS TO GO IN A LOCK 

		with self.lock:

			input_data = np.array([input_data])

			if not np.isnan(input_data).any():

				#update dynamic threshold - action should have been suggested (or not) by learner [nb this potentially increases confidence based on validated actions *as well as* unprompted actions?]
				if label in self.labels[0] and reward !=0 : 
					print("calling update threshold")
					self.update_threshold(input_data, label, reward)

				print("adding a new instance to database")

				if self.training_data.shape[0] == 1 and self.train_first_example == True:
					self.training_data[0][:] = input_data
					self.train_first_example = False
				else:
					self.training_data = np.append(self.training_data,input_data,axis=0)

				label = np.array([[label]])
				#print label

				if self.labels.shape[0] == 1 and self.label_first_example == True:
					self.labels[0][:] = label
					self.label_first_example = False
				else:
					self.labels = np.append(self.labels,label,axis=0)

				reward = np.array([[reward]])
				if self.rewards.shape[0] == 1 and self.reward_first_example == True:
					self.rewards[0][:] = reward
					self.reward_first_example = False
				else:
					self.rewards = np.append(self.rewards,reward,axis=0)

				#self.save_model()

			else:
				print("Input data contained nans - not adding to instance collection")

				

				#print self.training_data
		

	def predict(self, input_data):

		with self.lock:
			self.input_data = np.array(input_data) 

			if not np.isnan(self.input_data).any():
				input_distance = (np.linalg.norm(input_data-self.training_data,axis=1,keepdims=True))/18 
				#euc_distance = np.apply_along_axis(self.euclidean_distance, axis=1, arr=self.training_data) #[this returns same as above but wrong structure?]

				# import pdb
				# pdb.set_trace()

				similarity = np.array([1]) - input_distance
				#print similarity #NOTE: similarities all look really high! Most > 0.9...
				expected_reward = np.multiply(similarity,self.rewards)
				max_reward = np.amax(expected_reward)


				# import pdb
				# pdb.set_trace()

				print("max reward is: ")
				print(max_reward)

				if max_reward > self.threshold:
					max_index = np.where(expected_reward == np.amax(expected_reward))
					# print "max_index is: "
					# print max_index
					# print type(max_index)
					if type(max_index) == tuple: #i.e. more than one state has same expected reward
						#print "if tuple triggered"
						max_index = max_index[0]
						if len(max_index) > 1:
							print("if len > 1 triggered")
							max_index = max_index[0]
					# print "max_index is: "
					# print max_index
					prediction = self.labels[max_index].item()
					return int(prediction)
				else:
					return 0 #0 = no action suggestion
			else:
				return 0

			record = np.array2str(self.input_data) + ',' + \
			str(max_reward) 

			with open('knn_learner_predictions.csv', 'a') as logfile:
				logfile.write(record + "\n")
				logfile.flush()

	def euclidean_distance(self, training_data):
		return distance.euclidean(training_data, self.input_data)/18 #20=number of features that distance covers