#!/usr/bin/env python

import datetime
import os
import numpy as np
import threading

from sklearn.neural_network import MLPClassifier
from sklearn.externals import joblib
from sklearn.model_selection import cross_val_score

class MLPLearner():

	def __init__(self, learner_type, layers, probability):

		self.threshold = probability

		# if learner_type == "ACTION":
		# 	self.threshold = 0.5 #8 actionclasses so chance = .125 -> 0.3 just looked right in quick testing for reducing false positives
		# else:
		# 	self.threshold = 0.5 #4 styles so chance = .25

		self.training_data = np.empty([1,18])
		self.labels = np.empty([1,1])
		self.rewards = np.empty([1,1])
		self.clf = MLPClassifier(solver='lbfgs', alpha=1e-5, hidden_layer_sizes=layers, activation='relu', random_state=1)

		self.train_first_example = True
		self.label_first_example = True
		self.reward_first_example = True

		self.type = learner_type
		self.lock = threading.Lock()


	def load_model(self): # , model_folder):

		models = os.listdir("./mlpmodels/"+self.type+"/")
		models.sort(reverse=True)

		clf_file = models[2] #need to check actual order here!
		training_data_file = models[0]
		labels_file = models[1]
		#rewards_file = models[1]

		self.training_data = np.load("./mlpmodels/"+self.type+"/"+training_data_file)
		print("loading training data array from: " + "./mlpmodels/"+self.type+"/"+training_data_file)
		
		self.labels = np.load("./mlpmodels/"+self.type+"/"+labels_file)
		print("loading labels data array from: " + "./mlpmodels/"+self.type+"/"+labels_file)

		# self.rewards = np.load("./mlpmodels/"+self.type+"/"+rewards_file)
		# print "loading labels data array from: " + "./mlpmodels/"+self.type+"/"+rewards_file

		self.clf = joblib.load("./mlpmodels/"+self.type+"/"+clf_file)

		# import pdb
		# pdb.set_trace()

		self.train_first_example = False
		self.label_first_example = False
		self.reward_first_example = False

	def save_model(self):
		date_label = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")

		np.save("./mlpmodels/"+self.type+"/"+date_label+"-training_data.npy",self.training_data)
		np.save("./mlpmodels/"+self.type+"/"+date_label+"-labels.npy",self.labels)
		# np.save("./mlpmodels/"+self.type+"/"+date_label+"-rewards.npy",self.rewards)

		joblib.dump(self.clf, "./mlpmodels/"+self.type+"/"+date_label+"-clf.pkl")

		print("saving model")

	def add_new_instance(self, input_data, label, reward):
		with self.lock:

			input_data = np.array([input_data])
			print("label is: ")
			print(label)

			if not np.isnan(input_data).any() and label >= 0:

				if reward == -1: #i.e. if rejected then correct action is do nothing
					label = 0

				if self.training_data.shape[0] == 1 and self.train_first_example == True:
					self.training_data[0][:] = input_data
					self.train_first_example = False
				else:
					self.training_data = np.append(self.training_data,input_data,axis=0)

				label = np.array([label])
				reward = np.array([reward])

				print self.training_data.shape
				print self.labels.shape

				print label.shape
				print reward.shape
				

				if self.labels.shape[0] == 1 and self.label_first_example == True:
					self.labels[0][:] = label
					self.label_first_example = False
				else:
					self.labels = np.append(self.labels,[label],axis=0)

				# if self.rewards.shape[0] == 1 and self.reward_first_example == True:
				# 	self.rewards[0][:] = reward
				# 	self.rewards_first_example = False
				# else:
				# 	self.rewards = np.append(self.rewards,reward,axis=0)

				#retrain MLP on every new instance
				# print "Re-training MLP"
				# self.clf.fit(self.training_data,self.labels)


			else:
				print("Input data contained nans - not adding to instance collection")

	def retrain(self):
		print("Re-training MLP")
		self.clf.fit(self.training_data,self.labels)


	def add_new_instances(self,input_data, labels, rewards):
		#logic on re-labelling -ve reward labels to be labels of do nothing (0) and passive accepts (0) to be +ve examples
		self.training_data = np.asarray(input_data)
		self.labels = np.asarray(labels)
		self.rewards = np.asarray(rewards)

		# print(self.rewards.shape)
		# print(self.training_data.shape)
		# print(self.labels.shape)

		neg_indices = np.where(self.rewards==-1)[0]
		self.labels[neg_indices] = 0
		
		#alt. throw them away (seem to be reducing accuracy a lot...)
		#self.training_data = np.delete(self.training_data, neg_indices, axis=0)
		#self.labels = np.delete(self.labels, neg_indices)

		print(self.training_data.shape)
		print(self.labels.shape)

		zero_indices = (rewards == 0)
		self.labels[zero_indices] = 1

		print(np.unique(self.labels))
		print(np.unique(self.rewards))

		self.clf.fit(self.training_data, self.labels)
		

	def predict(self, input_data):
		with self.lock:
			self.input_data = np.array([input_data]) 

			#print self.input_data.shape

			if not np.isnan(self.input_data).any():
				prediction =  self.clf.predict(self.input_data)
				print "mlp: predicted class"
				print prediction
			else:
				prediction = [0]

			return prediction[0]

			record = np.array2str(self.input_data) + ',' + \
			str(prediction) 

			with open('mlp_learner_predictions.csv', 'a') as logfile:
				logfile.write(record + "\n")
				logfile.flush()

	def predict_proba(self, input_data):
		with self.lock:
			self.input_data = np.array([input_data]) 

			#print self.input_data.shape

			if not np.isnan(self.input_data).any():
				prediction_probabilities =  self.clf.predict_proba(self.input_data)
				#print prediction_probabilities
				max_prob = np.amax(prediction_probabilities)
				if max_prob >= self.threshold:
					max_prob_index = np.argmax(prediction_probabilities)
					best_prediction = self.clf.classes_[max_prob_index]
				else:
					best_prediction = 0
			else:
				best_prediction = 0

			return best_prediction

			record = np.array2str(self.input_data) + ',' + \
			str(best_prediction) 

			with open('mlp_learner_predictions.csv', 'a') as logfile:
				logfile.write(record + "\n")
				logfile.flush()

	def predict_proba_testing(self, input_data):
		with self.lock:
			self.input_data = np.array([input_data]) 

			#print self.input_data.shape

			if not np.isnan(self.input_data).any():
				prediction_probabilities =  self.clf.predict_proba(self.input_data)
				#print prediction_probabilities
				probability = np.amax(prediction_probabilities)
				max_prob_index = np.argmax(prediction_probabilities)
				best_prediction = self.clf.classes_[max_prob_index]

			else:
				best_prediction = 0
				probability = 0

			return [best_prediction, probability]

			record = np.array2str(self.input_data) + ',' + \
			str(best_prediction) 

			with open('mlp_learner_predictions.csv', 'a') as logfile:
				logfile.write(record + "\n")
				logfile.flush()

	def cross_val_scores(self):
		return cross_val_score(self.clf, self.input_data, self.labels, cv=3)