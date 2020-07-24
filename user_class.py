#!/usr/bin/env python

import csv

class User():
	def __init__(self):
		self.name = ""
		self.age = 0
		self.user_id = ""
		self.session_number = 0
		self.self_attitude = 0.0
		self.expert_attitude = 0.0
		#self.personality = [0.0, 0.0, 0.0, 0.0]
		self.extraversion = 0.0
		self.agreeableness = 0.0
		self.conscientiousness = 0.0
		self.emotional_stability = 0.0
		self.openness_experience = 0.0
		self.activity_level = 0.0
		self.programme_time = 0
		self.walk_speed_pb = 0.0
		self.walk_speed_avg = 0.0
		self.run_speed_pb = 0.0
		self.run_speed_avg = 0.0
		self.resting_heart_rate = 0
		self.motivator = ''
		self.heart_effort_threshold = 0

	def set_user_id(self, user_id): #think this is unneccessary and can just set straight from script using class in...
		self.user_id = user_id

	def set_from_csv(self):
		with open('user_database.csv') as csvDataFile:
			user_database = csv.reader(csvDataFile)
			for user_data in user_database:
				if user_data[1] == self.user_id:
					self.name = user_data[0]
					self.age = user_data[18]
					self.session_number = int(user_data[2])
					self.self_attitude = float(user_data[3])
					self.expert_attitude = float(user_data[4])
					#self.personality = [float(user_data[5]), float(user_data[6]), float(user_data[7]), float(user_data[8]), float(user_data[9])]
					self.extraversion = float(user_data[5])
					self.agreeableness = float(user_data[6])
					self.conscientiousness = float(user_data[7])
					self.emotional_stability = float(user_data[8])
					self.openness_experience = float(user_data[9])
					self.activity_level = float(user_data[10])
					self.programme_time = float(user_data[11])
					self.walk_speed_pb = float(user_data[12])
					self.walk_speed_avg = float(user_data[13])
					self.run_speed_pb = float(user_data[14])
					self.run_speed_avg = float(user_data[15])
					self.resting_heart_rate = int(user_data[16])
					self.motivator = user_data[17]
					self.heart_effort_threshold = int(0.8*(220-int(self.age))) #80% of maximum heart rate estimated at (220-age)bpm note this isn't stored in user class
					#...

	# def get_name(self): #TBC: almost certainly don't need these as always working with a User() object
	# 	return self.name

	# def get_user_id(self):
	# 	return self.user_id

	# def get_session(self):
	# 	return self.session

	# def get_self_attitude(self):
	# 	return self.self_attitude

	# def get_expert_attitude(self):
	# 	return self.expert_attitude

	# def get_personality(self):
	# 	return self.personality

	# def get_extraversion(self):
	# 	return self.extraversion

	# def get_agreeableness(self):
	# 	return self.agreeableness

	# def get_conscientiousness(self):
	# 	return self.conscientiousness

	# def get_emotional_stability(self):
	# 	return self.emotional_stability

	# def get_openness_experience(self):
	# 	return self.openness_experience

	# def get_activity_level(self):
	# 	return self.activity_level

	# def get_programme_time(self):
	# 	return self.programme_time

	# def get_walk_pb(self):
	# 	return self.walk_speed_pb

	# def get_walk_avg(self):
	# 	return self.walk_speed_avg

	# def get_run_pb(self):
	# 	return self.run_speed_pb

	# def get_run_avg(self):
	# 	return self.run_speed_avg

	# def get_resting_heart(self):
	# 	return self.resting_heart_rate

	def update_csv(self, updated_record): #tested - this works! 
		with open('user_database.csv', 'r') as csvDataFile:
			user_database_reader = csv.reader(csvDataFile)
			user_database = list(user_database_reader)
			index = 0
			for user_data in user_database:
				if user_data[1] == self.user_id:
					user_index = index
					index +=1
				else:
					index += 1
			user_database[user_index] = updated_record
		with open('user_database.csv', 'w') as csvDataFile:
			user_database_writer = csv.writer(csvDataFile)
			user_database_writer.writerows(user_database)

		csvDataFile.close()