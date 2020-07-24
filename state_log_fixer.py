import pandas as pd
import numpy as np

state_log_df = pd.read_csv('fixed_state_log.csv', names=['time', 'state_id', 'user_id', 'session_condition','self_attitude', 'expert_attitude', 'extraversion', \
	'agreeableness', 'conscientiousness', 'emotional_stability', 'openness_experience', 'activity_level', 'session_number', 'programme_state', 'task_success', \
	'session_time', 'session_time_remaining', 'normalised_session_time', 'programme_time', 'normalised_programme_time', 'time_spent_prog_action', \
	'time_remaining_prog_action', 'programme_action_progress', 'programme_action_duration', 'current_speed', 'relative_speed_average', \
	'relative_speed_best', 'time_since_last_action', 'normalised_time_since_last_action', 'heart_rate', 'relative_heart_rate', 'au12', 'au25',\
	'session_mood', 'user_run_pb', 'user_walk_pb', 'user_run_avg','user_walk_avg'])

print state_log_df.shape

print(state_log_df.head())

relevant_states = state_log_df.loc[(state_log_df['user_id']=="DP")] #& (state_log_df['session_number']==1)] #& (state_log_df['session_time']>=360) & (state_log_df['session_time']<=450)].copy() 

print relevant_states.shape
print relevant_states.head()

walk_speeds = [3.0]
run_speeds = [4.0]

walk_pb = 3.0
run_pb = 4.0

run_avg = 4.0
walk_avg = 3.0

programme_time = 0.0
counter = 0
session_number = 0
prev_session_number = 0

programme_length = 32400.0

for index, row in relevant_states.iterrows():

	# print 'index: '
	# print index

	### FIXING PROGRAMME STATE - MR-3 AND LB-9"

	# print state_log_df.loc[index,'programme_state'] 

	# state_log_df.loc[index,'programme_state'] = 0.5

	# print state_log_df.loc[index,'programme_state'] 

	### FIXING SPEED & -VE PROGRAMME DURATIONS - ALL ###

	# speed = state_log_df.loc[index,'current_speed']
	# prog_action_progress = state_log_df.loc[index,'programme_action_progress']
	# programme_state = state_log_df.loc[index,'programme_state']


	# print 'speed: '
	# print speed
	# print 'prog_action_progress: '
	# print prog_action_progress

# 	if prog_action_progress < 0:
# 		print "replacing action progress with NaN"
# 		state_log_df.loc[index,'programme_action_progress'] = float('NaN')


# 	if speed < 1 or speed > 7.9: #or 7.69 < speed < 7.71:
# 		print "replacing speed with NaNs"
# 		state_log_df.loc[index,'current_speed'] = float('NaN')
# 		state_log_df.loc[index,'relative_speed_average'] = float('NaN')
# 		state_log_df.loc[index,'relative_speed_best'] = float('NaN')
# 		state_log_df.loc[index,'user_walk_avg'] = walk_avg
# 		state_log_df.loc[index,'user_walk_pb'] = walk_pb
# 		state_log_df.loc[index,'user_run_avg'] = run_avg
# 		state_log_df.loc[index,'user_run_pb'] = run_pb

# 	else:
# 		if not np.isnan(speed):
# 			if programme_state == 0.5:
# 				if prog_action_progress <= 0.8:
# 					walk_speeds.append(speed)
# 					walk_avg = sum(walk_speeds)/len(walk_speeds)

# 					# print "new walk_avg: "
# 					# print walk_avg
					
# 					if speed > walk_pb:
# 						walk_pb = speed
# 						state_log_df.loc[index,'user_walk_pb'] = walk_pb

# 						# print "new walk_pb: "
# 						# print walk_pb

# 				state_log_df.loc[index,'relative_speed_average'] = speed/(2*walk_avg)
# 				# print "setting relative_speed_average to: "
# 				# print state_log_df.loc[index,'relative_speed_average']

# 				state_log_df.loc[index,'relative_speed_best'] = speed/(2*walk_pb)
# 				# print "setting relative_speed_best to: "
# 				# print state_log_df.loc[index,'relative_speed_best']

# 			else:
# 				if prog_action_progress <= 0.8:
# 					run_speeds.append(speed)
# 					run_avg = sum(run_speeds)/len(run_speeds)

# 					# print "new run_avg: "
# 					# print run_avg

# 					if speed > run_pb:
# 						run_pb = speed
# 						state_log_df.loc[index,'user_run_pb'] = run_pb

# 				state_log_df.loc[index,'relative_speed_average'] = speed/(2*run_avg)
# 				# print "setting relative_speed_average to: "
# 				# print state_log_df.loc[index,'relative_speed_average']

# 				state_log_df.loc[index,'relative_speed_best'] = speed/(2*run_pb)
# 				# print "setting relative_speed_best to: "
# 				# print state_log_df.loc[index,'relative_speed_best']

# 		else:
# 			print "speed was already a NaN"
# 		# 	state_log_df.loc[index,'relative_speed_average'] = float('NaN')
# 		# 	state_log_df.loc[index,'relative_speed_best'] = float('NaN

# 		state_log_df.loc[index,'user_walk_avg'] = walk_avg
# 		state_log_df.loc[index,'user_walk_pb'] = walk_pb
# 		state_log_df.loc[index,'user_run_avg'] = run_avg
# 		state_log_df.loc[index,'user_run_pb'] = run_pb

# print "final walk_avg is: " + str (walk_avg)
# print "final walk_pb is: " + str(walk_pb)
# print "final run_avg is: " + str(run_avg)
# print "final run_pb is: " + str(run_pb)

# 	# import pdb
# 	# pdb.set_trace()


	### FIXING HEART RATE - FB-1, LB-1 ###
	# print state_log_df.loc[index,'relative_heart_rate']

	# heart_rate = state_log_df.loc[index,'heart_rate']
	# user_resting_heart = 83 #check from user_csv databases 

	# if heart_rate >= (2*user_resting_heart):
	# 	relative_heart_rate = 1
	# else:
	# 	relative_heart_rate = float(heart_rate)/(2*float(user_resting_heart))

	# state_log_df.loc[index,'relative_heart_rate'] = relative_heart_rate

	# print state_log_df.loc[index,'relative_heart_rate

	### FIXING PROGRAMME TIME - GB, MR, JW, DP###

	#part two: relative programme time
	#print state_log_df.loc[index,'normalised_programme_time'] 
	state_log_df.loc[index,'normalised_programme_time'] = state_log_df.loc[index,'programme_time'] / programme_length
	#tigprint state_log_df.loc[index,'normalised_programme_time'] 

	# #part one: programme time
	# session_number = state_log_df.loc[index,'session_number']

	# if session_number != prev_session_number:
	# 	# if session_number == 6: ##hack for GB
	# 	# 	programme_time += 1260

	# 	# if session_number == 2: ##hack for JW
	# 	# 	programme_time += 1200
	# 	# if session_number == 4:
	# 	# 	programme_time += 1200

	# 	print "session_number: " + str(state_log_df.loc[index,'session_number'])
	# 	print "session_time: " +  str(state_log_df.loc[index,'session_time'])
	# 	print "programme_time: " + str(programme_time)

	# 	programme_time += state_log_df.loc[index,'session_time']
	# 	print "new programme_time is: " + str(programme_time)

	# 	prev_session_number = state_log_df.loc[index,'session_number']
	# 	prev_index = index

	# else:
	# 	programme_time += (state_log_df.loc[index,'session_time'] - state_log_df.loc[prev_index,'session_time'])
	# 	prev_index = index


	#print "new programme_time: " + str(programme_time)

	# import pdb
	# pdb.set_trace

	state_log_df.loc[index,'programme_time'] = programme_time	

#print "new programme_time: " + str(programme_time)
state_log_df.to_csv("fixed_state_log.csv",header=False, index=False)



#print(relevant_states['relative_speed_best'].mean())

#print relevant_states
#print relevant_states.session_number.unique