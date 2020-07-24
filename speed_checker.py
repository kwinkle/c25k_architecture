import pandas as pd

state_log_df = pd.read_csv('state_logger.csv', names=['time', 'state_id', 'user_id', 'session_condition','self_attitude', 'expert_attitude', 'extraversion', \
	'agreeableness', 'conscientiousness', 'emotional_stability', 'openness_experience', 'activity_level', 'session_number', 'programme_state', 'task_success', \
	'session_time', 'session_time_remaining', 'normalised_session_time', 'programme_time', 'normalised_programme_time', 'time_spent_prog_action', \
	'time_remaining_prog_action', 'programme_action_progress', 'programme_action_duration', 'current_speed', 'relative_speed_average', \
	'relative_speed_best', 'time_since_last_action', 'normalised_time_since_last_action', 'heart_rate', 'relative_heart_rate', 'au12', 'au25',\
	'session_mood', 'user_run_pb', 'user_walk_pb', 'user_run_avg','user_walk_avg'])

print state_log_df.shape

print(state_log_df.head())

# relevant_states = state_log_df.loc[(state_log_df['user_id']=="LB") & (state_log_df['session_number']==10) & (state_log_df['session_time']>=720) & (state_log_df['session_time']<=900)].copy() 

# print relevant_states.shape
# print relevant_states.head()

number_issues=0

for index, row in state_log_df.iterrows():

	if index > 0:
		current_speed = state_log_df.loc[index,'current_speed']
		prev_speed = state_log_df.loc[index-1,'current_speed']
		difference = abs(current_speed - prev_speed)

		if difference > 0.2:
			print "found something off at index: " + str(index)
			print "current speed: " + str(current_speed)
			print "prev_speed " + str(prev_speed)
			number_issues +=1



	# timestamp = row['time']
	# state_id = row['state_id']
	# user_id = row['user_id']
	# session_condition = row['session_condition']
	# self_attitude = row['self_attitude']
	# expert_attitude = row['expert_attitude']
	# extraversion = row['extraversion']
	# agreeableness = row['agreeableness']
	# conscientiousness = row['conscientiousness']
	# emotional_stability = row['emotional_stability']
	# openness_experience = row['openness_experience']
	# activity_level = row['activity_level']
	# session_number = row['session_number']

	# state_log_df.loc[index,'programme_state'] = 1.0

	# print state_log_df.loc[index,'programme_state']

	# task_success = row['task_success']
	# session_time = row['session_time']
	# session_time_remaining = row['session_time_remaining']
	# normalised_session_time = row['normalised_session_time']
	# programme_time = row['programme_time']
	# normalised_programme_time = row['normalised_programme_time']
	# time_spent_prog_action = row['time_spent_prog_action']
	# time_remaining_prog_action = row['time_remaining_prog_action']
	# programme_action_progress = row['programme_action_progress']
	# programme_action_duration = row['programme_action_duration']
	# current_speed = row['current_speed']
	# relative_speed_average = row['relative_speed_average']
	# relative_speed_best = row['relative_speed_best']
	# time_since_last_action = row['time_since_last_action'] 
	# normalised_time_since_last_action = row['normalised_time_since_last_action']
	# heart_rate = row['heart_rate']
	# relative_heart_rate = row['relative_heart_rate']
	# au12 = row['au12']
	# au25 = row['au25']
	# session_mood = row['session_mood']
	# user_run_pb = row['user_run_pb']
	# user_walk_pb = row['user_walk_pb']
	# user_run_avg = row['user_run_avg']
	# user_walk_avg = row['user_walk_avg']


# state_log_df.to_csv("testing_statelog_fixer.csv",header=False, index=False)


print "overall potential dodgy states: " + str(number_issues)
print "(out of: " + str(state_log_df.shape) + ")"

#print(relevant_states['relative_speed_best'].mean())

#print relevant_states
#print relevant_states.session_number.unique