#import data
import pandas as pd

action_log_df = pd.read_csv('action_logger.csv', names=['user_id','session_number','action_id','validation_timestamp','validation_state_id',\
	'action','style','duration','validation','trigger_timestamp','trigger_state_id'])

print(action_log_df.head())

state_log_df = pd.read_csv('state_logger.csv', names=['time', 'state_id', 'user_id', 'session_condition','self_attitude', 'expert_attitude', 'extraversion', \
	'agreeableness', 'conscientiousness', 'emotional_stability', 'openness_experience', 'activity_level', 'session_number', 'programme_state', 'task_success', \
	'session_time', 'session_time_remaining', 'normalised_session_time', 'programme_time', 'normalised_programme_time', 'time_spent_prog_action', \
	'time_remaining_prog_action', 'programme_action_progress', 'programme_action_duration', 'current_speed', 'relative_speed_average', \
	'relative_speed_best', 'time_since_last_action', 'normalised_time_since_last_action', 'heart_rate', 'relative_heart_rate', 'au12', 'au25',\
	'session_mood', 'user_run_pb', 'user_walk_pb', 'user_run_avg','user_walk_avg'])

print(state_log_df.head())


#note blacklisted sessions
blacklisted = {
	"MR": [3],
	"LB": [9],
	"MB": [3]
}

for index, row in state_log_df.iterrows():
	state_id = state_log_df.loc[index,'state_id']
	if not state_id in action_log_df['validation_timestamp']:
		action_id = "NO ACTION"
		action = "NONE"
		actionclass = "NONE"
		validation = "VALIDATED"

		validation_state_id = state_id
		user_id = state_log_df.loc[index,'user_id']
		self_attitude = state_log_df.loc[index,'self_attitude']
		expert_attitude = state_log_df.loc[index,'expert_attitude']
		extraversion = state_log_df.loc[index,'extraversion']
		agreeableness = state_log_df.loc[index,'agreeableness'] 
		conscientiousness =state_log_df.loc[index,'conscientiousness']
		emotional_stability = state_log_df.loc[index,'emotional_stability'] 
		openness_experience = state_log_df.loc[index,'openness_experience'] 
		activity_level = state_log_df.loc[index,'activity_level']
		programme_state = state_log_df.loc[index,'programme_state'] 
		normalised_session_time = state_log_df.loc[index,'normalised_session_time']
		normalised_programme_time = state_log_df.loc[index,'normalised_programme_time']
		programme_action_progress = state_log_df.loc[index,'programme_action_progress']
		programme_action_duration = state_log_df.loc[index,'programme_action_duration']
		relative_speed_average = state_log_df.loc[index,'relative_speed_average']
		relative_speed_best = state_log_df.loc[index,'relative_speed_best']
		normalised_time_since_last_action = state_log_df.loc[index,'normalised_time_since_last_action']
		relative_heart_rate = state_log_df.loc[index,'relative_heart_rate']
		au12 = state_log_df.loc[index,'au12'] 
		au25 = state_log_df.loc[index,'au25']
		session_mood = state_log_df.iloc[0]['session_mood']

		reward = 1
		
		action_record = action_id  + ',' + \
		actionclass  + ',' + \
		str(self_attitude)  + ',' + \
		str(expert_attitude)  + ',' + \
		str(extraversion)  + ',' + \
		str(agreeableness)  + ',' + \
		str(conscientiousness)  + ',' + \
		str(emotional_stability)  + ',' + \
		str(openness_experience)  + ',' + \
		str(activity_level)  + ',' + \
		str(programme_state)  + ',' + \
		str(normalised_session_time)  + ',' + \
		str(normalised_programme_time)  + ',' + \
		str(programme_action_progress)  + ',' + \
		str(programme_action_duration)  + ',' + \
		str(relative_speed_average)  + ',' + \
		str(relative_speed_best)  + ',' + \
		str(normalised_time_since_last_action)  + ',' + \
		str(relative_heart_rate)  + ',' + \
		str(au12)  + ',' + \
		str(au25)  + ',' + \
		str(session_mood) + ',' + \
		str(reward)

		with open('no_action_instances.csv', 'a') as logfile:
			logfile.write(action_record + "\n")
			logfile.flush()


