ACTION= "ACTION"
STYLE= "STYLE"

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


#choice of user/session to get actions for
user = "PT"
session =22

relevant_actions = action_log_df[(action_log_df['action'] != "STYLE_UPDATE") & \
(action_log_df['user_id'] == user) & (action_log_df['session_number'] == session)].copy()

print(relevant_actions.head())
shape = relevant_actions.shape
print("number of rows (action examples) is: {}".format(shape[0]))

for index, row in relevant_actions.iterrows():

	user_id=row['user_id']
	session_number=row['session_number']

	validation_state_id = row['validation_state_id']
	relevant_state_log = state_log_df.loc[state_log_df['state_id']==validation_state_id]

	session_time = relevant_state_log.iloc[0]['session_time']
	norm_session_time = relevant_state_log.iloc[0]['normalised_session_time']

	session_condition = relevant_state_log.iloc[0]['session_condition']

	actionclass = row['action']

	style = row['style']

	validation = row['validation']

	action_record = user_id  + ',' + \
	str(session_number) + ',' + \
	str(session_time)  + ',' + \
	str(norm_session_time)  + ',' + \
	str(actionclass)  + ',' + \
	str(style)  + ',' + \
	str(validation)  + ',' + \
	str(session_condition)

	with open('action_timeline.csv', 'a') as logfile:
		logfile.write(action_record + "\n")
		logfile.flush()