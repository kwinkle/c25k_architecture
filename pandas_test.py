import pandas as pd

# df = pd.read_csv('test.csv',names=['time','state_id','user_id','programme_state','task_success','session_time','session_time_remaining', \
# 	'normalised_session_time', 'programme_time', 'normalised_programme_time', 'time_spent_prog_action', 'time_remaining_prog_action', \
# 	'programme_action_progress', 'programme_action_duration', 'current_speed', 'relative_speed_average', 'relative_speed_best',\
# 	'time_since_last_action', 'normalised_time_since_last_action', 'heart_rate', 'relative_heart_rate', 'session_mood', 'user_run_pb', \
# 	'user_walk_pb','user_run_avg', 'user_walk_avg'])

df = pd.read_csv('state_logger.csv', names=['time', 'state_id', 'user_id', 'session_condition','self_attitude', 'expert_attitude', 'extraversion', 'agreeableness', \
				'conscientiousness', 'emotional_stability', 'openness_experience', 'activity_level', 'session_number', 'programme_state', 'task_success', \
				'session_time', 'session_time_remaining', 'normalised_session_time', 'programme_time', 'normalised_programme_time', 'time_spent_prog_action', \
				'time_remaining_prog_action', 'programme_action_progress', 'programme_action_duration', 'current_speed', 'relative_speed_average', 'relative_speed_best', \
				'time_since_last_action', 'normalised_time_since_last_action', 'heart_rate', 'relative_heart_rate', 'au12', 'au25','session_mood', 'user_run_pb', 'user_walk_pb', \
				'user_run_avg','user_walk_avg'])

sliced_df = df.loc[(df['user_id']=="GB") & (df['programme_state']==0.5) & (df['current_speed']<4.0)].copy()
print sliced_df
print sliced_df["current_speed"].max()

# print df = df[df['programme_state']==0.5].copy()
# print df = df['current_speed']
# print df['current_speed'].mean()