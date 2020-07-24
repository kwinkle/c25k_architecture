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


#note blacklisted sessions
blacklisted = {
	"MR": [3],
	"LB": [9],
	"MB": [3]
}

#generate style or action instances?

generator_type = ACTION

if generator_type == ACTION:

	relevant_actions = action_log_df[(action_log_df['action'] != "STYLE_UPDATE") & (action_log_df['action'] != "RUN") & \
	(action_log_df['validation'] != "CONTROL") & (action_log_df['action'] != "WALK")].copy()

	print(relevant_actions.head())
	shape = relevant_actions.shape
	print("number of rows (action examples) is: {}".format(shape[0]))

	for index, row in relevant_actions.iterrows():

		user_id=row['user_id']
		session_number=row['session_number']

		if user_id in blacklisted:
			if not session_number in blacklisted[user_id]:

				user_id = row['user_id']
				action_id = row['action_id']
				action = row['action']
				#print(action)
				style = row['style']

				#abstraction to class
				if action == "TIME":
					actionclass = "TIME"
				elif action == "HUMOUR" or action == "CHALLENGE" or action == "SYMPATHISE":
					actionclass = "SOCIAL"
				elif action == "MAINTAIN" or action == "SPEEDUP" or action == "SPEEDDOWN":
					actionclass = "TASK"
				elif action == "PRAISE":
					actionclass = "REWARD"
				elif action == "ANIMATION":
					actionclass = "ANIMATION"
				elif action == "GET_CLOSER":
					actionclass = "GET_CLOSER"
				elif action == "BACK_OFF":
					actionclass = "BACK_OFF"
				elif action == "CHECKPRE":
					actionclass = "CHECKPRE"
				else:
					print("unable to abstract action")

				validation = row['validation']

				#validation to reward value
				if validation == "VALIDATED" or validation == "UNPROMPTED":
					reward = 1
				elif validation == "REFUSED":
					reward = -1
				elif validation == "PASSIVE_ACCEPTED":
					reward = 0
				else:
					print("unable to assign reward")

				validation_state_id = row['validation_state_id']
				relevant_state_log = state_log_df.loc[state_log_df['state_id']==validation_state_id]
				self_attitude = relevant_state_log.iloc[0]['self_attitude']
				expert_attitude = relevant_state_log.iloc[0]['expert_attitude']
				extraversion = relevant_state_log.iloc[0]['extraversion']
				agreeableness = relevant_state_log.iloc[0]['agreeableness'] 
				conscientiousness =relevant_state_log.iloc[0]['conscientiousness']
				emotional_stability = relevant_state_log.iloc[0]['emotional_stability'] 
				openness_experience = relevant_state_log.iloc[0]['openness_experience'] 
				activity_level = relevant_state_log.iloc[0]['activity_level']
				programme_state = relevant_state_log.iloc[0]['programme_state'] 
				normalised_session_time = relevant_state_log.iloc[0]['normalised_session_time']
				normalised_programme_time = relevant_state_log.iloc[0]['normalised_programme_time']
				programme_action_progress = relevant_state_log.iloc[0]['programme_action_progress']
				programme_action_duration = relevant_state_log.iloc[0]['programme_action_duration']
				relative_speed_average = relevant_state_log.iloc[0]['relative_speed_average']
				relative_speed_best = relevant_state_log.iloc[0]['relative_speed_best']
				normalised_time_since_last_action = relevant_state_log.iloc[0]['normalised_time_since_last_action']
				relative_heart_rate = relevant_state_log.iloc[0]['relative_heart_rate']
				au12 = relevant_state_log.iloc[0]['au12'] 
				au25 = relevant_state_log.iloc[0]['au25']
				session_mood = relevant_state_log.iloc[0]['session_mood']

				action_record = user_id + ',' + \
				action_id  + ',' + \
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

				with open('user_action_instances.csv', 'a') as logfile:
					logfile.write(action_record + "\n")
					logfile.flush()

		else:

				user_id = row['user_id']
				action_id = row['action_id']
				action = row['action']
				#print(action)
				style = row['style']

				#abstraction to class
				if action == "TIME":
					actionclass = "TIME"
				elif action == "HUMOUR" or action == "CHALLENGE" or action == "SYMPATHISE":
					actionclass = "SOCIAL"
				elif action == "MAINTAIN" or action == "SPEEDUP" or action == "SPEEDDOWN":
					actionclass = "TASK"
				elif action == "PRAISE":
					actionclass = "REWARD"
				elif action == "ANIMATION":
					actionclass = "ANIMATION"
				elif action == "GET_CLOSER":
					actionclass = "GET_CLOSER"
				elif action == "BACK_OFF":
					actionclass = "BACK_OFF"
				elif action == "CHECKPRE":
					actionclass = "CHECKPRE"
				else:
					print("unable to abstract action")

				validation = row['validation']

				#validation to reward value
				if validation == "VALIDATED" or validation == "UNPROMPTED":
					reward = 1
				elif validation == "REFUSED":
					reward = -1
				elif validation == "PASSIVE_ACCEPTED":
					reward = 0
				else:
					print("unable to assign reward")

				validation_state_id = row['validation_state_id']
				relevant_state_log = state_log_df.loc[state_log_df['state_id']==validation_state_id]
				self_attitude = relevant_state_log.iloc[0]['self_attitude']
				expert_attitude = relevant_state_log.iloc[0]['expert_attitude']
				extraversion = relevant_state_log.iloc[0]['extraversion']
				agreeableness = relevant_state_log.iloc[0]['agreeableness'] 
				conscientiousness =relevant_state_log.iloc[0]['conscientiousness']
				emotional_stability = relevant_state_log.iloc[0]['emotional_stability'] 
				openness_experience = relevant_state_log.iloc[0]['openness_experience'] 
				activity_level = relevant_state_log.iloc[0]['activity_level']
				programme_state = relevant_state_log.iloc[0]['programme_state'] 
				normalised_session_time = relevant_state_log.iloc[0]['normalised_session_time']
				normalised_programme_time = relevant_state_log.iloc[0]['normalised_programme_time']
				programme_action_progress = relevant_state_log.iloc[0]['programme_action_progress']
				programme_action_duration = relevant_state_log.iloc[0]['programme_action_duration']
				relative_speed_average = relevant_state_log.iloc[0]['relative_speed_average']
				relative_speed_best = relevant_state_log.iloc[0]['relative_speed_best']
				normalised_time_since_last_action = relevant_state_log.iloc[0]['normalised_time_since_last_action']
				relative_heart_rate = relevant_state_log.iloc[0]['relative_heart_rate']
				au12 = relevant_state_log.iloc[0]['au12'] 
				au25 = relevant_state_log.iloc[0]['au25']
				session_mood = relevant_state_log.iloc[0]['session_mood']

				action_record = user_id + ',' + \
				action_id  + ',' + \
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

				with open('user_action_instances.csv', 'a') as logfile:
					logfile.write(action_record + "\n")
					logfile.flush()



	actionclass_instances_df = pd.read_csv('user_action_instances.csv', names=['action_id','actionclass',\
		'self_attitude', 'expert_attitude', 'extraversion','agreeableness', 'conscientiousness', 'emotional_stability', 'openness_experience', 'activity_level',\
		'programme_state', 'normalised_session_time', 'normalised_programme_time', 'programme_action_progress', 'programme_action_duration',\
		'relative_speed_average', 'relative_speed_best', 'normalised_time_since_last_action', 'relative_heart_rate', 'au12', 'au25','session_mood',\
		'reward'])

	print(actionclass_instances_df.head())
	print(actionclass_instances_df.shape)

else:

	relevant_actions = action_log_df[(action_log_df['action'] != "STYLE_UPDATE") & (action_log_df['validation'] != "CONTROL")].copy()

	print(relevant_actions.head())
	shape = relevant_actions.shape
	print("number of rows (style examples) is: {}".format(shape[0]))

	for index, row in relevant_actions.iterrows():

		user_id=row['user_id']
		session_number=row['session_number']

		if user_id in blacklisted:
			if not session_number in blacklisted[user_id]:
					
				user_id = row['user_id']
				action_id = row['action_id']
				style = row['style']

				validation = row['validation']

				#validation to reward value
				if validation == "VALIDATED" or validation == "UNPROMPTED":
					reward = 1
				elif validation == "REFUSED":
					reward = -1
				elif validation == "PASSIVE_ACCEPTED": #programme actions pass correct styles as passive_accepts...fixed in future!
					reward = 1
				else:
					print("unable to assign reward")

				validation_state_id = row['validation_state_id']
				relevant_state_log = state_log_df.loc[state_log_df['state_id']==validation_state_id]
				self_attitude = relevant_state_log.iloc[0]['self_attitude']
				expert_attitude = relevant_state_log.iloc[0]['expert_attitude']
				extraversion = relevant_state_log.iloc[0]['extraversion']
				agreeableness = relevant_state_log.iloc[0]['agreeableness'] 
				conscientiousness =relevant_state_log.iloc[0]['conscientiousness']
				emotional_stability = relevant_state_log.iloc[0]['emotional_stability'] 
				openness_experience = relevant_state_log.iloc[0]['openness_experience'] 
				activity_level = relevant_state_log.iloc[0]['activity_level']
				programme_state = relevant_state_log.iloc[0]['programme_state'] 
				normalised_session_time = relevant_state_log.iloc[0]['normalised_session_time']
				normalised_programme_time = relevant_state_log.iloc[0]['normalised_programme_time']
				programme_action_progress = relevant_state_log.iloc[0]['programme_action_progress']
				programme_action_duration = relevant_state_log.iloc[0]['programme_action_duration']
				relative_speed_average = relevant_state_log.iloc[0]['relative_speed_average']
				relative_speed_best = relevant_state_log.iloc[0]['relative_speed_best']
				normalised_time_since_last_action = relevant_state_log.iloc[0]['normalised_time_since_last_action']
				relative_heart_rate = relevant_state_log.iloc[0]['relative_heart_rate']
				au12 = relevant_state_log.iloc[0]['au12'] 
				au25 = relevant_state_log.iloc[0]['au25']
				session_mood = relevant_state_log.iloc[0]['session_mood']

				action_record = user_id + ',' + \
				action_id  + ',' + \
				style  + ',' + \
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


				with open('user_style_instances.csv', 'a') as logfile:
					logfile.write(action_record + "\n")
					logfile.flush()

		else:

				user_id = row['user_id']
				action_id = row['action_id']
				style = row['style']

				validation = row['validation']

				#validation to reward value
				if validation == "VALIDATED" or validation == "UNPROMPTED":
					reward = 1
				elif validation == "REFUSED":
					reward = -1
				elif validation == "PASSIVE_ACCEPTED": #programme actions pass correct styles as passive_accepts...fixed in future!
					reward = 1
				else:
					print("unable to assign reward")

				validation_state_id = row['validation_state_id']
				relevant_state_log = state_log_df.loc[state_log_df['state_id']==validation_state_id]
				self_attitude = relevant_state_log.iloc[0]['self_attitude']
				expert_attitude = relevant_state_log.iloc[0]['expert_attitude']
				extraversion = relevant_state_log.iloc[0]['extraversion']
				agreeableness = relevant_state_log.iloc[0]['agreeableness'] 
				conscientiousness =relevant_state_log.iloc[0]['conscientiousness']
				emotional_stability = relevant_state_log.iloc[0]['emotional_stability'] 
				openness_experience = relevant_state_log.iloc[0]['openness_experience'] 
				activity_level = relevant_state_log.iloc[0]['activity_level']
				programme_state = relevant_state_log.iloc[0]['programme_state'] 
				normalised_session_time = relevant_state_log.iloc[0]['normalised_session_time']
				normalised_programme_time = relevant_state_log.iloc[0]['normalised_programme_time']
				programme_action_progress = relevant_state_log.iloc[0]['programme_action_progress']
				programme_action_duration = relevant_state_log.iloc[0]['programme_action_duration']
				relative_speed_average = relevant_state_log.iloc[0]['relative_speed_average']
				relative_speed_best = relevant_state_log.iloc[0]['relative_speed_best']
				normalised_time_since_last_action = relevant_state_log.iloc[0]['normalised_time_since_last_action']
				relative_heart_rate = relevant_state_log.iloc[0]['relative_heart_rate']
				au12 = relevant_state_log.iloc[0]['au12'] 
				au25 = relevant_state_log.iloc[0]['au25']
				session_mood = relevant_state_log.iloc[0]['session_mood']

				action_record = user_id + ',' + \
				action_id  + ',' + \
				style  + ',' + \
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

				with open('user_style_instances.csv', 'a') as logfile:
					logfile.write(action_record + "\n")
					logfile.flush()

	style_instances_df = pd.read_csv('user_style_instances.csv', names=['action_id','style',\
		'self_attitude', 'expert_attitude', 'extraversion','agreeableness', 'conscientiousness', 'emotional_stability', 'openness_experience', 'activity_level',\
		'programme_state', 'normalised_session_time', 'normalised_programme_time', 'programme_action_progress', 'programme_action_duration',\
		'relative_speed_average', 'relative_speed_best', 'normalised_time_since_last_action', 'relative_heart_rate', 'au12', 'au25','session_mood',\
		'reward'])

	print(style_instances_df.head())
	print(style_instances_df.shape)