from learnerTestbedGeneralisation import LearnerTestBed
from mlplearner import MLPLearner
from _5_c25klearner import C25KLearner

ACTION="ACTION"
STYLE="STYLE"

test_layers = [(5,2), (40,20,20), (40,40,20), (60,40,20), (60,40,40,20), (60,60,40,20)]
probabilities = [0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9]

test_num_layers = [5,4,3,2]
test_layer_size = [60, 50, 40, 30, 20]

# for number_layers in test_num_layers:
# 	test_setting = [60]*number_layers
# 	test_setting[-1]=20

# 	print test_setting

# 	counter = (number_layers-2)

# 	while counter >= 0:
# 		for layer_size in test_layer_size:

# 			test_setting[counter] = layer_size

# 			print test_setting

for setting in test_layers:

	test_type = ACTION
	style_learner = MLPLearner(STYLE, setting, 0.5)
	action_learner = MLPLearner(ACTION, setting, 0.5)
	# style_learner = KNNLearner(STYLE)
	# action_learner = KNNLearner(ACTION)	
	learner = C25KLearner(style_learner,action_learner)
	testbed = LearnerTestBed(learner, test_type)

	if test_type == ACTION:
		testbed.prepare_training_data('new_action_instances.csv')
	elif test_type == STYLE:
		testbed.prepare_training_data('new_style_instances.csv')

	testbed.train_model_array()

	print("Results for setting: " + str(setting))
	testbed.metrics()
			#counter -= 1

			#record = str(test_setting)

			# #data logging
			# with open('settings_logger.csv', 'a') as logfile:
			# 	logfile.write(record + "\n")
			# 	logfile.flush()

# for prob in probabilities:
# 	test_type = ACTION
# 	style_learner = MLPLearner(STYLE, (40,20,20), prob)
# 	action_learner = MLPLearner(ACTION, (40,20,20), prob)
# 	# style_learner = KNNLearner(STYLE)
# 	# action_learner = KNNLearner(ACTION)	
# 	learner = C25KLearner(style_learner,action_learner)
# 	testbed = LearnerTestBed(learner, test_type)

# 	if test_type == ACTION:
# 		testbed.prepare_training_data('new_action_instances.csv')
# 	elif test_type == STYLE:
# 		testbed.prepare_training_data('new_style_instances.csv')

# 	testbed.train_model_array()

# 	print("Test type: "+ test_type)
# 	print("Results for setting: " + str(prob))
# 	testbed.metrics()