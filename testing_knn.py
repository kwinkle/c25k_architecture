#!/usr/bin/env python

#from numpy import array

import random
from sklearn.neighbors import KNeighborsClassifier

inputs = []
outputs = []

r = 1

while r in range (0,100): 
	input1 = random.randint(1,10)
	input2 = random.randint(1,10)
	input_list = []
	input_list.append(input1)
	input_list.append(input2)
	inputs.append(input_list)
	print "inputs looks like " + str(inputs)
	outputs.append(random.randint(1,10))
	print "outputs looks like " + str(outputs)
	knn = KNeighborsClassifier(n_neighbors=1)
	knn.fit(inputs, outputs)
	#test = random.randint(1,10)
	print "test value is " + str([5,6])
	result = knn.predict([[5,6]])
	print "suggested output is " + str(result)
	r +=1 