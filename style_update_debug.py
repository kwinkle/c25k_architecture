#!/usr/bin/env python

import rospy
import c25k_msgs

from std_msgs.msg import Int32, String 
from c25k_msgs.msg import ValidatedC25KAction, C25KActionType, C25KActionStyle

def publish():
	rospy.init_node('style_debug')
	pub = rospy.Publisher('suggested_style', ValidatedC25KAction, queue_size = 1)
	test_action = ValidatedC25KAction()
	test_action.action = C25KActionType.STYLE_UPDATE
	test_action.style = C25KActionStyle.NEUTRAL
	test_action.duration = 0
	test_action.validation = 0

	rate = rospy.Rate(1)


	while not rospy.is_shutdown():
		rate.sleep()
		print "publishing " + str(test_action.action)
		pub.publish(test_action)

if __name__ == '__main__':
	try:
		publish()
	except rospy.ROSInterruptException:
		pass
