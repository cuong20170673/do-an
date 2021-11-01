#!/usr/bin/env python3

import rospy
from smach import State, StateMachine, Concurrence, Container, UserData
from smach_ros import MonitorState, ServiceState, SimpleActionState, IntrospectionServer
from std_msgs.msg import String


class check_move(State):
	def __init__(self):
		State.__init__(self, outcomes=['move', 'dontmove'])
		rospy.Subscriber('/check_go', String, self.check_go)
		self.check = ''

	def check_go(self, msg):
		self.check = msg.data

	def execute(self, userdata):
		if check != '' :
		    self.check = ''
		    return "move"
		else :
		    return "dontmove"
