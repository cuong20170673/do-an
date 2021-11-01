#!/usr/bin/env python3

import rospy
import smach
import smach_ros
import time
from std_msgs.msg import String
from sensor_msgs.msg import BatteryState 
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseFeedback, MoveBaseResult
import actionlib
from sensor_msgs.msg import LaserScan

class CheckBattery(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['high', 'low'])
		rospy.Subscriber('laptop_charge', BatteryState, self.callback)
		self.status = 0

	def callback(self, data):
		self.status = data.percentage

	def execute(self, userdata):
		
		if self.status <= 95 :

                   return "low"
		
		else :
		   return "high"

