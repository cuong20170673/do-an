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
                   current_table_position = [0.649,0.543, 0.998, 0.060]
                   goal = MoveBaseGoal()
                   client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
                   rospy.loginfo("Waiting for move base server")
                   client.wait_for_server()
                   goal.target_pose.header.frame_id = 'map'
                   goal.target_pose.pose.position.x = current_table_position[0]
                   goal.target_pose.pose.position.y = current_table_position[1]
                   goal.target_pose.pose.orientation.z = current_table_position[2]
                   goal.target_pose.pose.orientation.w = current_table_position[3]
                   client.send_goal(goal)
                   client.wait_for_result()
                   rospy.loginfo(client.get_result())
                   print("finish move to docking")
                   return "low"
		
		else :
		   return "high"

