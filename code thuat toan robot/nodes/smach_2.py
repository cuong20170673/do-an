#! /usr/bin/python3

import actionlib
import rospy
from autodock_core.msg import AutoDockingGoal, AutoDockingResult
from autodock_core.msg import AutoDockingAction, AutoDockingFeedback
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseFeedback, MoveBaseResult
import smach
import smach_ros
import time

class Dock(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['finish','fail'])

    def execute(self, userdata):
        current_table_position = [-6.025,2.633, 1.000, -0.002]
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
    
      ###############################################
        client_1 = actionlib.SimpleActionClient("autodock_action",AutoDockingAction)
        client_1.wait_for_server()
        goal = AutoDockingGoal()
        goal.docking_waypoint = "1"
        client_1.send_goal(goal)
        finished_within_time=client_1.wait_for_result(rospy.Duration(60))
        if not finished_within_time:
                client_1.cancel_goal()
                rospy.loginfo("Timed out achieving goal")
                return "fail"
        else:
           return "finish"
                
                








