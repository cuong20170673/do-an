#!/usr/bin/env python3

import rospy

import smach
import smach_ros
import time

import test
class Bar(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome2'])

    def execute(self, userdata):
        rospy.loginfo('Executing state BAR')
        return 'outcome2'

def setup_sm():

    sm = smach.StateMachine(outcomes=['Done'])

    with sm:

        smach.StateMachine.add('RECOGNIZE', test.Recognizer( ),
                               transitions={'speak' : 'BAR', 'mute' : 'Done'})

        smach.StateMachine.add('BAR', Bar(), 
                               transitions={'outcome2':'RECOGNIZE'})
       
    sis = smach_ros.IntrospectionServer('Judith_StateMachineServer', sm, '/SM_JUDITH')
    sis.start()

    outcome = sm.execute()

    
    sis.stop()

if __name__ == '__main__':
    rospy.init_node('test_02_sm')
    setup_sm()
1
