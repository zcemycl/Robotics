#!/usr/bin/env python
import rospy
import time
import actionlib
from ardrone_as.msg import ArdroneAction,ArdroneGoal
from ardrone_as.msg import ArdroneResult,ArdroneFeedback

nImage = 1

def feedback_callback(feedback):
    global nImage
    print('[Feedback] image n.%d received'%nImage)
    nImage+=1

rospy.init_node('waitforresult')
action_server_name = '/ardrone_action_server'
client = actionlib.SimpleActionClient(action_server_name,
                    ArdroneAction)

rospy.loginfo('Waiting for action Server '+action_server_name)
client.wait_for_server()
rospy.loginfo('Action Server Found...'+action_server_name)

goal = ArdroneGoal()
goal.nseconds = 10

client.send_goal(goal,feedback_cb=feedback_callback)
rate = rospy.Rate(1)

rospy.loginfo('Lets Start wait for action')
while not client.wait_for_result():
    rospy.loginfo('Doing Stuff while waiting')
    rate.sleep()
rospy.loginfo('Finished')