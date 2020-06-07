#!/usr/bin/env python
import rospy 
import time
import actionlib
from ardrone_as.msg import ArdroneAction,ArdroneGoal
from ardrone_as.msg import ArdroneResult,ArdroneFeedback

PENDING,ACTIVE = 0,1
DONE,WARN,ERROR = 2,3,4
nImage = 1

def feedback_callback(feedback):
    global nImage
    print('[Feedback] image n.%d received'%nImage)
    nImage += 1

rospy.init_node('no_waitforresult')
action_server_name = '/ardrone_action_server'
client = actionlib.SimpleActionClient(action_server_name,
        ArdroneAction)

rospy.loginfo('Waiting for action server '+action_server_name)
client.wait_for_server()
rospy.loginfo('Action Server found...'+action_server_name)

goal = ArdroneGoal()
goal.nseconds = 10

client.send_goal(goal,feedback_cb=feedback_callback)

state_result = client.get_state()
rate = rospy.Rate(1)
rospy.loginfo('state_result: '+str(state_result))

while state_result < DONE:
    rospy.loginfo('Doing stuff while waiting')
    rate.sleep()
    state_result = client.get_state()
    rospy.loginfo('state_result: '+str(state_result))

rospy.loginfo('[Result] State: '+str(state_result))
if state_result == ERROR:
    rospy.logerr('Something went wrong')
if state_result == WARN:
    rospy.logwarn('Warning in server')








