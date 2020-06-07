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
client = actionlib.SimpleActionClient(
    action_server_name,ArdroneAction
)

rospy.loginfo('Waiting for action Server')
client.wait_for_server()
rospy.loginfo('Action Server found')

goal = ArdroneGoal()
goal.nseconds = 10

client.send_goal(goal,feedback_cb=feedback_callback)
state_result = client.get_state()
rate = rospy.Rate(1)

rospy.loginfo('state_result: '+str(state_result))
counter = 0
while state_result < DONE:
    rospy.loginfo('Doing Stuff while waiting')
    counter += 1
    rate.sleep()
    state_result = client.get_state()
    rospy.loginfo('state_result: '+str(state_result))
    if counter == 2:
        rospy.logwarn('Canceling Goal...')
        client.cancel_goal()
        rospy.logwarn('Goal Canceled')
        state_result = client.get_state()
        rospy.loginfo('Update state_result after Cancel: '+str(state_result))




