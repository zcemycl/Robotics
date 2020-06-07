#!/usr/bin/env python 
import rospy
import time 
import actionlib
from actions_quiz.msg import CustomActionMsgAction,CustomActionMsgResult,CustomActionMsgFeedback
from std_msgs.msg import Empty

class TakeoffLandClass(object):
    _feedback = CustomActionMsgFeedback()
    def __init__(self):
        self._as = actionlib.SimpleActionServer('action_custom_msg_as',
                CustomActionMsgAction,self.goal_callback,False)
        self._as.start()
        self.ctrl_c = False
        self.rate = rospy.Rate(10)

    def goal_callback(self,goal):
        r = rospy.Rate(1)
        success = True

        self._pub_takeoff = rospy.Publisher('/drone/takeoff',
            Empty,queue_size=1)
        self._takeoff_msg = Empty()
        self._pub_land = rospy.Publisher('/drone/land',
            Empty,queue_size=1)
        self._land_msg = Empty()

        signal = goal.goal
        i = 0
        while not i == 3:
            if self._as.is_preempt_requested():
                rospy.loginfo('Goal has been cancelled')
                self._as.set_preempted()
                success = False
                break
            if signal == "TAKEOFF":
                self._pub_takeoff.publish(self._takeoff_msg)
                rospy.loginfo('Taking off...')
            elif signal == "LAND":
                self._pub_land.publish(self._land_msg)
                rospy.loginfo('Landing...')
            time.sleep(1)
            self._feedback.feedback = signal
            self._as.publish_feedback(self._feedback)
            i+=1

if __name__ == '__main__':
    rospy.init_node('action_client')
    TakeoffLandClass()
    rospy.spin()