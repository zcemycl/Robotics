#!/usr/bin/env python 
import rospy
import time 
import actionlib
from actionlib.msg import TestFeedback,TestAction,TestResult
from ardrone_as.msg import ArdroneAction,ArdroneGoal
from ardrone_as.msg import ArdroneResult,ArdroneFeedback
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from fibonacci_action_server import FibonacciClass

class MoveSquareClass(object):
    _feedback = TestFeedback()
    _result = TestResult()
    def __init__(self):
        self._as = actionlib.SimpleActionServer('move_ardrone_square_as',TestAction,self.goal_callback,False)
        self._as.start()
        self.ctrl_c = False
        self.rate = rospy.Rate(10)

    def publish_once_in_cmd_vel(self,cmd):
        while not self.ctrl_c:
            connections = self._pub_cmd_vel.get_num_connections()
            if connections > 0:
                self._pub_cmd_vel.publish(cmd)
                rospy.loginfo('Publish in cmd_vel...')
                break
            else:
                self.rate.sleep()
    
    def stop_drone(self):
        rospy.loginfo('Stopping...')
        self._move_msg.linear.x = 0.0
        self._move_msg.angular.z = 0.0
        self.publish_once_in_cmd_vel(self._move_msg)
    
    def turn_drone(self):
        rospy.loginfo('Turning...')
        self._move_msg.linear.x = 0.0
        self._move_msg.angular.z = 1.0
        self.publish_once_in_cmd_vel(self._move_msg)

    def move_forward_drone(self):
        rospy.loginfo('Moving...')
        self._move_msg.linear.x = 1.0
        self._move_msg.angular.z = 0.0
        self.publish_once_in_cmd_vel(self._move_msg)

    def goal_callback(self,goal):
        r = rospy.Rate(1)
        success = True 

        self._pub_cmd_vel = rospy.Publisher('/cmd_vel',
            Twist,queue_size=1)
        self._move_msg = Twist()
        self._pub_takeoff = rospy.Publisher('/drone/takeoff',
            Empty,queue_size=1)
        self._takeoff_msg = Empty()
        self._pub_land = rospy.Publisher('/drone/land',
            Empty,queue_size=1)
        self._land_msg = Empty()

        i = 0
        while not i == 3:
            self._pub_takeoff.publish(self._takeoff_msg)
            rospy.loginfo('Taking off...')
            time.sleep(1)
            i+=1

        sideSeconds = goal.goal
        turnSeconds = 1.8

        i = 0
        for i in xrange(0,4):
            if self._as.is_preempt_requested():
                rospy.loginfo('Goal has been cancelled')
                self._as.set_preempted()
                success = False
                break

            self.move_forward_drone()
            time.sleep(sideSeconds)
            self.turn_drone()
            time.sleep(turnSeconds)

            self._feedback.feedback = i
            self._as.publish_feedback(self._feedback)
            r.sleep()

        if success:
            self._result.result = 4*(sideSeconds+turnSeconds)
            rospy.loginfo('It took %i seconds.'%self._result.result)
            self._as.set_succeeded(self._result)

            self.stop_drone()
            i = 0
            while not i == 3:
                self._pub_land.publish(self._land_msg)
                rospy.loginfo('Landing...')
                time.sleep(1)
                i+=1

if __name__ == '__main__':
    rospy.init_node('action_client')
    MoveSquareClass()
    rospy.spin()