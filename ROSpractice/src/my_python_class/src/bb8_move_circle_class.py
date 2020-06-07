#!/usr/bin/env python
import rospy 
from geometry_msgs.msg import Twist

class MoveBB8():
    def __init__(self):
        self.pub = rospy.Publisher('/cmd_vel',
                    Twist,queue_size=1)
        self.cmd = Twist()
        self.ctrl_c = False
        self.rate = rospy.Rate(10)
        rospy.on_shutdown(self.shutdownhook)
    def publish_once_in_cmd_vel(self):
        while not self.ctrl_c:
            connections = self.pub.get_num_connections()
            if connections > 0:
                self.pub.publish(self.cmd)
                rospy.loginfo('Cmd Published')
                break
            else:
                self.rate.sleep()
    def shutdownhook(self):
        self.ctrl_c = True
    def move_bb8(self,linear_speed=.2,angular_speed=.2):
        self.cmd.linear.x = linear_speed
        self.cmd.angular.z = angular_speed
        rospy.loginfo('Moving in BB8!')
        self.publish_once_in_cmd_vel()