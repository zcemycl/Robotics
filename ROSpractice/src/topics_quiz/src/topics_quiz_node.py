#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

def callback(msg):
    L,M,R = msg.ranges[719],msg.ranges[360],msg.ranges[0]
    move.linear.x = .2
    if M < 1.2:
        move.linear.x = .05
        move.angular.z = .1
    elif L > 30 and R > 30 and M > 30:
        move.linear.x = .2
        move.angular.z = 0
    pub.publish(move)

rospy.init_node('topics_quiz_node')
pub = rospy.Publisher('/cmd_vel',Twist)
sub = rospy.Subscriber('/kobuki/laser/scan',
        LaserScan,callback)
rate = rospy.Rate(2)
move = Twist()
rospy.spin()
