#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry

def callback(msg):
    print(msg.header)

rospy.init_node('odo_topsub')
sub = rospy.Subscriber('/odom',Odometry,callback)
rospy.spin()
