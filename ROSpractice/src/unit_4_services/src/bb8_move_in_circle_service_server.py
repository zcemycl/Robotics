#!/usr/bin/env python
import rospy
from std_srvs.srv import Empty, EmptyResponse
from geometry_msgs.msg import Twist

def my_callback(request):
    print('My_callback has been called')
    move.linear.x = .05
    move.angular.z = .1
    pub.publish(move)
    return EmptyResponse()

rospy.init_node('service_server')
my_service = rospy.Service('/move_bb8_in_circle',Empty,my_callback)
pub = rospy.Publisher('/cmd_vel',Twist,queue_size=1)
move = Twist()
rospy.spin()