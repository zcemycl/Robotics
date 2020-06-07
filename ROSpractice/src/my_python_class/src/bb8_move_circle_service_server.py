#!/usr/bin/env python
import rospy
from std_srvs.srv import Empty,EmptyResponse
from bb8_move_circle_class import MoveBB8

def my_callback(request):
    rospy.loginfo('Service has been called.')
    bb8obj = MoveBB8()
    bb8obj.move_bb8()
    rospy.loginfo('Finished service.')
    return EmptyResponse()

rospy.init_node('service_server')
my_service = rospy.Service('/move_bb8_in_circle',
        Empty,my_callback)
rospy.loginfo('Service /move_bb8_in_circle Ready')
rospy.spin()
