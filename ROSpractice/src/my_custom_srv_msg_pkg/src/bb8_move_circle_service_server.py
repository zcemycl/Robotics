#!/usr/bin/env python
import rospy
from move_bb8 import MoveBB8
from my_custom_srv_msg_pkg.srv import MyCustomServiceMessage, MyCustomServiceMessageResponse

def my_callback(request):
    rospy.loginfo('Service has been called')
    obj = MoveBB8()
    obj.move_bb8(request.duration)
    rospy.loginfo('Finished')
    response = MyCustomServiceMessageResponse()
    response.success = True
    return response

rospy.init_node('service_server')
service = rospy.Service('/move_bb8_in_circle',
            MyCustomServiceMessage,my_callback)
rospy.loginfo('Service')
rospy.spin()