#!/usr/bin/env python
import rospy
from my_custom_srv_msg_pkg.srv import MyCustomServiceMessage,MyCustomServiceMessageResponse

def my_callback(request):
    print("Request Data==> duration="+str(request.duration))
    my_response = MyCustomServiceMessageResponse()
    if request.duration > 5.0:
        my_response.success = True
    else:
        my_response.success = False
    return my_response

rospy.init_node('service_client')
my_service = rospy.Service('/my_service',MyCustomServiceMessage,my_callback)
rospy.spin()