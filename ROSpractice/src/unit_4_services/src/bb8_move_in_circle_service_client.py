#!/usr/bin/env python
import rospy
import rospkg
from std_srvs.srv import Empty, EmptyRequest

rospy.init_node("service_client")
rospy.wait_for_service('/move_bb8_in_circle') 
move_bb8_in_circle_service_client = rospy.ServiceProxy('/move_bb8_in_circle', Empty) # Create the connection to the service
move_bb8_in_circle_request_object = EmptyRequest() # Create an object of type EmptyRequest

result = move_bb8_in_circle_service_client(move_bb8_in_circle_request_object) # Send through the connection the path to the trajectory file to be executed
print(result) # Print the result given by the service called
