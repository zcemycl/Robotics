#! /usr/bin/env python
import rospkg
import rospy
from services_quiz.srv import BB8CustomServiceMessage, BB8CustomServiceMessageRequest


rospy.init_node('service_move_bb8_in_square_custom_client') # Initialise a ROS node with the name service_client
rospy.wait_for_service('/move_bb8_in_square_custom') # Wait for the service client /move_bb8_in_square to be running
move_bb8_in_square_service_client = rospy.ServiceProxy('/move_bb8_in_square_custom', BB8CustomServiceMessage) # Create the connection to the service
move_bb8_in_square_request_object = BB8CustomServiceMessageRequest() # Create an object of type EmptyRequest


"""
# BB8CustomServiceMessage
float64 side       # The distance of each side of the square
int32 repetitions    # The number of times BB-8 has to execute the square movement when the service is called
---
bool success         # Did it achieve it?
"""
move_bb8_in_square_request_object.side = 2
move_bb8_in_square_request_object.repetitions = 2

rospy.loginfo("Start Two Small Squares...")
result = move_bb8_in_square_service_client(move_bb8_in_square_request_object) # Send through the connection the path to the trajectory file to be executed
rospy.loginfo(str(result)) # Print the result given by the service called

move_bb8_in_square_request_object.side = 4
move_bb8_in_square_request_object.repetitions = 1

rospy.loginfo("Start One Big Square...")
result = move_bb8_in_square_service_client(move_bb8_in_square_request_object) # Send through the connection the path to the trajectory file to be executed
rospy.loginfo(str(result))

rospy.loginfo("END of Service call...")