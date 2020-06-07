#! /usr/bin/env python

import rospy
from std_srvs.srv import Empty, EmptyResponse # you import the service message python classes generated from Empty.srv.
from services_quiz.srv import BB8CustomServiceMessage, BB8CustomServiceMessageResponse
from geometry_msgs.msg import Twist
import time

def my_callback(request):
    
    rospy.loginfo("The Service move_bb8_in_square_custom has been called")
    
    radius = request.side
    for i in range(request.repetitions):
        rospy.loginfo("Moving forward...")
        move_circle.linear.x = 0.2
        move_circle.angular.z = 0
        my_pub.publish(move_circle)
        time.sleep(radius)
        rospy.loginfo("Rotating...")
        move_circle.linear.x = 0
        move_circle.angular.z = 0.2
        my_pub.publish(move_circle)
        time.sleep(4)
        rospy.loginfo("Moving forward...")
        move_circle.linear.x = 0.2
        move_circle.angular.z = 0
        my_pub.publish(move_circle)
        time.sleep(radius)
        rospy.loginfo("Rotating...")
        move_circle.linear.x = 0
        move_circle.angular.z = 0.2
        my_pub.publish(move_circle)
        time.sleep(4)
        rospy.loginfo("Moving forward...")
        move_circle.linear.x = 0.2
        move_circle.angular.z = 0
        my_pub.publish(move_circle)
        time.sleep(radius)
        rospy.loginfo("Rotating...")
        move_circle.linear.x = 0
        move_circle.angular.z = 0.2
        my_pub.publish(move_circle)
        time.sleep(4)
        rospy.loginfo("Moving forward...")
        move_circle.linear.x = 0.2
        move_circle.angular.z = 0
        my_pub.publish(move_circle)
        time.sleep(radius)
        rospy.loginfo("Rotating...")
        move_circle.linear.x = 0
        move_circle.angular.z = 0.2
        my_pub.publish(move_circle)
        time.sleep(4)
        rospy.loginfo("Stopping...")
        move_circle.linear.x = 0
        move_circle.angular.z = 0
        my_pub.publish(move_circle)
        time.sleep(2)
        
    rospy.loginfo("Finished service move_bb8_in_square_custom")
    response = BB8CustomServiceMessageResponse()
    response.success = True
    return response # the service Response class, in this case EmptyResponse

rospy.init_node('service_move_bb8_in_square_server') 
my_service = rospy.Service('/move_bb8_in_square_custom', BB8CustomServiceMessage , my_callback) # create the Service called move_bb8_in_circle with the defined callback
my_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
move_circle = Twist()
rospy.loginfo("Service /move_bb8_in_square_custom Ready")
rospy.spin() # mantain the service open.