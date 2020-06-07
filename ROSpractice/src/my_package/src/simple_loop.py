#!/usr/bin/env python
import rospy
rospy.init_node("ObiWan")
rate = rospy.Rate(2)
while not rospy.is_shutdown():
    print("Help me Obi-Wan Kenobi, you're my only hope.")
    rate.sleep()
    