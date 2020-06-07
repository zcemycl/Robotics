#!/usr/bin/env python
import rospy
from trajectory_by_name_srv.srv import TrajByName, TrajByNameRequest
import sys

rospy.init_node('service_client')
rospy.wait_for_service('/trajectory_by_name')
traj_by_name_service = rospy.ServiceProxy(
    'trajectory_by_name',TrajByName)
traj_by_name_object = TrajByNameRequest()
traj_by_name_object.traj_name = "release_food"
result = traj_by_name_service(traj_by_name_object)
print(result)
