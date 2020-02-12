#!/usr/bin/env python
"""
Utility functions that are commonly used across the visual_servoing package.
Used mostly for extending coordinate transformations beyond the scope of transformations.py.
Written by Alex Zhu (alexzhu(at)seas.upenn.edu)
"""
import numpy as np

import roslib
from std_msgs.msg import (
    Header,
)
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Quaternion,
)
from tf.transformations import *

def get_t_R(pose):
    """
    Returns the translation vector (4x1) and rotation matrix (4x4) from a pose message
    """
    t=np.transpose(np.matrix([pose.position.x,pose.position.y,pose.position.z,0]))
    quat=[pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
    R_full=quaternion_matrix(quat)
    R=R_full
    return t,R

def make_pose_stamped_msg(t,R):
    """
    Returns a pose stamped message from a translation vector and rotation matrix (4x4) for publishing.
    NOTE: Does not set the target frame.
    """
    pose_stamped_msg=PoseStamped()
    pose_stamped_msg.header=Header()
    pose_stamped_msg.header.stamp=rospy.Time.now()
    pose_msg=Pose()
    pose_msg.position.x=t[0]
    pose_msg.position.y=t[1]
    pose_msg.position.z=t[2]
    quat=quaternion_from_matrix(R)
    pose_msg.orientation.x=quat[0]
    pose_msg.orientation.y=quat[1]
    pose_msg.orientation.z=quat[2]
    pose_msg.orientation.w=quat[3]
    pose_stamped_msg.pose=pose_msg
    return pose_stamped_msg
