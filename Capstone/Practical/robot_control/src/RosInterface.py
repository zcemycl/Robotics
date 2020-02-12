#!/usr/bin/env python
"""
ROS based interface for the Course Robotics Specialization Capstone Autonomous Rover.
Updated June 15 2016.
"""
# ROS imports
import roslib
import rospy

from std_msgs.msg import (
    Header,
)

from apriltags_ros.msg import (
    AprilTagDetectionArray,
    AprilTagDetection,
)

from sensor_msgs.msg import Imu

from geometry_msgs.msg import (
    PoseArray,
    PoseStamped,
    Pose,
    Twist,
)

import cv2
import yaml
import numpy as np

import sys

# Extra utility functions
from utility import *

class ROSInterface(object):
    """
    Class used to interface with the rover. Gets sensor measurements through ROS subscribers,
    and transforms them into the 2D plane, and publishes velocity commands.
    """
    def __init__(self, t_cam_to_body):
        """
        Initialize the class
        """
        # Internal variables
        self._imu_received = False
        self._no_detection=True
        self._no_imu=True
        self._imu = None
        self._t = None
        self._R = None
        self._R_cam2bot = np.array([[0,0,1,0],[1,0,0,0],[0,1,0,0],[0,0,0,1]])
        self._t_cam2bot = t_cam_to_body
        self._R_tag2bot = np.array([[0,-1,0,0],[0,0,1,0],[-1,0,0,0],[0,0,0,1]])

        # ROS publishers and subscribers
        self._pub = rospy.Publisher("/cmd_vel",Twist,queue_size=10)
        rospy.Subscriber("/camera/tag_detections",AprilTagDetectionArray,self._tag_pose_callback)
        rospy.Subscriber("/imu", Imu, self._imu_callback)

    def _tag_pose_callback(self,posearray):
        """
        Callback function for AprilTag measurements
        """
        if (len(posearray.detections)==0):
            return
        (self._t, self._R) = get_t_R(posearray.detections[0].pose.pose)
        self._angle = -np.arctan2(-self._R[2,0],np.sqrt(self._R[2,0]**2+self._R[2,2]**2))
        if math.isnan(self._angle):
            return
        self._R = np.dot(np.dot(self._R_cam2bot, self._R),self._R_tag2bot)
        self._t = np.dot(self._R_cam2bot, self._t)+self._t_cam2bot
        self._marker_num = posearray.detections[0].id
        self._no_detection = False

    def _imu_callback(self, imu):
        """
        Callback function for IMU measurements
        """
        self._imu = np.array([[imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z, imu.angular_velocity.z, imu.header.stamp.to_sec()]]).T
        self._no_imu = False

    def get_imu(self):
        if self._no_imu:
            return None
        else:
            return self._imu
        
    def get_measurements(self):
        """
        Returns information about the last tag seen if any. Returns (x,y,theta) as a
        3x1 numpy array. Returns None if no new tag is seen.
        """
        if self._no_detection:
            return None
        self._no_detection = True
        dx = self._t[0,0]
        dy = self._t[1,0]
        return [dx,dy,self._angle,self._marker_num] # Was [[]]

    def command_velocity(self,vx,wz):
        """
        Commands the robot to move with linear velocity vx and angular
        velocity wz
        """
        twist=Twist()
        twist.linear.x = vx
        twist.angular.z = wz
        self._pub.publish(twist)
