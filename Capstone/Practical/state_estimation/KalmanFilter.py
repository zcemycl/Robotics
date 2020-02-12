#!/usr/bin/python
import numpy as np
from numpy.linalg import inv
import matplotlib.pyplot as plt
from matplotlib import animation
from matplotlib import patches
# import pylab
import time
import math


class KalmanFilter:
    """
    Class to keep track of the estimate of the robots current state using the
    Kalman Filter
    """

    def __init__(self, markers):
        """
        Initialize all necessary components for Kalman Filter, using the
        markers (AprilTags) as the map
        Input: 
        markers - an N by 4 array loaded from the parameters, with each element
            consisting of (x,y,theta,id) where x,y gives the 2D position of a
            marker/AprilTag, theta gives its orientation, and id gives its
            unique id to identify which one you are seeing at any given
            moment
        """
        self.markers = markers
        self.last_time = None  # Used to keep track of time between measurements 
        self.Q_t = np.eye(2)
        self.R_t = np.eye(3)

        self.x_t = np.array([[0.345], [0.345], [1.570796]])  # Bad.
        self.x_t_prediction = np.array([[0], [0], [0]])
        self.P_t = 1000 * np.eye(3)

    def prediction(self, v, imu_meas):
        """
        Performs the prediction step on the state x_t and covariance P_t
        Inputs:
        v - a number representing in m/s the commanded speed of the robot
        imu_meas - a 5 by 1 numpy array consisting of the values
            (acc_x,acc_y,acc_z,omega,time), with the fourth of the values giving
            the gyroscope measurement for angular velocity (which you should
            use as ground truth) and time giving the current timestamp. Ignore
            the first three values (they are for the linear acceleration which
            we don't use)
        Outputs: a tuple with two elements
        predicted_state - a 3 by 1 numpy array of the prediction of the state
        predicted_covariance - a 3 by 3 numpy array of the prediction of the
            covariance
        """
        # todo: change v[2] to omega from imu and dt from imu time
        dt = imu_meas[4, 0] - self.last_time
        omega = v[1]
        # omega = imu_meas[3, 0]
        self.last_time = imu_meas[4, 0]
        G = None
        N = None

        # G = df/dx
        # N = df/dn
        # Coursera's way
        # if omega == 0.0:
        #     G = np.eye(3) + dt * np.array([[0, 0, -v[0] * np.sin(self.x_t[2, 0])],
        #                                    [0, 0, v[0] * np.cos(self.x_t[2, 0])], [0, 0, 0]])
        #
        #     N = dt * np.array([[-np.sin(self.x_t[2, 0]), 0], [np.cos(self.x_t[2, 0]), 0], [0, 1]])
        #     self.x_t_prediction = self.x_t + dt * np.array(
        #         [[v[0] * np.cos(self.x_t[2, 0])], [v[0] * np.sin(self.x_t[2, 0])], [omega]])
        # else:
        #     # Thurn's way
        G = np.eye(3) + np.array([[0, 0, -(v[0] / omega * np.cos(self.x_t[2, 0])) + (v[0] / omega * np.cos(self.x_t[2, 0] + omega * dt))],
                                  [0, 0, -(v[0] / omega * np.sin(self.x_t[2, 0])) + (v[0] / omega * np.sin(self.x_t[2, 0] + omega * dt))],
                                  [0, 0, 0]])
        N = np.array([[(-np.sin(self.x_t[2, 0]) + np.sin(self.x_t[2, 0] + omega * dt)) / omega,
                       (v[0] * (np.sin(self.x_t[2, 0]) - np.sin(self.x_t[2, 0] + omega * dt)) / (omega ** 2)) +
                       (v[0] * (np.cos(self.x_t[2, 0] + omega * dt) * dt) / omega)],
                      [(np.cos(self.x_t[2, 0]) - np.cos(self.x_t[2, 0] + omega * dt)) / omega,
                       (-v[0] * (np.cos(self.x_t[2, 0]) - np.cos(self.x_t[2, 0] + omega * dt)) / (omega ** 2)) +
                       (v[0] * (np.sin(self.x_t[2, 0] + omega * dt) * dt) / omega)],
                      [0, dt]]);
        self.x_t_prediction = self.x_t + \
                              np.array([[-(v[0] / omega * np.sin(self.x_t[2, 0])) + (
                              v[0] / omega * np.sin(self.x_t[2, 0] + omega * dt))],
                                        [(v[0] / omega * np.cos(self.x_t[2, 0])) - (
                                        v[0] / omega * np.cos(self.x_t[2, 0] + omega * dt))],
                                        [omega * dt]])

        self.P_t_prediction = (G.dot(self.P_t)).dot(np.transpose(G)) + (N.dot(self.Q_t)).dot(np.transpose(N))
        return (self.x_t_prediction, self.P_t_prediction)

    def update(self, z_t):
        """
        Performs the update step on the state x_t and covariance P_t
        Inputs:
        z_t - an array of length N with elements that are 4 by 1 numpy arrays.
            Each element has the same form as the markers, (x,y,theta,id), with
            x,y gives the 2D position of the measurement with respect to the
            robot, theta the orientation of the marker with respect to the
            robot, and the unique id of the marker, which you can find the
            corresponding marker from your map
        Outputs:
        predicted_state - a 3 by 1 numpy array of the updated state
        predicted_covariance - a 3 by 3 numpy array of the updated covariance
        """
         
        H = np.eye(3)
        K = (self.P_t_prediction.dot(np.transpose(H))).dot(
            inv((H.dot(self.P_t_prediction)).dot(np.transpose(H)) + self.R_t))

        if z_t != None and z_t.any():

            for i in range(z_t.shape[0]):
                # retrieve pose of the tag in world frame from the map(markers)
                tag_w_pose = self.tag_pos(z_t[i, 3])
                # pose of the tag as measured from the robot
                tag_r_pose = z_t[i, :3]
                # pose of the robot in the world frame
                robot_pose = self.robot_pos(tag_w_pose, tag_r_pose)

                self.x_t = self.x_t_prediction + K.dot(robot_pose - self.x_t_prediction)
        else:
            self.x_t = self.x_t_prediction

        self.P_t = self.P_t_prediction - (K.dot(H)).dot(self.P_t_prediction)

        return self.x_t, self.P_t

    def robot_pos(self, w_pos, r_pos):
        H_W = np.array([[math.cos(w_pos[2]), -math.sin(w_pos[2]), w_pos[0]],
                        [math.sin(w_pos[2]), math.cos(w_pos[2]), w_pos[1]],
                        [0, 0, 1]])
        H_R = np.array([[math.cos(r_pos[2]), -math.sin(r_pos[2]), r_pos[0]],
                        [math.sin(r_pos[2]), math.cos(r_pos[2]), r_pos[1]],
                        [0, 0, 1]])
        w_r = H_W.dot(inv(H_R))
        robot_pose = np.array([[w_r[0, 2]], [w_r[1, 2]], [math.atan2(w_r[1, 0], w_r[0, 0])]])
        return robot_pose

    def tag_pos(self, marker_id):
        for i in range(len(self.markers)):
            marker_i = np.copy(self.markers[i])
            if marker_i[3] == marker_id:
                return marker_i[0:3]
        return None

    def step_filter(self, v, imu_meas, z_t):
        """
        Perform step in filter, called every iteration (on robot, at 60Hz)
        Inputs:
        v, imu_meas - descriptions in prediction. Will be None value if
            values are not available
        z_t - description in update. Will be None value if measurement is not
            available
        Outputs:
        x_t - current estimate of the state
        """

        if imu_meas != None and imu_meas.shape == (5, 1):
            if self.last_time == None:
                self.last_time = imu_meas[4, 0]
            else:
                self.prediction(v, imu_meas)
                self.update(z_t)
        return self.x_t

