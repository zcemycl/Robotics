#!/usr/bin/env python
"""
ROS based interface for the Course Robotics Specialization Capstone Autonomous Rover.
Updated 02 Feb 2020.
"""
#import rospy

import yaml
import numpy as np

import sys

from RobotSim import RobotSim
import matplotlib.pyplot as plt


# TODO for student: User files, uncomment as completed
from path_planner import dijkstras
from KalmanFilter import KalmanFilter
from DiffDriveController import DiffDriveController

class RobotControl(object):
    """
    Class used to interface with the rover. Gets sensor measurements through ROS subscribers,
    and transforms them into the 2D plane, and publishes velocity commands.
    """
    def __init__(self, world_map,occupancy_map, pos_init, pos_goal, max_speed, max_omega, x_spacing, y_spacing, t_cam_to_body):
        """
        Initialize the class
        Inputs: (all loaded from the parameter YAML file)
        world_map - a P by 4 numpy array specifying the location, orientation,
            and identification of all the markers/AprilTags in the world. The
            format of each row is (x,y,theta,id) with x,y giving 2D position,
            theta giving orientation, and id being an integer specifying the
            unique identifier of the tag.
        occupancy_map - an N by M numpy array of boolean values (represented as
            integers of either 0 or 1). This represents the parts of the map
            that have obstacles. It is mapped to metric coordinates via
            x_spacing and y_spacing
        pos_init - a 3 by 1 array specifying the initial position of the robot,
            formatted as usual as (x,y,theta)
        pos_goal - a 3 by 1 array specifying the final position of the robot,
            also formatted as (x,y,theta)
        max_speed - a parameter specifying the maximum forward speed the robot
            can go (i.e. maximum control signal for v)
        max_omega - a parameter specifying the maximum angular speed the robot
            can go (i.e. maximum control signal for omega)
        x_spacing - a parameter specifying the spacing between adjacent columns
            of occupancy_map
        y_spacing - a parameter specifying the spacing between adjacent rows
            of occupancy_map
        t_cam_to_body - numpy transformation between the camera and the robot
            (not used in simulation)
        """

        # TODO for student: Comment this when running on the robot 
        self.robot_sim = RobotSim(world_map, occupancy_map, pos_init, pos_goal,
                                  max_speed, max_omega, x_spacing, y_spacing)
        # TODO for student: Use this when transferring code to robot
        # Handles all the ROS related items
        #self.ros_interface = ROSInterface(t_cam_to_body)

        # Uncomment as completed
        self.kalman_filter = KalmanFilter(world_map)
        self.diff_drive_controller = DiffDriveController(max_speed, max_omega)
        plan = dijkstras(occupancy_map, x_spacing, y_spacing, pos_init, pos_goal)
        self.state_tol = 0.1
        self.path = plan.tolist()
        print("Path: ", self.path, type(self.path))
        self.path.reverse()
        self.path.pop()
        self.state = pos_init
        self.goal = self.path.pop()
        self.x_offset = x_spacing
        self.vw = (0, 0, False)
        # self.goal[0] += self.x_offset/2
        # self.goal[1] += y_spacing
        print("INIT GOAL: ", self.goal)

    #     def dijkstras(occupancy_map, x_spacing, y_spacing, start, goal):


    def process_measurements(self):
        """
        Main loop of the robot - where all measurements, control, and estimation
        are done. This function is called at 60Hz
        """

        meas = self.robot_sim.get_measurements()
        imu_meas = self.robot_sim.get_imu()

        self.vw = self.diff_drive_controller.compute_vel(self.state, self.goal)
        print("VW: ", self.vw)
        print("Running Controller.")

        if self.vw[2] == False:
            self.robot_sim.command_velocity(self.vw[0], self.vw[1])
        else:
            self.robot_sim.command_velocity(0, 0)
        
        est_x = self.kalman_filter.step_filter(self.vw, imu_meas, meas)
        print("EST X: ", est_x, est_x[2][0])
        if est_x[2][0] > 2.617991667:
            est_x[2][0] = 2.617991667
        if est_x[2][0] < 0.523598333:
            est_x[2][0] = 0.523598333
        self.state = est_x
        print("Get GT Pose: ", self.robot_sim.get_gt_pose())
        print("EKF Pose: ", est_x)
        self.robot_sim.get_gt_pose()
        self.robot_sim.set_est_state(est_x)
        
        if np.all(imu_meas != None):
            self.kalman_filter.prediction(self.vw, imu_meas)

        if np.all(meas != None) and meas != []:
            print("Measurements: ", meas)
            if np.all(imu_meas != None):
                # self.kalman_filter.prediction(self.vw, imu_meas)
                self.kalman_filter.update(meas)


        pos_x_check = ((self.goal[0] + self.state_tol) > est_x.item(0)) and \
                      ((self.goal[0] - self.state_tol) < est_x.item(0))

        pos_y_check = ((self.goal[1] + self.state_tol) > est_x.item(1)) and \
                      ((self.goal[1] - self.state_tol) < est_x.item(1))

        if pos_x_check and pos_y_check:
            if self.path != []:
                self.goal = self.path.pop()
                # self.goal[0] += self.x_offset/2
                # self.goal[1] += y_spacing
            else:
                self.goal = est_x

def main(args):
    # Load parameters from yaml
    param_path = 'params.yaml'  # rospy.get_param("~param_path")
    f = open(param_path,'r')
    params_raw = f.read()
    f.close()
    params = yaml.load(params_raw)
    occupancy_map = np.array(params['occupancy_map'])
    world_map = np.array(params['world_map'])
    pos_init = np.array(params['pos_init'])
    pos_goal = np.array(params['pos_goal'])
    max_vel = params['max_vel']
    max_omega = params['max_omega']
    t_cam_to_body = np.array(params['t_cam_to_body'])
    x_spacing = params['x_spacing']
    y_spacing = params['y_spacing']

    # Intialize the RobotControl object
    robotControl = RobotControl(world_map, occupancy_map, pos_init, pos_goal,
                                max_vel, max_omega, x_spacing, y_spacing,
                                t_cam_to_body)

    # Run the simulation
    while not robotControl.robot_sim.done and plt.get_fignums():
        robotControl.process_measurements()
        robotControl.robot_sim.update_frame()

    mng = plt.get_current_fig_manager()
#    mng.frame.Maximize(True)
    plt.ioff()
    plt.show()

    # TODO for student: Use this to run the interface on the robot
    # Call process_measurements at 60Hz
    """r = rospy.Rate(60)
    while not rospy.is_shutdown():
        robotControl.process_measurements()
        r.sleep()
    # Done, stop robot
    robotControl.ros_interface.command_velocity(0,0)"""

if __name__ == "__main__":
    import time
    # time.sleep(3)
    main(sys.argv)


