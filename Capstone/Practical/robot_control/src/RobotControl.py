#!/usr/bin/env python
"""
ROS based interface for the Course Robotics Specialization Capstone Autonomous Rover.
Updated June 15 2016.
"""
import rospy

import yaml
import numpy as np

import sys

from RosInterface import ROSInterface

# User files, uncomment as completed
#from KalmanFilter import KalmanFilter
from DiffDriveController import DiffDriveController
from path_planner import dijkstras

class RobotControl(object):
    """
    Class used to interface with the rover. Gets sensor measurements through ROS subscribers,
    and transforms them into the 2D plane, and publishes velocity commands.
    """
    def __init__(self, world_map,occupancy_map, pos_init, pos_goal, max_speed, max_omega, x_spacing, y_spacing, t_cam_to_body):
        """
        Initialize the class
        """

        # Handles all the ROS related items
        self.ros_interface = ROSInterface(t_cam_to_body)

        # YOUR CODE AFTER THIS
        
        # Uncomment as completed
        #self.kalman_filter = KalmanFilter(world_map)
        self.diff_drive_controller = DiffDriveController(max_speed, max_omega)

    def process_measurements(self):
        """ 
        YOUR CODE HERE
        This function is called at 60Hz
        """
        meas = self.ros_interface.get_measurements()
        imu_meas = self.ros_interface.get_imu()

	#print("IMU measurements: ", imu_meas)
        #self.ros_interface.command_velocity(0.3, 0.5)
	
	if meas != None:
	    print("Measurements: ", meas)
	    state = np.array([0.0, 0.0, 0.0])
            goal = np.array([meas[0], -meas[1], meas[2]])

            vw = self.diff_drive_controller.compute_vel(state, goal)
	    print("Computed command vel: ", vw)
            if vw[2] == False:
                self.ros_interface.command_velocity(vw[0], vw[1])
            else:
                self.ros_interface.command_velocity(0, 0)
            return
        else:
	    print("No measurments...")
#	    self.ros_interface.command_velocity(0, 0)
            return
           
def main(args):
    rospy.init_node('robot_control')

    # Load parameters from yaml
    param_path = rospy.get_param("~param_path")
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
    robotControl = RobotControl(world_map,occupancy_map, pos_init, pos_goal, max_vel, max_omega, x_spacing, y_spacing, t_cam_to_body)

    # Call process_measurements at 60Hz
    r = rospy.Rate(60)
    #This was added to have the robot run for one second.
    #time=rospy.get_time()
    while not rospy.is_shutdown(): #and rospy.get_time()-time < 1: #remove and stmt
        robotControl.process_measurements()
        r.sleep()
    # Done, stop robot
    robotControl.ros_interface.command_velocity(0,0)

if __name__ == "__main__":
    try:
        main(sys.argv)
    except rospy.ROSInterruptException: pass


