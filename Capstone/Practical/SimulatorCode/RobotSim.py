#!/usr/bin/python
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation
from matplotlib import patches

#import thread
import time
import math

class RobotSim(object):
    """
    Visualizes/simulates trajectory of robot over time
    """
    def __init__(self, markers, occupancy_map, pos_init, goal_pos, max_speed,
                 max_omega, x_spacing, y_spacing):
        """
        Initializes the class
        Inputs:
        markers - a P by 4 numpy array specifying the location, orientation,
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
        """
        # Parameters for simulation (global parameters)
        # Current estimate of state - for visualization purposes
        self.__est_state = None
        # framerate of the simulation (also used for integration)
        self.__dt = 0.05
        # How many time steps behind the robot the path is drawn
        self.__lag_len = 100000
        # How wide to draw the markers in the simulation
        self.__marker_width = 0.05
        # How high to draw the markers in the simulation
        self.__marker_height = 0.1
        # Simulated IMU noise structure
        self.__imu_noise = [0.05, 0.05, 0.05, 0.02]
        # Simulated control input noise structure
        self.__control_noise = [0.05, 0.05]
        # Simulated noise from image measurements
        self.__image_noise = [0.01, 0.01, np.pi/180/2]
        # Maximum allowed angular velocity of the simulated robot (rad/s)
        self.__MAX_OMEGA = max_omega
        # Maximum allowed forward velocity of the simulated robot (m/s)
        self.__MAX_VELOCITY = max_speed 
        # IMU gravity vector magnitude
        self.__GRAVITY = -9.81
        # Receiving rate for image measurements
        self.__MEAS_RATE = 0.15
        # Rate of IMU (currently faster than simulation rate 
        self.__IMU_RATE = 0.045
        # Shapes for drawing the robot in the simulation
        self.__shapes = [
                [[ 4, 2],[ 4,-2],[-4,-2],[-4, 2]],
                [[-1, 3],[-3, 3],[-3, 2],[-1, 2]],
                [[ 3, 3],[ 1, 3],[ 1, 2],[ 3, 2]],
                [[-1,-3],[-3,-3],[-3,-2],[-1,-2]],
                [[ 3,-3],[ 1,-3],[ 1,-2],[ 3,-2]],
                [[ 4, 2],[ 4,-2],[ 5, 0]],
                ]
        for s in range(len(self.__shapes)):
            for k in range(len(self.__shapes[s])):
                # Resize shapes to match true robot
                self.__shapes[s][k][0] /= 56.25
                self.__shapes[s][k][1] /= 60.
        # Passed in parameters
        self.__occupancy_map = occupancy_map
        self.__x_spacing = x_spacing
        self.__y_spacing = y_spacing
        self.markers = markers
        self.markers_flipped = np.copy(self.markers)
        for i in range(self.markers_flipped.shape[0]):
            self.markers_flipped[i][2]+=np.pi
            if self.markers_flipped[i][2] > np.pi:
                self.markers_flipped[i][2]-=2*np.pi
            elif self.markers_flipped[i][2] < np.pi:
                self.markers_flipped[i][2]+=2*np.pi

        # Parameters used to update the robot in the simulation
        self.last_meas_time = -1
        self.last_imu_time = -1
        self.__view_half_angle = 37*np.pi/180
        self.__x_gt = pos_init
        self.__vel = 0
        self.__omega = 0
        # For history
        self.__acc_history = np.array([pos_init[:,0].tolist(),pos_init[:,0].tolist(),pos_init[:,0].tolist()])
        # For termination
        self.done=False
        self.__frame_num = 0

        plt.ion()
        self.__plot()
            
    def get_imu(self):
        """
        Get IMU measurements
        Outputs: Returns 5 by 1 numpy vector (acc_x, acc_y, acc_z, omega, time)
        acc_* - linear acceleration values
        omega - angular velocity value
        time - current time stamp
        """
        curr_time = self.__frame_num*self.__dt
        if curr_time - self.last_imu_time < self.__IMU_RATE:
            return None
        noise = np.random.normal(0,self.__imu_noise)
        R = self.__gen_R(self.__x_gt[2,0])

        accx = (self.__acc_history[0, 0]+ self.__acc_history[2, 0]-2* self.__acc_history[1, 0])/(self.__dt**2)
        accy = (self.__acc_history[0, 1]+self.__acc_history[2, 1]-2*self.__acc_history[1, 1])/(self.__dt**2)
        
        omega = self.__omega
        
        imu_meas = np.array([[accx,accy,self.__GRAVITY,omega,curr_time]]).T

        imu_meas[0:2,:] = np.dot(R.T, imu_meas[0:2,:])
        imu_meas[0:4,:] += noise[:, None]
        return imu_meas

    def __plot(self):
        """
        Start the simulation - plot/visualize robot motion
        """
        # Main plot function calls
        self.__line, = plt.plot(self.__x_gt[0,0], self.__x_gt[1,0],'o')
        plt.axis('equal')
        axes = plt.axes(xlim=(-0.5,2),ylim=(0,2.5))
        axes.set_aspect('equal')
       

        # Viewing angle visualization
        distance = 100
        cosx = np.cos(self.__view_half_angle)
        sinx = np.sin(self.__view_half_angle)
        self.__view_lines_1, = plt.plot(
                                [self.__x_gt[0,0], self.__x_gt[0,0] + distance*cosx],
                                [self.__x_gt[1,0], self.__x_gt[1,0] + distance*sinx],'r-')
        self.__view_lines_2, = plt.plot(
                                [self.__x_gt[0,0], self.__x_gt[0,0] + distance*cosx],
                                [self.__x_gt[1,0], self.__x_gt[1,0] - distance*sinx],'r-')

#        plt.hold(True)

        # Drawing of robot (building drawing objects for it)
        self.__bot_parts = [ 0 for i in range(len(self.__shapes)) ]
        for s in range(len(self.__shapes)):
            self.__bot_parts[s] = patches.Polygon(
                                    self.__shapes[s],
                                    fill=True,facecolor='b',edgecolor='k')
            axes.add_patch(self.__bot_parts[s])

        self.__bot_parts_est = [ 0 for i in range(len(self.__shapes)) ]

        for s in range(len(self.__shapes)):
            self.__bot_parts_est[s] = patches.Polygon(
                                    self.__shapes[s],
                                    fill=False, facecolor='w',edgecolor='k')
            axes.add_patch(self.__bot_parts_est[s])

        # Drawing of the map (building drawing objects)
        # Obstacles
        self.__obstacles = [0 for i in range(np.sum(self.__occupancy_map))]
        obs_iter = 0
        for r in range(self.__occupancy_map.shape[0]):
            for c in range(self.__occupancy_map.shape[1]):
                if self.__occupancy_map[r,c]:
                    self.__obstacles[obs_iter] = patches.Rectangle(
                        (c*self.__x_spacing,
                         r*self.__y_spacing),
                        self.__x_spacing,
                        self.__y_spacing,
                        fill=True, facecolor='r', edgecolor='k')
                    axes.add_patch(self.__obstacles[obs_iter])
                    obs_iter+=1
        # Markers
        self.__markers = [ 0 for i in range(len(self.markers_flipped)) ]
        self.__markers_dir = [ 0 for i in range(len(self.markers_flipped)) ]
        for m in range(len(self.markers_flipped)):
            angle=(self.markers_flipped[m][2]/np.pi)*180.0
            R = np.array([[np.cos(self.markers_flipped[m][2]), -np.sin(self.markers_flipped[m][2])],
                          [np.sin(self.markers_flipped[m][2]), np.cos(self.markers_flipped[m][2])]])
            offset = np.array([[self.__marker_width, self.__marker_height]]).T
            rotated_offset = np.dot(R,offset)
            
            self.__markers[m] = patches.Rectangle(
                (self.markers_flipped[m][0] - rotated_offset[0]/2.,
                 self.markers_flipped[m][1] - rotated_offset[1]/2.),
                self.__marker_width,
                self.__marker_height,
                angle=(self.markers_flipped[m][2]/np.pi)*180.0,
                fill=True,facecolor='r',edgecolor='k')
            
            dir_offset = np.array([[-self.__marker_width, 0.04]]).T
            rotated_dir_offset = np.dot(R, dir_offset)
            self.__markers_dir[m] = patches.Rectangle(
                (self.markers_flipped[m][0] - rotated_dir_offset[0]/2.,
                 self.markers_flipped[m][1] - rotated_dir_offset[1]/2.),
                self.__marker_width*0.5,
                0.04,
                angle=(self.markers_flipped[m][2]/np.pi)*180.0,
                fill=True,facecolor='r',edgecolor='k')
            axes.add_patch(self.__markers[m])
            axes.add_patch(self.__markers_dir[m])

        # Trail points of path
        self.__history_x = np.zeros((1,self.__lag_len))
        self.__history_y = np.zeros((1,self.__lag_len))
        self.__trails = [plt.plot(hx,hy,'k')[0] for hx,hy in zip(self.__history_x,self.__history_y)]

    def command_velocity(self,vx,wz):
        """
        Control the v and omega of the robot
        Inputs:
        vx - number for linear velocity of the robot, in m/s
        wz - number for angular velocity of the robot, in rad/s
        Outputs: None, but controls the robot
        """
        noise = np.random.normal(0,self.__control_noise)
        self.__vel = min(max(vx + noise[0],0),self.__MAX_VELOCITY)
        self.__omega = min(max(wz + noise[1],-self.__MAX_OMEGA),self.__MAX_OMEGA)

    def __gen_R(self, theta):
        """
        Generate 2D rotation matrix
        Inputs: theta - angle of rotation
        Outputs: R - a 2 by 2 numpy array of rotation of theta radians
        """
        return np.array([[np.cos(theta), -np.sin(theta)],
                         [np.sin(theta),  np.cos(theta)]])

    def __H(self,X):
        """
        Given an X = [x,y,theta], create associated transform
        Inputs: X - an array of size 3 with [x,y,theta] in it
        Output: H - a 3 by 3 numpy array of homogeneous representation rotation
                    and translation
        """
        return np.array([[np.cos(X[2]), -np.sin(X[2]), X[0]],
                         [np.sin(X[2]),  np.cos(X[2]), X[1]],
                         [         0.0,           0.0,  1.0]])
    def __vectH(self,H):
        """
        Given H created from H(X), extract the X
        Inputs: H - a 3 by 3 numpy array of homogeneous representation rotation
                    and translation
        Outpus: X - an array of size 3 of [x,y,theta] of transformation
        """
        return [H[0,2],H[1,2],math.atan2(H[1,0],H[0,0])]

    def get_measurements(self):
        """
        Returns a list of lists of visible landmarks and a fresh boolean that
        checks if it the measurement is ready
        Outputs:
        measurements - a N by 5 list of visible tags or None. The tags are in
            the form in the form (x,y,theta,id,time) with x,y being the 2D
            position of the marker relative to the robot, theta being the
            relative orientation of the marker with respect to the robot, id
            being the identifier from the map, and time being the current time
            stamp. If no tags are seen, the function returns None.
        """
        curr_time = self.__frame_num*self.__dt
        if curr_time - self.last_meas_time < self.__MEAS_RATE or len(self.markers_flipped) == 0:
            return None
        self.last_meas_time = curr_time
        self.__visible_markers = [False for i in range(len(self.markers_flipped))]

        H_WR = self.__H(self.__x_gt[:,0])
        # Get measurements to the robot frame
        meas = []

        for i in range(len(self.markers_flipped)):
            H_WT = self.__H(self.markers[i])
            H_RT = np.linalg.solve(H_WR,H_WT)
            
            x_new = H_RT[0:2,2]
            theta_new = math.atan2(H_RT[1,0], H_RT[0,0])
            marker_view_angle = \
                np.absolute(np.arccos(x_new[0]/(math.sqrt(x_new[0]**2 + x_new[1]**2))))
            if abs(marker_view_angle) < self.__view_half_angle and abs(theta_new) < np.pi/3 and x_new[0] < 2:
                self.__visible_markers[i] = True
                meas_i = np.array([x_new[0],x_new[1], theta_new, self.markers_flipped[i][3], self.last_meas_time])
                meas_i[0:3] = meas_i[0:3] + np.array([np.random.normal(0, self.__image_noise)])
                meas.append(meas_i.tolist())
        return meas

    def set_est_state(self, est_state):
        """
        Sets the estimated state of the simulation - for visualization purposes
        Inputs: est_state - a 3 by 1 numpy array of (x,y,theta) of the estimated
                            state
        """
        self.__est_state = est_state

    def get_gt_pose(self):
        return self.__x_gt
  
    def update_frame(self):
        """
        Called to update the simulation on every frame
        (where the simulating happens)
        """
        # Prep for the next frame
#        plt.hold(True)
        self.__frame_num += 1
        if self.done:
            return # early termination

        # Color visible markers
        for i in range(len(self.__visible_markers)):
            if self.__visible_markers[i]:
                self.__markers[i].set_facecolor('g')
                self.__markers_dir[i].set_facecolor('y')
            else:
                self.__markers[i].set_facecolor('r')
                self.__markers_dir[i].set_facecolor('r')

        # Update properties
        self.__x_gt[0,0] += self.__dt*self.__vel*np.cos(self.__x_gt[2,0])
        self.__x_gt[1,0] += self.__dt*self.__vel*np.sin(self.__x_gt[2,0])
        self.__x_gt[2,0] += self.__dt*self.__omega
        if self.__x_gt[2,0] > np.pi:
            self.__x_gt[2,0] -= 2*np.pi
        if self.__x_gt[2,0] < -np.pi:
            self.__x_gt[2,0] += 2*np.pi
        
        # Position and Direction
        posx  = self.__x_gt[0,0]
        posy  = self.__x_gt[1,0]
        theta = self.__x_gt[2,0]
        # Angles
        cos = math.cos(theta)
        sin = math.sin(theta)

        # Draw main point of path
        self.__line.set_xdata(posx)
        self.__line.set_ydata(posy)

        if self.__frame_num > 0:
            # Draw trails
            self.__history_x[:,((self.__frame_num-1)%self.__lag_len):] = posx
            self.__history_y[:,((self.__frame_num-1)%self.__lag_len):] = posy
            self.__trails[0].set_xdata(self.__history_x[0])
            self.__trails[0].set_ydata(self.__history_y[0])

        # Viewing angle visualization
        distance = 2
        cosx = np.cos(self.__view_half_angle + theta)
        sinx = np.sin(self.__view_half_angle + theta)
        cosy = np.cos(-self.__view_half_angle + theta)
        siny = np.sin(-self.__view_half_angle + theta)
        self.__view_lines_1.set_xdata([posx, posx + distance*cosx])
        self.__view_lines_1.set_ydata([posy, posy + distance*sinx])
        self.__view_lines_2.set_xdata([posx, posx + distance*cosy])
        self.__view_lines_2.set_ydata([posy, posy + distance*siny])

        # Draw robot (with rectables specified by __shapes)
        for s in range(len(self.__shapes)):
            pts = np.zeros((len(self.__shapes[s]),2))
            for k in range(len(self.__shapes[s])):
                pts[k][0] = posx + cos*self.__shapes[s][k][0] - sin*self.__shapes[s][k][1]
                pts[k][1] = posy + sin*self.__shapes[s][k][0] + cos*self.__shapes[s][k][1]
            self.__bot_parts[s].set_xy(pts)

        # Draw estimated position of robot (ghost version of robot)
        if self.__est_state is not None:
            for s in range(len(self.__shapes)):
                pts = np.zeros((len(self.__shapes[s]),2))
                for k in range(len(self.__shapes[s])):
                    cos = math.cos(self.__est_state[2,0])
                    sin = math.sin(self.__est_state[2,0])
                    
                    pts[k][0] = self.__est_state[0,0] + cos*self.__shapes[s][k][0] - sin*self.__shapes[s][k][1]
                    pts[k][1] = self.__est_state[1,0] + sin*self.__shapes[s][k][0] + cos*self.__shapes[s][k][1]
                self.__bot_parts_est[s].set_xy(pts)
            self.__est_state = None

        # Finish plotting stuff
        plt.axes(xlim=(-0.5,2),ylim=(0,2.5))

        # Plot title
        plt.title("At timestep: %.2f" %(self.__frame_num*self.__dt))

        # Update for acceleration history (to visualize path)
        self.__acc_history[0] = self.__acc_history[1]
        self.__acc_history[1] = self.__acc_history[2]
        self.__acc_history[2] = [self.__x_gt[0,0],self.__x_gt[1,0], self.__x_gt[2,0]]
        plt.pause(0.001)
        return
