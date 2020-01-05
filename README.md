# Robotics [![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
Collection of Python implementations of Robotics and Computer Vision theories taught in 
1. University of Pennsylvania Robotics Specialization 
2. University of Cambridge Computer Vision
3. University of Cambridge Robotics
4. University of Cambridge Reinforcement Learning and Decision Making

Aims of this repo are to translate all MATLAB codes into Python to develop a deeper understanding about those Robotics and Computer Vision concepts, and provide more readable scripts for beginners to begin their learning about Robotics. 

## Table of Contents
1. Aerial Robotics

[Quadcopter 1D](https://github.com/zcemycl/Robotics/blob/master/Aerial%20Robotics/Quadcopter1D.ipynb) | [Quadcopter 2D](https://github.com/zcemycl/Robotics/blob/master/Aerial%20Robotics/Quadcopter2D.ipynb) 
:-------------------------:|:-------------------------:
<img src="https://github.com/zcemycl/Robotics/blob/master/Aerial%20Robotics/aerial1d.png" width="500">|<img src="https://github.com/zcemycl/Robotics/blob/master/Aerial%20Robotics/aerial2d.png" width="500">
2. Computational Motion Planning

[Dijkstra Algorithm](https://github.com/zcemycl/Robotics/blob/master/Computational%20Motion%20Planning/DijkstraGrid/DijkstraAlgorithm.ipynb)|[A-star Algorithm](https://github.com/zcemycl/Robotics/blob/master/Computational%20Motion%20Planning/Astar/AstarAlgorithm.ipynb)|[Configuration Space](https://github.com/zcemycl/Robotics/blob/master/Computational%20Motion%20Planning/ConfigurationSpace/ConfigurationSpace.ipynb)|[Potential Field Path](https://github.com/zcemycl/Robotics/blob/master/Computational%20Motion%20Planning/PotentialFieldPlanPath/PotentialFieldPath.ipynb)
:-------------------------:|:-------------------------:|:-------------------------:|:-------------------------:
<img src="https://github.com/zcemycl/Robotics/blob/master/Computational%20Motion%20Planning/DijkstraGrid/Dijkstra.gif" width="200">|<img src="https://github.com/zcemycl/Robotics/blob/master/Computational%20Motion%20Planning/Astar/AStar.gif" width="200">|<img src="https://github.com/zcemycl/Robotics/blob/master/Computational%20Motion%20Planning/ConfigurationSpace/configspace.png" width="200">|<img src="https://github.com/zcemycl/Robotics/blob/master/Computational%20Motion%20Planning/PotentialFieldPlanPath/result.png" width="200">
3. Mobility [Coming Soon]
4. Perception

[2D Homography](https://github.com/zcemycl/Robotics/blob/master/Perception/Logo%20Projection/LogoProjection.ipynb)|[3D Homography](https://github.com/zcemycl/Robotics/blob/master/Perception/3D%20object%20projection/3D%20Homography.ipynb)|[Optical Track](https://github.com/zcemycl/Robotics/blob/master/Perception/Optical%20Track/CornerTracking.ipynb)|[Point Cloud](https://github.com/zcemycl/Robotics/blob/master/Perception/Point%20Cloud/PointCloud.ipynb)|[Calibration](https://github.com/zcemycl/Robotics/blob/master/Perception/Calibration/calibrationviaChessBoard.ipynb)
:-------------------------:|:-------------------------:|:-------------------------:|:-------------------------:|:-------------------------:
<img src="https://github.com/zcemycl/Robotics/blob/master/Perception/Logo%20Projection/result.png" width="160">|<img src="https://github.com/zcemycl/Robotics/blob/master/Perception/3D%20object%20projection/ar_result.png" width="160">|<img src="https://github.com/zcemycl/Robotics/blob/master/Perception/Optical%20Track/result.png" width="160">|<img src="https://github.com/zcemycl/Robotics/blob/master/Perception/Point%20Cloud/pointcloud.png" width="160">|<img src="https://github.com/zcemycl/Robotics/blob/master/Perception/Calibration/calibrationChessBoard.png" width="160">
5. Estimation and Learning

[Kalman Filter](https://github.com/zcemycl/Robotics/blob/master/Estimation%20and%20Learning/Kalman%20Filter/BallPathPrediction_KalmanFilter.ipynb)|[Occupancy Grid Map](https://github.com/zcemycl/Robotics/blob/master/Estimation%20and%20Learning/Occupancy%20Grid%20Map/occGridMapping.ipynb)|[Particle Localization](https://github.com/zcemycl/Robotics/blob/master/Estimation%20and%20Learning/Particle%20Localization/ParticleLocalization.ipynb)
:---:|:---:|:---:
<img src="https://github.com/zcemycl/Robotics/blob/master/Estimation%20and%20Learning/Kalman%20Filter/BallPathPrediction.png" width="200">|<img src="https://github.com/zcemycl/Robotics/blob/master/Estimation%20and%20Learning/Occupancy%20Grid%20Map/occGridMap.png" width="200">|<img src="https://github.com/zcemycl/Robotics/blob/master/Estimation%20and%20Learning/Particle%20Localization/particleLocalization.png" width="200">
6. Capstone

[PD Control](https://github.com/zcemycl/Robotics/blob/master/Capstone/PD%20control/PDTrack.ipynb)|[PD Track Arm](https://github.com/zcemycl/Robotics/blob/master/Capstone/PD%20control/ManipTrack.ipynb)|[Solve ODE](https://github.com/zcemycl/Robotics/blob/master/Capstone/ODE%20solver/Ordinary%20Differential%20Equation.ipynb)|[Estimated Kalman Filter](https://github.com/zcemycl/Robotics/blob/master/Capstone/Estimated%20Kalman%20Filter/EKF.ipynb)
:---:|:---:|:---:|:---:
<img src="https://github.com/zcemycl/Robotics/blob/master/Capstone/PD%20control/PDTrackresult.png" width="200">|<img src="https://github.com/zcemycl/Robotics/blob/master/Capstone/PD%20control/PDArm.png" width="200">|<img src="https://github.com/zcemycl/Robotics/blob/master/Capstone/ODE%20solver/ODEresult.png" width="200">|<img src="https://github.com/zcemycl/Robotics/blob/master/Capstone/Estimated%20Kalman%20Filter/EKFresult.png" width="200">
[Mobile Inverted Pendulum](https://github.com/zcemycl/Robotics/blob/master/Capstone/Mobile%20Inverted%20Pendulum/MIP.ipynb)|[LQR MIP](https://github.com/zcemycl/Robotics/blob/master/Capstone/LQR%20MIP/LQRMIP.ipynb)|[PID MIP](https://github.com/zcemycl/Robotics/blob/master/Capstone/PID%20MIP/PIDControlMIP.ipynb)
<img src="https://github.com/zcemycl/Robotics/blob/master/Capstone/Mobile%20Inverted%20Pendulum/MIPsimulation.png" width="200">|<img src="https://github.com/zcemycl/Robotics/blob/master/Capstone/LQR%20MIP/LQRMIPresult.png" width="200">|<img src="https://github.com/zcemycl/Robotics/blob/master/Capstone/PID%20MIP/PIDMIPresult.png" width="200">|

7. Reinforcement Learning and Decision Making

[Value Iteration](https://github.com/zcemycl/Robotics/blob/master/Reinforcemnet%20Learning/ValueIteration.ipynb)|[Policy Iteration](https://github.com/zcemycl/Robotics/blob/master/Reinforcemnet%20Learning/PolicyIteration.ipynb)|[sarsa](https://github.com/zcemycl/Robotics/blob/master/Reinforcemnet%20Learning/sarsa.ipynb)|[Q-Learning](https://github.com/zcemycl/Robotics/blob/master/Reinforcemnet%20Learning/q-learning.ipynb)
:---:|:-------:|:---:|:---:
<img src="https://github.com/zcemycl/Robotics/blob/master/Reinforcemnet%20Learning/ValIter1.png" width="200">|<img src="https://github.com/zcemycl/Robotics/blob/master/Reinforcemnet%20Learning/ValIter3.png" width="200">|<img src="https://github.com/zcemycl/Robotics/blob/master/Reinforcemnet%20Learning/sarsa3.png" width="200">|<img src="https://github.com/zcemycl/Robotics/blob/master/Reinforcemnet%20Learning/qlearn3.png" width="200">
