# Map My World Robot(SLAM project) [![Udacity - Robotics NanoDegree Program](https://s3-us-west-1.amazonaws.com/udacity-robotics/Extra+Images/RoboND_flag.png)](https://www.udacity.com/robotics)


As RTAB-Map one of the best solutions for SLAM to develop robots that can map environments in 3D, using this package to
create a 2D occupancy grid and 3D octomap. By developing a new package to interface with the rtabmap-ros package and deploy a
robot in a simulated environment using gazebo and RViz then using teleop to move around the environment to generate a proper map.

![robot1](https://s3.amazonaws.com/video.udacity-data.com/topher/2018/February/5a820888_giphy/giphy.gif)


## introduction

Robot localization is the process of determining where a mobile robot is located with respect to its environment. Localization is one of the most fundamental competencies required by an autonomous robot as the knowledge of the robot's own location is an essential precursor to making decisions about future actions.

localization and mapping (SLAM) is the computational problem of constructing or updating a map of an unknown environment while simultaneously keeping track of an agent's location within it.

![robot1](https://github.com/mohamedsayedantar/slam_project/blob/master/images/gazebo1.png)

there are several algorithms known for solving it, at least approximately, in tractable time for certain environments. Popular approximate solution methods include the particle filter, extended Kalman filter, Co-variance intersection, and Graph-SLAM.

RTAB-Map (Real-Time Appearance-Based Mapping) is a RGB-D Graph SLAM approach based on a global Bayesian loop closure detector. The loop closure detector uses a bag-of-words approach to determinate how likely a new image comes from a previous location or a new location. When a loop closure hypothesis is accepted, a new constraint is added to the map's graph, then a graph optimizer minimizes the errors in the map.


## Background

In a general sense, the purpose of SLAM algorithms is easy enough to iterate. A robot will use simultaneous localization and mapping to estimate its position and orientation (or pose) in space while creating a map of its environment. This allows the robot to identify where it is and how to move through some unknown space.

In the real world, a mobile robot with a two-dimensional laser rangefinder sensor is generally deployed on a flat surface to capture a slice of the 3D world. Those two-dimensional slices will be merged at each instant and partitioned into grid cells to estimate the posterior through the occupancy grid mapping algorithm. Three-dimensional maps can also be estimated through the occupancy grid algorithm, but at much higher computational memory because of the large number of noisy three-dimensional measurements that need to be filtered out.

For two dimensional maps, describing a slice of the 3D world. In resource constrained systems, it can be very computationally expensive to build and maintain these maps. 3D representations are even more costly. That being said, robots live in the 3D world, and we want to represent that world and the 3D structures within it as accurately and reliably as possible.

### Collecting data
3D lidar can be used, which is a single sensor with an array of laser beams stacked horizontally. Alternatively, a 2D lidar can be tilted (horizontally moving up and down) or rotated (360 degrees) to obtain 3D coverage.

An RGB-D camera is a single visual camera combined with a laser rangefinder or infrared depth sensor, and allows for the determination of the depth of the image, and ultimately the distance from an object. A stereo camera is a pair of offset cameras, and can be used to directly infer the distance of close objects, in the same way as humans do with their two eyes.

A single camera system is cheaper and smaller, but the software algorithms needed for monocular SLAM are much more complex. Depth cannot be directly inferred from the sensor data of a single image from a single camera. Instead, it is calculated by analyzing data from a sequence of frames in a video.

### FastSLAM

The FastSLAM algorithm uses a custom particle filter approach to solve the full SLAM problem with known correspondences. Using particles, FastSLAM estimates a posterior over the robot path along with the map. Each of these particles holds the robot trajectory which will give an advantage to SLAM to solve the problem of mapping with known poses. In addition, to the robot trajectory, each particle holds a map and a local Gaussian represents each feature of the map. With this algorithm, the problem divided into a separate independent problem. Each of which aims to solve the problem of estimating features of the map. To solve these independent mini problems, FastSlam uses the low dimensional extended Kalman filter. While math features are treated independently, dependency only exists between robot pose uncertainty. This custom approach of representing posterior with particle filter and Gaussian is known by the Rao-Blackwellized Particle Filter One. With the Monte Carlo Localization(MCL) FastSLAM estimates the robot trajectory and with Low-Dimensional Extended Kalman Filter (EKF), FastSLAM estimates features of the map.

### GraphSLAM

GraphSlam is another SLAM algorithm that solves the full SLAM problem. This means that the algorithm recovers the entire path and map, instead of just the most recent pose and map. This difference allows it to consider dependencies between current and previous poses.


## Scene and robot configuration

### Scene

first using the provided environment in gazebo for simulation, rtabmap package to solve slam problem, rviz, and teleop package to move around the environment to map it.

then using the gazebo models database to create the new environment from scratch by adding the ground, cars and other elements to make the environment similar to a small street.

![robot1](https://github.com/mohamedsayedantar/slam_project/blob/master/images/gazebo2.png)

### Robot

using the robot from localization project without any changes, this robot has a RGB-D camera and two laser sensors with six wheels.

![robot1](https://github.com/mohamedsayedantar/udacity_bot/blob/master/images/R14.png)

![robot1](https://github.com/mohamedsayedantar/slam_project/blob/master/images/frames.png)

### Packages

eventually by creating a package named slam-project this package contain the following main folders: 

    - worlds folder
    - launch folder
    - urdf folder
    - meshes folder
    - config folder
    
    




































