<img width="1197" height="846" alt="image" src="https://github.com/user-attachments/assets/ce70e4fd-1690-43ff-8c96-a4222ce0a6a8" />



This repo creates a smooth trajectory using spline and then follows it with a simple controller.


## Setup
ROS2 Jazzy

Gazebo Harmonic

# Setting up the Repo

cd into your {workspace}/src

~~~
clone https://github.com/Mehul0x/Waypoint-Follower.git
cd ..
colcon build
~~~
Launch the simulation using 
~~~
source install/setup.sh 
ros2 launch waypoint_follower path_tracking.launch.py
~~~

# Architecture

<img width="2231" height="830" alt="Screenshot from 2025-10-15 06-18-00" src="https://github.com/user-attachments/assets/af4f19a9-303e-49b8-9840-256061caf231" />

There are 2 nodes
1. traj_gen_vis_node -> creates the smooth trajectory through waypoints, and visualises them by pusblishing to appropriate topics
2. tracking_controller_node -> uses odometry with the smoothened path to generate appropriate cmd_vel, also does obstacle avoidance using laser scan.

You can set the waypoints in 
~~~
config/waypoints.yaml
~~~
make sure, there are same number of x and y, and that x is strictly increasing.

You can also set the waypoints by putting path to another .yaml file in the argument while launching the launch file

# 


<img width="869" height="86" alt="image" src="https://github.com/user-attachments/assets/e45afa66-31f4-47a1-b6ae-4aaa5f6c19b3" />

We interpoalte the points, with fixed time intervals, we account for the speed and the orientation of the bot while selecting the extrapolating points.

#

<img width="1069" height="68" alt="image" src="https://github.com/user-attachments/assets/ad2c4644-ddff-4c0f-9dc3-67278f0d3dbd" />

The ontroller calculates the turning rate required for an object to smoothly steer towards a target.

It determines the error angle to the target, and then uses that angle, the object's forward speed, and its distance from the target to compute a turning velocity that ensures a smooth, curved path to the destination.

## Limiatation 

 The waypoints need to have x>0 and need to be strictly increasing.

## Obstacle Avoidance (the one implemented)

The robot uses the laser scan data, it checks if there is any obstacle in a radius of 0.5m, then it checks the angle at which contact is possible, if it may collide, it slows down and turns anti clockwise.



