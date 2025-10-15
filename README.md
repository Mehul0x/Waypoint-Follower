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

