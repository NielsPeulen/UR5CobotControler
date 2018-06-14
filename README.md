# UR5CobotControler
This UR5 controler will control your robot and can stop the cobot while it's moving.
This package is tested on a UR5 robot with ROS kinetic.


In this package you can learn how to use the UR5 robot in a new way. with this package, you can stop the UR5 robot while it is moving.

# Installing packages
Before you can use this package you need to have a working package that can control the UR5.
follow these tutorials for controling the robot:

UR5 controler:  http://wiki.ros.org/universal_robot

Simulator:      http://wiki.ros.org/ur_gazebo

Planner:        http://moveit.ros.org/install/source/

# How to use this package

This package contains several launchfiles and several nodes you can run. First we gonna look at the simulation part.

for the simulation you can run the "UR5_simulation_control.launch" launch file. this launches ROScore, Gazebo, RViz, Moveit! and the UR5 package. when you want to use the functionality of this pack, you can start the "ur5.control.py" node using rosrun. 

This node creates a topic called "ur5_control". When you publish a string "stop" the robot will stop moving. when a empty string is send or something different than "stop", the robot will plan a new path and starts moving. 



