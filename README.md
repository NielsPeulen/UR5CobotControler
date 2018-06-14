# UR5CobotControler
This UR5 controller will control your robot and can stop the cobot while it's moving.
This package is tested on a UR5 robot with ROS kinetic.

Get an preview here: https://www.youtube.com/watch?v=YmooYBySHqc

In this package you can learn how to use the UR5 robot in a new way. with this package, you can stop the UR5 robot while it is moving.

# Installing packages
Before you can use this package you need to have a working package that can control the UR5.
follow these tutorials for controlling the robot:

UR5 controller:  http://wiki.ros.org/universal_robot

Simulator:      http://wiki.ros.org/ur_gazebo

Planner:        http://moveit.ros.org/install/source/

# How to use this package

This package contains several launch files and several nodes you can run. First we are going to look at the simulation part.

# Simulation
For the simulation you can run the "UR5_simulation_control.launch" launch file. this launches ROScore, Gazebo, RViz, Moveit! and the UR5 package. when you want to use the functionality of this pack, you can start the "ur5.control.py" node using rosrun. 

This node creates a topic called "ur5_control". When you publish a string "stop" the robot will stop moving. when a empty string is send or something different than "stop", the robot will plan a new path and starts moving. 

This node will move to 6 waypoints that you can set in the code. The easiest way to do this is by moving the robot with Rviz to the position you want, then start the "debug.py" node. this node will print the position. this position you can copy in control node.
if you want more waypoints you have to make the for loop longer. and add a waypoint in the waypoints class. 

```
nkaldsjfalkjsdfl
```

# real UR5
For starting the program with the real robot, you need to start the "UR5_robot_control.launch" launch file. in this file you need to set the correct IP address. before you start the launch file. you can also uncomment the arena ( see package Ron Theelen below) for getting restrictions in you movement. 

if everything is installed correctly, you should be able to move the real robot arm with RViz. The you can start the "ur5.control.py" node. this node will work the same as in the simulation.

# Cobot safety project
When You want to add the full system you have to add the package for the simulation in Rviz made by Ron472: https://github.com/Ron472/cobot_visualisation . This package will make a workcell in your planner. when you post objects to this package you can get realtime objects in you workcell.
To make this project complete you have to add the .py file of StefanCals: https://github.com/StefanCals/ObjectDetection. This script uses a kinect camera to see object in space and by using the package on Ron, it is visible in RViz. 
