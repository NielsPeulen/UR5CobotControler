# UR5CobotControler
This UR5 controller will control your robot and can stop the cobot while it's moving.
This package is tested on a UR5 robot with ROS kinetic.

Get an preview here: https://www.youtube.com/watch?v=YmooYBySHqc

In this package you can learn how to use the UR5 robot in a new way. with this package, you can stop the UR5 robot while the UR5 robot is moving.

## Installing packages
Before you can use this package, you need to have to install some packages that can control the UR5 first.
follow these tutorials for controlling the robot:

UR5 controller:  http://wiki.ros.org/universal_robot

Simulator:      http://wiki.ros.org/ur_gazebo

Planner:        http://moveit.ros.org/install/source/

# How to install this package in your catkin workspace

```
cd catkin_ws/src
git clone https://github.com/NielsPeulen/UR5CobotControler.git
cd ..
catkin_make
cd catkin_ws/src/UR5CobotControler_master/
sudo chmod +x src/ur5_tutorial.py 
sudo chmod +x src/ur5_control.py
sudo chmod +x src/debug.py 
```
# How to use this package

This package contains several launch files and several nodes you can run. First we are going to look at the simulation part.

## Simulation
For the simulation you can run the "UR5_simulation_control.launch" launch file. this launches ROScore, Gazebo, RViz, Moveit! and the UR5 package.
```
roslaunch UR5CobotControler-master UR5_simulation_control.launch
```
when you want to use the functionality of this package, you can start the "ur5.control.py" node using rosrun in a new tab. 
```
rosrun UR5CobotControler-master ur5_control.py
```
This node creates a topic called "ur5_control". When you publish a string "stop" the robot will stop moving. when a empty string is send or something different than "stop", the robot will plan a new path and starts moving. 
```
rostopic pub /ur5_control std_msgs/String "stop"
rostopic pub /ur5_control std_msgs/String ""
```
This node will move to 6 waypoints that you can set in the code, ur5_control.py The easiest way to do this is by moving the robot with Rviz to the position you want, then start the "debug.py" node. this node will print the position. this position you can copy in control node.
```
rosrun UR5CobotControler-master debug.py
```
In the ur5_control.py that you can find in the src directory, code line 140 to 146 you can find all the waypoints the robot will move to. 
If you want to add a waypoint give it a logical name, define it as waypoint and set all position values in the right order.
You can add waypoints after code line 146. Be aware: all values are in **quaternion**.
```
# values order by: oriX, oriY, oriZ, oriW, posX, posY, posZ
wphome = waypoint(-0.528360791026,-0.50224685565,0.468930024121,0.498685876053,0.476622409669,-0.422863959504,0.607072856755)
```

## Real UR5
For starting the program with the real robot, you need to start the "UR5_robot_control.launch" launch file. in this file you need to set the correct IP address. before you start the launch file. you can also uncomment the arena ( see package Ron Theelen below) for getting restrictions in you movement. 

if everything is installed correctly, you should be able to move the real robot arm with RViz. The you can start the "ur5.control.py" node. this node will work the same as in the simulation.

When you start using the real robot you need to follow this steps first: https://github.com/ThomasTimm/ur_modern_driver

Be aware that you need to change the ur_modern_driver/src/ur_hardware_interface.cpp to the ur_hardware_interface.cpp that is uploaded to this github. Copy it and paste it in your catkin_ws/src/ur_modern_driver/src. Don't forget to delete the old one.

```
roslaunch UR5CobotControler-master UR5_robot_control.launch
rosrun UR5CobotControler-master ur5_control.py
```

### Cobot safety project
When You want to add the full system you have to add the package for the simulation in Rviz made by Ron472: https://github.com/Ron472/cobot_visualisation . This package will make a workcell in your planner. when you post objects to this package you can get realtime objects in you workcell.
To make this project complete you have to add the .py file of StefanCals: https://github.com/StefanCals/ObjectDetection. This script uses a kinect camera to see object in space and by using the package on Ron, it is visible in RViz. 

# Background
When You look at the source code of the main node (ur5_control) you see I've made a class that contains waypoints. here it hold the position and orientation. 
In the main loop you can define waypoints and set the correct position. the mailoop also contains an array with all the waypoints that need to be executed. here you can add as many as you want.

if this is set, the main loop goes into a while loop where all waypoint are execuded. **important**: ``` arm.go(wait=False)``` will start moving the arm. wait is set to false. this means we can interrupt the arm at anytime. 
in this while loop we also check if needs to stop. this is done by listening to the topic ur5_contorl. when stop is printed there. the arm will stop.

for checking when the position is reached, we use the goal position of the arm and subtract that from the current position. if this is zero (within the given margin), then the arm will plan and move to the next waypoint.




