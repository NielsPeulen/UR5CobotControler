#!/usr/bin/env python
'''ur5_control ROS Node'''
# license removed for brevity
import sys
from copy import deepcopy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from actionlib_msgs.msg import GoalStatusArray
from std_msgs.msg import String



if __name__ == '__main__': 
    #innitialize moveit_commander and rospy
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('ur5_control', anonymous=True)

    rospy.loginfo("\n\n-------------- innitialize complete -------------------------\n")
    #interface whole robot:
    robot = moveit_commander.RobotCommander()
    # rospy.loginfo("Robot: %s", robot)
    #interface object surroundings
    # scene = moveit_commander.PlanningSceneInterface()
    # rospy.loginfo("Scene: %s", scene)
    # print"Robot: %s, Scene: %s "%(robot, scene)

    #interface object group of joints
    arm = moveit_commander.MoveGroupCommander('manipulator')
    rospy.loginfo("\n\n-------------- object defined-------------------------\n")

    #set reference frame for pose targets
    reference_frame = "/base_link"
    print"reference_frame: %s"% reference_frame

    #create publisher
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)
    print"trajectory publisher: %s"% display_trajectory_publisher
    print "-------------- waiting for RVIZ.. -------------------------"
    rospy.sleep(1)  #needs to be 10
    #print robot information
    print "-------------- Reference frame: %s -------------------------" % arm.get_planning_frame()
    print "-------------- End effector: %s -------------------------" % arm.get_end_effector_link()
    
    while True:
       print arm.get_current_pose()
       rospy.sleep(1)
       pass

