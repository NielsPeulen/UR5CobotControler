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

#global for arm data and stop data
global arm
global controlData

#Callback for getStop info
def callback(data):
    global controlData
    global arm
    controlData = data
    print data
    if controlData.data == "stop":
        arm.stop()
        pass

#define Class for waypoints 
class waypoint():
    #make waypoints values
    def __init__(self, oriX, oriY, oriZ, oriW, posX, posY, posZ):
        self.oriX = oriX
        self.oriY = oriY
        self.oriZ = oriZ
        self.oriW = oriW
        self.posX = posX
        self.posY = posY
        self.posZ = posZ

# Start planning arm
def StartPlanning(position):
    global arm
    #clear target
    print   position.oriX
    arm.clear_pose_targets()
    #start planning plan 1
    print "\n\n-------------- start planning -------------------------\n"
    pose_target = geometry_msgs.msg.Pose()
    pose_target.orientation.x = position.oriX
    pose_target.orientation.y = position.oriY
    pose_target.orientation.z = position.oriZ
    pose_target.orientation.w = position.oriW

    pose_target.position.x = position.posX    
    pose_target.position.y = position.posY
    pose_target.position.z = position.posZ
    arm.set_pose_target(pose_target)
    #print pose_target
    plan1 = arm.plan()

    print "\n\n-------------- start visualizing plan -------------------------\n"
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()

    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan1)
    display_trajectory_publisher.publish(display_trajectory);

#check if waypoint is reached
def checkPos(position):
    global arm
    goalreached = False
    currentPos = arm.get_current_pose()
    #margin for getting to position
    margin = 0.005
    #Be aware: not checking orientation
    if currentPos.pose.position.x - position.posX  <= margin and currentPos.pose.position.y - position.posY  <= margin  and currentPos.pose.position.z - position.posZ  <= margin:
        if currentPos.pose.position.x - position.posX  >= -margin and currentPos.pose.position.y - position.posY  >= -margin  and currentPos.pose.position.z - position.posZ  >= -margin:
            goalreached = True
        return goalreached
        pass
    pass
pass   

#main loop
if __name__ == '__main__': 
    global controlData
    controlData = String()
    global arm

    #subsribe for feedback

    rospy.Subscriber("ur5_control", String, callback)
    #arm_pose = rospy.Publisher('arm_pose', string, quee_size=10)

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
    arm.set_max_velocity_scaling_factor(0.5)
    arm.set_max_acceleration_scaling_factor(0.5)
    
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

    #in simulation
    # values order by: oriX, oriY, oriZ, oriW, posX, posY, posZ
    # wphome  = waypoint(0,0.707,0,0.707,0.5,0.24,0.27)
    # wp1move = waypoint(0,0.707,0,0.707,0.3,0.44,0.27)
    # wp2pick = waypoint(0,0.707,0,0.707,0.3,0.44,0.2)
    # wp3up   = waypoint(0,0.707,0,0.707,0.3,0.44,0.27)
    # wp4move = waypoint(0,0.707,0,0.707,-0.1,0.5,0.27)
    # wp5place = waypoint(0,0.707,0,0.707,-0.1,0.5,0.20)
    # wp6up   = waypoint(0,0.707,0,0.707,-0.1,0.5,0.27)

    # For real 
    # values order by: oriX, oriY, oriZ, oriW, posX, posY, posZ
    wphome  = waypoint(-0.528360791026,-0.50224685565,0.468930024121,0.498685876053,0.476622409669,-0.422863959504,0.607072856755)
    wp1move = waypoint(-0.528360791026,-0.50224685565,0.468930024121,0.498685876053,0.0687504253905,-0.744506996949,0.471566043034)
    wp2pick = waypoint(-0.528360791026,-0.50224685565,0.468930024121,0.498685876053,0.0687504253905,-0.744506996949,0.183653174407)
    wp3up   = waypoint(-0.528360791026,-0.50224685565,0.468930024121,0.498685876053,0.0687504253905,-0.744506996949,0.471566043034)
    wp4move = waypoint(-0.528360791026,-0.50224685565,0.468930024121,0.498685876053,0.476622409669,-0.422863959504,0.471566043034)
    wp5place = waypoint(-0.528360791026,-0.50224685565,0.468930024121,0.498685876053,0.476622409669,-0.422863959504,0.183653174407)
    wp6up   = waypoint(-0.528360791026,-0.50224685565,0.468930024121,0.498685876053,0.476622409669,-0.422863959504,0.471566043034)

    print "-------------- wait for start --------------"
    print "-------------- rostopic pub /UR5_stop std_msgs/String---"

    moving = False
    curWaypoint = 0
    oldwaypoint = -1
    # set waypoints in array
    waypointlist=[]
    waypointlist.append(wphome)
    waypointlist.append(wp1move)
    waypointlist.append(wp2pick)
    waypointlist.append(wp3up)
    waypointlist.append(wp4move)
    waypointlist.append(wp5place)
    waypointlist.append(wp6up)
    print controlData.data
    while True:
        #stop
        if controlData.data == "stop":
            arm.stop()
            print "stop 1"
            rospy.sleep(0.5)
            oldwaypoint = curWaypoint -1
            moving = False
            pass
        #if no stop the run all waypoints
        else:
            print curWaypoint, oldwaypoint, moving
            #check if nieuw waypoint
            if oldwaypoint < curWaypoint:
                waypointreached = False
                StartPlanning(waypointlist[curWaypoint])
                rospy.Subscriber("UR5_stop", String, callback)
                #Start moving
                if moving == False:
                    print "start moving!!!!"
                    arm.go(wait=False)
                    moving = True
                    pass
                print "Still moving"
                oldwaypoint += 1
                pass
            # if position is reached, go next waypoint
            if checkPos(waypointlist[curWaypoint]) == True:
                curWaypoint += 1
                moving = False
                print ("waypoint is:%s", curWaypoint)
                pass
            if curWaypoint == 7:
                curWaypoint = 0
                oldwaypoint = -1
                pass
            pass
        pass

