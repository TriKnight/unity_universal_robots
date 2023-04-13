#!/usr/bin/env python3

########### Basic Postion Joint Control UR Robot ###################
# ---------------------------------------------------------------------
# This code use Ros Action help the UR robot move to desirable goal.
# Author: Tri Knight
# ---------------------------------------------------------------------

import rospy
import numpy as np
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from unity_universal_robots.msg import URMoveitJoints


# Define the joint name of the UR10e robot.
JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]
# The scaled versions should be preferred over the non-scaled versions.
JOINT_TRAJECTORY_CONTROLLERS = [
    "scaled_pos_joint_traj_controller", # for the real-robot
    "pos_joint_traj_controller", # for the sim-robot

]

class UnityPosControl:
    """ This is test the joint robot trajectories in simulation and real robot"""
    
    def __init__(self):
        rospy.init_node("test_trajectories")
        self.lastposition = []
        self.pose_np =[]
        self.ur_position=[]
        # For simulation robot
        # self.joint_trajectory_controller = JOINT_TRAJECTORY_CONTROLLERS[1]
        # For real robot 
        self.joint_trajectory_controller = JOINT_TRAJECTORY_CONTROLLERS[0]

        ## ROS Subscriber Topic
        # Subscribe to the ar_pose_marker topic to get the image width and height
        self.ur_sub_joints = rospy.Subscriber('ur10e_joints', URMoveitJoints, self.callback_joints)
        
        ## ROS parameters
        self.rate = rospy.Rate(1) # Hz
        # The main program here
        self.Position_control()
        #rospy.spin()
        

    def callback_joints(self, msg):
        ## Try to get the joint states msg from Unity
        self.ur_position = msg.joints
        self.send_joint_trajectory() 
   
    
    ## Send Position to the real robot
    def send_joint_trajectory(self):
        """ Creates a trajectories and send it to the robot"""
        trajectory_client = actionlib.SimpleActionClient(
            "{}/follow_joint_trajectory".format(self.joint_trajectory_controller),
            FollowJointTrajectoryAction,
        )
        # Wait for the server to start up and start listening for goals.
        trajectory_client.wait_for_server()
        #create and fill trajectory the goal
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = JOINT_NAMES
        #Create the list of the joint angles in radian
        position_list = self.ur_position
        
        point = JointTrajectoryPoint()
        point.positions = position_list
        point.time_from_start = rospy.Duration(5)
        goal.trajectory.points.append(point)

        rospy.logwarn("Robot begin moving..............")
        rospy.loginfo("The robot will move to the following waypoints: \n{}".format(position_list))
        trajectory_client.send_goal(goal)
        trajectory_client.wait_for_result()
        result = trajectory_client.get_result()
        rospy.loginfo("Trajectory execution finished in state {}".format(result.error_code))

    ## This is main program:
    def Position_control(self):
        """ Control the robot"""
        while not rospy.is_shutdown():
            self.rate.sleep()
        rospy.signal_shutdown("We are done here!")

if __name__ == '__main__':
    try:
        client = UnityPosControl()
        client.Position_control()
    except rospy.ROSInterruptException:
        pass
    

