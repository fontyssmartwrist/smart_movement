#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import trajectory_msgs.msg 
import random
from math import pi
from sensor_msgs.msg import JointState
from smart_movement.msg import codesys_joint, CurrentJointValues, smart_joint
from std_msgs.msg import Float32MultiArray

class GeneratePathWithMoveIt():
    def __init__(self):
        # Initialize moveit_commandor
        moveit_commander.roscpp_initialize(sys.argv)
        # Create a node 
        rospy.init_node('moveit_command_centre')
        # Setup control group of joints
        self.group = moveit_commander.MoveGroupCommander("smart_arm")
        # Read incoming data
        self.goal_listen = rospy.Subscriber('/goal_smart_wrist', geometry_msgs.msg.Pose, self.goal_received)
        # Publish the codesys waypoints to comunication node
        self.give_waypoints = rospy.Publisher('/codesys_waypoints', codesys_joint, queue_size=10)
        # Get the real joint values from codesys
        self.real_listen = rospy.Subscriber('/current_joint_values', CurrentJointValues, self.set_current_pos)
        # Send current pose to demo
        self.give_current_pose = rospy.Publisher('/move_group/fake_controller_joint_states', JointState, queue_size=10)
        # Get joint goal from the web
        self.web_goal = rospy.Subscriber('/web_goal', Float32MultiArray, self.go_to_joint_state)
        # Get 2d slider goal from the web control
        self.web_leveled_goal = rospy.Subscriber('/web_leveled_goal', geometry_msgs.msg.Point, self.translate_message)
        # Testing how multigoal scenes work
        #self.go_to_multigoal()
        self.current_joint_info()
        # Keep the script running
        rospy.spin()

    def goal_received(self, goal_data):
        # Store incoming data into angels of goal
        self.data_of_goal = goal_data
        # Make create a path to the goal
        self.go_to_pose_goal(self.data_of_goal)

    def translate_message(self, goal_data):
        # Store the Point message into a single list
        xyz = [goal_data.x, goal_data.y, goal_data.z]
        self.go_to_leveled(xyz)

    def set_current_pos(self, joint_data):
        # Build the current position message
        self.current_pos = JointState()
        self.current_pos.header.seq = 99999
        self.current_pos.header.stamp = rospy.Time.now()
        self.current_pos.header.frame_id = "/world"
        self.current_pos.name = ["end_link", "joint0", "joint1", "joint3", "joint4"]
        self.current_pos.position = [0.0, joint_data.theta0, joint_data.theta1, joint_data.theta2, joint_data.theta3]
        rospy.loginfo("Joint values were set manually")
        # Publish the current position to the simulation
        self.give_current_pose.publish(self.current_pos)

    def go_to_position(self, xyz):
        # Log info about the goal
        rospy.loginfo("Going to postition: {}".format(xyz))
        # Set the position target 
        self.group.set_position_target(xyz)
        # Command moveit to calculate a plan with the set goal and store it
        self.plan_of_wrist = self.group.plan()
        # Send the path to the codesys software
        self.send_goal()
        # Call the MoveIt! commander to compute the plan and execute it.
        self.group.go(wait=True) # The program holds until pose is reached

    def go_to_leveled(self,xyz):   
        # Give pose_goal the correct message type
        pose_goal = geometry_msgs.msg.Pose()
        # Insert the position to the pose goal
        pose_goal.position.x = xyz[0]
        pose_goal.position.y = xyz[1]
        pose_goal.position.z = xyz[2]
        # Add the missing leveling orientation
        pose_goal.orientation.x = 0.0
        pose_goal.orientation.y = 0.0
        pose_goal.orientation.z = 0.0
        pose_goal.orientation.w = 1.0
        # Log info about the goal
        rospy.loginfo("Going to leveled position:\n" + str(pose_goal.position))
        # Set the pose target with the created pose
        self.group.set_pose_target(pose_goal)
        # Command moveit to calculate a plan with the set goal and store it
        self.plan_of_wrist = self.group.plan()
        # Send the path to the codesys software
        self.send_goal()
        # Call the MoveIt! commander to compute the plan and execute it.
        self.group.go(wait=True) # The program holds until pose is reached
        # It is always good to clear your targets after planning with poses.
        self.group.clear_pose_targets()

    def go_to_pose_goal(self, pose_goal):
        # Set the pose target with the send pose
        self.group.set_pose_target(pose_goal)
        # Command moveit to calculate a plan with the set goal and store it
        self.plan_of_wrist = self.group.plan()
        rospy.loginfo(self.plan_of_wrist)
        # Send the path to the codesys software
        self.send_goal()
        # Now, we call the planner to compute the plan and execute it.
        self.plan = self.group.go(wait=True)
        # It is always good to clear your targets after planning with poses.
        self.group.clear_pose_targets()

    def go_to_joint_state(self, theta):      
        # Set joint goal message type
        self.joint_goal = self.group.get_current_joint_values()
        # Set each joint to the correct value
        self.joint_goal[0] = theta.data[0]
        self.joint_goal[1] = theta.data[1]
        self.joint_goal[2] = theta.data[2]
        self.joint_goal[3] = theta.data[3]
        self.joint_goal[4] = 0.0
        # Make a plan to move to the next goal
        self.plan_of_wrist = self.group.plan(self.joint_goal)
        # Send waypoints of the goal
        self.send_goal()
        # Execute moving to the joint until movement is reached
        self.group.go(self.joint_goal, wait=True)

    def send_goal(self):
        # Get information of the path
        amount_of_waypoints = len(self.plan_of_wrist.joint_trajectory.points)
        # Set variable to correct message type
        servo_waypoints = codesys_joint()
        # Set the amount of waypoints
        servo_waypoints.waypoints = amount_of_waypoints
        # Create empty lists to store the data
        servo_waypoints.theta0 = [0] * amount_of_waypoints
        servo_waypoints.theta1 = [0] * amount_of_waypoints
        servo_waypoints.theta2 = [0] * amount_of_waypoints
        servo_waypoints.theta3 = [0] * amount_of_waypoints
        # Loop for storing the trajectory in servo angles used in codesys, every cycle stores one waypoint
        for nb_of_waypoints in range(0, amount_of_waypoints):
            # Store the joint values in temporary variables for clearity
            moveit_waypoint_theta0 = self.plan_of_wrist.joint_trajectory.points[nb_of_waypoints].positions[0]
            moveit_waypoint_theta1 = self.plan_of_wrist.joint_trajectory.points[nb_of_waypoints].positions[1] 
            moveit_waypoint_theta2 = self.plan_of_wrist.joint_trajectory.points[nb_of_waypoints].positions[2]
            moveit_waypoint_theta3 = self.plan_of_wrist.joint_trajectory.points[nb_of_waypoints].positions[3] 
            # Translate the float64 points between -pi and pi to integer values used by the codesys 
            # and store them 
            servo_waypoints.theta0[nb_of_waypoints] = int((moveit_waypoint_theta0 / pi) * 3140)
            servo_waypoints.theta1[nb_of_waypoints] = int((moveit_waypoint_theta1 / pi) * 3140)
            servo_waypoints.theta2[nb_of_waypoints] = int((moveit_waypoint_theta2 / pi) * 3140)
            servo_waypoints.theta3[nb_of_waypoints] = int((moveit_waypoint_theta3 / pi) * 3140)
        # Pservo_waypoints to check if message was correct
        print servo_waypoints
        # Publish list of waypoints to /codesys_waypoints
        self.give_waypoints.publish(servo_waypoints)

    def current_joint_info(self):
        safe = self.group.get_current_joint_values()
        rospy.loginfo(safe)

if __name__ == '__main__':
    try:
        # Start the main class
        GeneratePathWithMoveIt()
        
    except rospy.ROSInitException:
        rospy.loginfo("node terminated.")
        pass