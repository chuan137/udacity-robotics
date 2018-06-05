#!/usr/bin/env python

# Copyright (C) 2017 Udacity Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *
from FK import calc_Rotation_0_3, calc_Rotation_rpy 

_DEBUG = true

def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()


	    # Extract end-effector position and orientation from request
	    # px,py,pz = end-effector position
	    # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])

            if _DEBUG:
                rospy.loginfo("Points: %3.6f %3.6f %3.6f" % (px, py, pz))
                rospy.loginfo("Angles: %3.6f %3.6f %3.6f %3.6f" % \
                                (req.poses[x].orientation.x, \
                                 req.poses[x].orientation.y, \
                                 req.poses[x].orientation.z, \
                                 req.poses[x].orientation.w))

        ### Your IK code here
        # Extract rotation from base frame to EE, and positions of EE and Wrist Center
            R = calc_Rotation_rpy(roll, pitch, yaw)
            EE = Matrix([[px], [py], [pz]])
            WC = EE - 0.303 * R[:,2]

	    # Calculate joint angles using Geometric IK method
            # theta1
            theta1 = atan2(WC[1], WC[0])

            # theta2, theta3
            side_a = sqrt(1.50**2 + 0.054**2)
            side_b_r = sqrt(WC[0]**2 + WC[1]**2) - 0.35
            side_b_z = WC[2] - 0.75
            side_b = sqrt(side_b_r**2 + side_b_z**2)
            side_c = 1.25

            if side_b > side_a + side_c:
                # avoid failures in angle_a and angle_b
                angle_a = 0
                angle_b = pi
            else:
                angle_a = acos((side_b**2 + side_c**2 - side_a**2) / (2*side_b*side_c))
                angle_b = acos((side_a**2 + side_c**2 - side_b**2) / (2*side_a*side_c))

            theta2 = pi/2 - angle_a - atan2(side_b_z, side_b_r)
            theta3 = pi/2 - angle_b - atan2(0.054, 1.50)

            # theta4, theta5, theta6 
            R0_3 = calc_Rotation_0_3(theta1, theta2, theta3)
            R3_6 = R0_3.inv() * R
            
            #theta5 = acos(R3_6[1,2])
            theta5 = atan2(sqrt(R3_6[0,2]**2 + R3_6[2,2]**2), R3_6[1,2])
            theta6 = atan2(-R3_6[1,1], R3_6[1,0])
            if sin(theta5) > 0:
                theta4 = atan2(R3_6[2,2], -R3_6[0,2])
            else:
                theta4 = atan2(-R3_6[2,2], R3_6[0,2])

            if _DEBUG:
                rospy.loginfo("thetas: %04.8f %04.8f %04.8f %04.8f %04.8f %04.8f" % \
                    (theta1, theta2, theta3, theta4, theta5, theta6) )
                rospy.loginfo("")
        ###

            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
	    joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
	    joint_trajectory_list.append(joint_trajectory_point)

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()
