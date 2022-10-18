#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import rospy
from geometry_msgs.msg import Twist,TwistStamped,Pose,PoseStamped
import tf

from se3_operations import *
from transformations import *

def velocity_saturator(vel6, linear_max=3, angular_max=3):
    linear_norm = np.linalg.norm(vel6[0:3], ord=2)
    angular_norm = np.linalg.norm(vel6[3:6], ord=2)
    if linear_norm > linear_max:
        vel6[0:3] = vel6[0:3] * (linear_max/linear_norm)
    if angular_norm > angular_max:
        vel6[3:6] = vel6[3:6] * (angular_max/angular_norm)
    return vel6

def twist_saturator(twist, linear_max=3, angular_max=3):
    return vec6_to_twist(velocity_saturator(twist_to_vec6(twist), linear_max, angular_max))

def vec6_to_twiststamped(vec6,time):
    msg = TwistStamped()
    msg.header.stamp = time
    msg.twist.linear.x = vec6[0]
    msg.twist.linear.y = vec6[1]
    msg.twist.linear.z = vec6[2]
    msg.twist.angular.x = vec6[3]
    msg.twist.angular.y = vec6[4]
    msg.twist.angular.z = vec6[5]
    return msg

def vec6_to_twist(vec6):
    msg = Twist()
    msg.linear.x = vec6[0]
    msg.linear.y = vec6[1]
    msg.linear.z = vec6[2]
    msg.angular.x = vec6[3]
    msg.angular.y = vec6[4]
    msg.angular.z = vec6[5]
    return msg

def twist_to_vec6(twist):
    vec6 = np.zeros((6,1))
    vec6[0] = twist.linear.x
    vec6[1] = twist.linear.y
    vec6[2] = twist.linear.z
    vec6[3] = twist.angular.x
    vec6[4] = twist.angular.y
    vec6[5] = twist.angular.z
    return vec6

def g_to_pose(g_mat):
    pose_msg =PoseStamped()
    pos_vec = g_mat[0:3,3:4].T.tolist()[0]
    R_mat = np.eye(4)
    R_mat[0:3,0:3] = g_mat[0:3,0:3]
    orientation = quaternion_from_matrix(R_mat)

    pose_msg.header.stamp = rospy.Time.now()
    pose_msg.pose.position.x = pos_vec[0]
    pose_msg.pose.position.y = pos_vec[1]
    pose_msg.pose.position.z = pos_vec[2]
    pose_msg.pose.orientation.x = orientation[0]
    pose_msg.pose.orientation.y = orientation[1]
    pose_msg.pose.orientation.z = orientation[2]
    pose_msg.pose.orientation.w = orientation[3]
    return pose_msg

def g_to_posestamped(g_mat):
    pose_msg =PoseStamped()
    pos_vec = g_mat[0:3,3:4].T.tolist()[0]
    R_mat = np.eye(4)
    R_mat[0:3,0:3] = g_mat[0:3,0:3]
    orientation = quaternion_from_matrix(R_mat)

    pose_msg.header.stamp = rospy.Time.now()
    pose_msg.pose.position.x = pos_vec[0]
    pose_msg.pose.position.y = pos_vec[1]
    pose_msg.pose.position.z = pos_vec[2]
    pose_msg.pose.orientation.x = orientation[0]
    pose_msg.pose.orientation.y = orientation[1]
    pose_msg.pose.orientation.z = orientation[2]
    pose_msg.pose.orientation.w = orientation[3]
    return pose_msg

def pose_to_g(pose_msg):
    pos = pose_msg.position
    ori = pose_msg.orientation
    vec3 = np.c_[np.array([pos.x, pos.y, pos.z])]
    R44 = quaternion_matrix([ori.x, ori.y, ori.z, ori.w])
    R44[0:3,3:4] = vec3
    return R44

def tf_to_gmat(tf_listner, parent, child):
    try:
        (trans,rot) = tf_listner.lookupTransform(child, parent, rospy.Time())
        gmat = np.eye(4)
        gmat = quaternion_matrix(rot)
        gmat[0:3,3:4] = np.c_[np.array(trans)]
        return gmat
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        rospy.logwarn(e)
        return None

    
    


