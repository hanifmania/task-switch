#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from geometry_msgs.msg import Twist, Pose, PoseStamped, Point
import tf

# reference = [0, 0, 0]
# ref_flag = 0
body_linear_vel = np.array([0, 0, 0])
body_angular_vel = np.array([0, 0, 0])

def velCallback(msg):
    global body_angular_vel
    global body_linear_vel
    body_linear_vel = np.array([msg.linear.x, msg.linear.y, msg.linear.z])
    body_angular_vel = np.array([msg.angular.x, msg.angular.y, msg.angular.z])

# def refCallback(msg):
#     global reference
#     global ref_flag
#     reference[0] = msg.x
#     reference[1] = msg.y
#     reference[2] = msg.z
#     ref_flag = 1
#
def wedge(u):
    u_wedge = np.array([[0, -u[2], u[1]],
                        [u[2], 0, -u[0]],
                        [-u[1], u[0], 0]])
    return u_wedge

def vel_wedge(v, w):
    V_wedge = np.array([[0,-w[2], w[1], v[0]],
                        [w[2], 0, -w[0], v[1]],
                        [-w[1], w[0], 0, v[2]],
                        [0, 0, 0, 0]])
    return V_wedge

def main():
    rospy.init_node('rigid_body', disable_signals=True)
    vel_sub = rospy.Subscriber('cmd_vel', Twist, velCallback)
    # reference_sub = rospy.Subscriber('reference', Point, refCallback)
    posestamped_pub = rospy.Publisher('pose', PoseStamped, queue_size=1)
    listener = tf.TransformListener()
    broadcaster = tf.TransformBroadcaster()
    rate = rospy.Rate(60)
    dt = 1.0 / 60
    # Get tfname published to
    tfname = rospy.get_param('~name_space', default='rigid_body')
    x = rospy.get_param("~x", default="0.0")
    y = rospy.get_param("~y", default="0.0")
    z = rospy.get_param("~z", default="0.0")
    ginit = [x,y,z]
    
    global body_linear_vel
    global body_angular_vel

    # Initialize rigid body
    g = np.eye(4)
    g[0:3,3:4] = np.c_[np.array(ginit)]
    gdot = gdot_old = np.zeros((4, 4))
    lasttime = rospy.Time.now()
    pose = Pose()
    posestamped = PoseStamped()

    while not rospy.is_shutdown():

        # Caluculate next pose of rigid body
        now = rospy.Time.now()
        # dt = now - lasttime
        Vb = vel_wedge(body_linear_vel, body_angular_vel)
        gdot = np.dot(g, Vb)
        # 台数積分
        g += (gdot + gdot_old) * dt / 2
        gdot_old = gdot
        lasttime = now

        # # if reference is published, move to there
        # global ref_flag
        # global reference
        # if ref_flag == 1:
        #     g[0, 3] = reference[0]
        #     g[1, 3] = reference[1]
        #     g[2, 3] = reference[2]
        #     ref_flag = 0
        #
        # # Projection of rotation matrix to SO(3)
        rotations = tf.transformations.rotation_from_matrix(g)
        projected_R = tf.transformations.rotation_matrix(rotations[0], rotations[1], rotations[2])
        g[0:3, 0:3] = projected_R[0:3, 0:3]

        position = g[0:3, 3:4].T.tolist()[0]
        orientation = tf.transformations.quaternion_from_matrix(projected_R)

        # Broadcast tf
        broadcaster.sendTransform(position, orientation, now, tfname, '/world')

        # Publish PoseStamped to topic
        pose.position.x = position[0]
        pose.position.y = position[1]
        pose.position.z = position[2]
        pose.orientation.x = orientation[0]
        pose.orientation.y = orientation[1]
        pose.orientation.z = orientation[2]
        pose.orientation.w = orientation[3]
        posestamped.pose = pose
        posestamped_pub.publish(posestamped)

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException: pass
