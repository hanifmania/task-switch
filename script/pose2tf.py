#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from geometry_msgs.msg import Pose, PoseStamped
import tf

def poseCallback(msg):

    global tfname

    broadcaster = tf.TransformBroadcaster()

    position = (msg.position.x, msg.position.y, msg.position.z)
    orientation = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
    now = rospy.Time.now()

    broadcaster.sendTransform(position, orientation, now, tfname, 'world')

def PoseStampedCallback(msg):

    global tfname

    broadcaster = tf.TransformBroadcaster()
    position = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
    orientation = (msg.pose.orientation.x, msg.pose.orientation.y,
                        msg.pose.orientation.z, msg.pose.orientation.w)
    frame_id = msg.header.frame_id
    now = rospy.Time.now()

    broadcaster.sendTransform(position, orientation, now, tfname, frame_id)

def main():
    rospy.init_node("pose2tf")
    posesub = rospy.Subscriber("pose", Pose, poseCallback)
    posestampedsub = rospy.Subscriber("posestamped", PoseStamped, PoseStampedCallback)

    global tfname
    tfname = rospy.get_param("~tfname", default="agent")

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down pose2tf")

if __name__ == '__main__':
    main()
