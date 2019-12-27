#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose

markerpub = rospy.Publisher("marker", Marker, queue_size=10)
red = green = blue = alpha = 0

def setCubeMarker(pose):

    global red, green, blue, alpha

    marker = Marker()

    marker.header.frame_id = "/world"
    marker.header.stamp = rospy.Time.now()

    marker.ns = "agent_pose"
    marker.id = 0

    marker.action = Marker.ADD

    marker.pose = pose

    marker.color.r = red
    marker.color.g = green
    marker.color.b = blue
    marker.color.a = alpha

    marker.scale.x = 0.3
    marker.scale.y = 0.3
    marker.scale.z = 0.1

    marker.lifetime = rospy.Duration()
    marker.type = Marker.CUBE

    return marker


def poseCallback(msg):
    marker = setCubeMarker(msg)
    markerpub.publish(marker)

def main():
    rospy.init_node("posemarker")
    posesub = rospy.Subscriber("pose", Pose, poseCallback)

    # Get marker color parameters
    global red, green, blue, alpha
    red = rospy.get_param("~red", default=1.0)
    green = rospy.get_param("~green", default=0.0)
    blue = rospy.get_param("~blue", default=0.0)
    alpha = rospy.get_param("~alpha", default=1.0)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down posemarker")

if __name__ == '__main__':
    main()
