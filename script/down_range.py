#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Create Range message from agent tf and camera parameters.

import rospy
import numpy as np
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Range
from task_switch.msg import Cameraparam
import tf

# r = 3.7 * 10 ** (-3) / 2
# lam = 5.0 * 10 ** (-3)
# phi = np.arctan(r/lam)
r = lam = phi = 0

def down_camera_tf(agenttf, cameratf):
    #####
    # Broadcast down camera tf relative to agent tf to generate frame_id of Range message.
    ####

    position = (0.0, 0.0, 0.0)
    camera_orientation = tf.transformations.quaternion_from_euler(0, np.pi/2, 0, "rxyz")
    # rospy.loginfo("{}".format(camera_orientation))

    # Broadcast tf
    broadcaster = tf.TransformBroadcaster()

    now = rospy.Time.now()
    broadcaster.sendTransform(position, camera_orientation, now, cameratf, agenttf)

def callbackCamera(msg):
    global r, lam, phi
    r = msg.radius
    lam = msg.lam
    phi = msg.phi

def createRange(phi, z, cameratf):

    downcamera_range = Range()
    downcamera_range.header.stamp = rospy.Time.now()
    downcamera_range.header.frame_id = cameratf
    downcamera_range.radiation_type = Range.INFRARED
    downcamera_range.field_of_view = 2*phi
    downcamera_range.field_of_view = 2*0.53
    downcamera_range.min_range = 0.2
    downcamera_range.max_range = 4.0
    downcamera_range.range = z

    return downcamera_range

def main():
    rospy.init_node("down_range")
    rangepub = rospy.Publisher("range", Range, queue_size=10)
    cameraparam_sub = rospy.Subscriber("cameraparam", Cameraparam, callbackCamera)
    agentlistner = tf.TransformListener()
    rate = rospy.Rate(60)

    # Get target frame names from parameter server
    agenttf = rospy.get_param("~agenttf", default="agent")
    cameratf = rospy.get_param("~cameratf", default="downcamera")

    while not rospy.is_shutdown():
        down_camera_tf(agenttf, cameratf)

        try:
            (position, orientation) = agentlistner.lookupTransform("/world", agenttf, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        global r, lam, phi
        downcamera_range = createRange(phi, position[2], cameratf)

        rangepub.publish(downcamera_range)
        rate.sleep()

if __name__ == "__main__":
    main()
