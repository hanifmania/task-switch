#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

velpub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
velocity = Twist()

vel_gain = 0.0
omega_gain = 0.0

def joyCallback(msg):
    global vel_gain, omega_gain
    velocity.linear.x = -vel_gain * msg.axes[0]
    velocity.linear.y = vel_gain * msg.axes[1]
    velocity.linear.z = vel_gain * msg.axes[4]
    velocity.angular.z = omega_gain * (msg.axes[5] - msg.axes[2])/2
    velpub.publish(velocity)

def main():
    rospy.init_node('joy2vel', disable_signals=True)
    joysub = rospy.Subscriber('/joy', Joy, joyCallback)
    global vel_gain, omega_gain
    vel_gain = rospy.get_param("~vel_gain", default=1)
    omega_gain = rospy.get_param("~omega_gain", default=1)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down joy2vel node")

if __name__ == "__main__":
    main()
