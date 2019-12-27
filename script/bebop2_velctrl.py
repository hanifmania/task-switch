#!/usr/bin/env python
# -*-coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs import Empty

ref_vel = Twist()
current_vel = Twist()
takeoff = 0
land = 0

def callbackVel(msg):
    global ref_vel
    ref_vel = msg

def callbackOdom(msg):
    global current_vel
    current_vel = msg.twist.twist

def callbackTakeoff(msg):
    global takeoff
    takeoff = 1

def callbackLand(msg):
    global land
    land = 1


def main():
    rospy.init_node("velocity_controller")
    rate = rospy.Rate(60)
    velsub = rospy.Subscriber("cmd_vel2", Twist, callbackVel)
    odomsub = rospy.Subscriber("odom", Odometry, callbackOdom)
    velpub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
    takeoffsub = rospy.Subscriber("takeoff", Empty, callbackTakeoff)
    landsub = rospy.Subscriber("land", Empty, callbackLand)

    while not rospy.is_shutdown():
        global ref_vel, current_vel, takeoff, land
        input_vel = Twist()

        if takeoff == 1 or land == 1:
            # velpub.publish(input_vel)
            pass
        else:
            # Caluculate control input by PID

            velpub.publish(input_vel)

        rate.sleep()


if __name__ == "__main__":
    main()