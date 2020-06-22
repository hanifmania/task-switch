#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Empty
import tf

import numpy as np
import cv2 as cv
import os

class coverageController():

    def __init__(self):
        # ROS Initialize
        rospy.init_node('coverageController', anonymous=True)

        self.listner = tf.TransformListener()
        # subscriber
        rospy.Subscriber("pose", PoseStamped, self.poseStampedCallback, queue_size=1)
        # publisher
        self.pub_twist = rospy.Publisher('cmd_input', Twist, queue_size=1)
        self.pub_takeoff = rospy.Publisher('takeoff', Empty, queue_size=1)
        self.pub_land = rospy.Publisher('land', Empty, queue_size=1)
        self.pub_reset = rospy.Publisher('reset', Empty, queue_size=1)


        #get_ROSparam
        self.clock = rospy.get_param("~clock",100)
        self.agentID = rospy.get_param("~agentID",1)

        # param initialize
        self.rate = rospy.Rate(self.clock)

        self.twist_from_controller = Twist()

        
        self.AgentNum = 2
        self.positions = np.zeros((self.AgentNum,3)) 


        
        rospy.loginfo("starting node")
        rospy.loginfo(self.agentID)
        
    def poseStampedCallback(self, pose_msg):
        self.position = np.array([pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z])
        self.orientation = np.array([pose_msg.pose.orientation.x, pose_msg.pose.orientation.y, \
                pose_msg.pose.orientation.z, pose_msg.pose.orientation.w])


    def update(self):
        if self.agentID == 1:
            self.twist_from_controller.linear.x = 0.1
            self.twist_from_controller.linear.y = 0.0
        else:
            self.twist_from_controller.linear.x = -0.1
            self.twist_from_controller.linear.y = 0.0

    

    def spin(self):
        while not rospy.is_shutdown():
            self.update()
            self.pub_twist.publish(self.twist_from_controller)
            listAgent = range(self.AgentNum)
            for i in listAgent:
                if i+1 == self.agentID:
                    pass
                else:
                    try:
                        agenttf = "/bebop10" + str(i+1) + "/rigidmotion"
                        (position, orientation) = self.listner.lookupTransform(
                            "/world",
                             agenttf,
                              rospy.Time(0))
                        # Save agent position
                        self.positions[i] = position
                    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                        continue
                rospy.loginfo(self.positions)
            self.rate.sleep()

    

        

if __name__ == '__main__':
    try:
        controller = coverageController()
        controller.spin()

    except rospy.ROSInterruptException: pass
