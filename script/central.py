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

class Collector():
    
    def __init__(self,agentName):
        subTopic = agentName + "/rigidmotion/pose"
        rospy.Subscriber(subTopic, PoseStamped, self.poseStampedCallback, queue_size=1)

    def poseStampedCallback(self, pose_msg):
        self.position = np.array([pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z])
        self.orientation = np.array([pose_msg.pose.orientation.x, pose_msg.pose.orientation.y, \
                pose_msg.pose.orientation.z, pose_msg.pose.orientation.w])

    def getPos(self):
        return self.position

class central():

    def __init__(self):
        # ROS Initialize
        rospy.init_node('centoral computer', anonymous=True)

        Collectors = []
        # subscriber
        agentList = ["bebop101","bebop102","bebop103"]
        for agentName in agentList
            collector = Collector(agentName) 
            Collectors.append(collector)
            
        # publisher
        self.pub_poseArray = rospy.Publisher('/listPoses', PoseArray, queue_size=1)


        #get_ROSparam
        self.clock = rospy.get_param("~clock",100)

        # param initialize
        self.rate = rospy.Rate(self.clock)

        self.twist_from_controller = Twist()

        



        rospy.loginfo("starting node")
        rospy.loginfo(self.agentID)
        
    def poseStampedCallback(self, pose_msg):
        self.position = np.array([pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z])
        self.orientation = np.array([pose_msg.pose.orientation.x, pose_msg.pose.orientation.y, \
                pose_msg.pose.orientation.z, pose_msg.pose.orientation.w])

    def update(self):
        self.twist_from_controller.linear.x = 0.1
        self.twist_from_controller.linear.y = 0.0

    

    def spin(self):
        while not rospy.is_shutdown():
            self.update()
            self.pub_twist.publish(self.twist_from_controller)
            self.rate.sleep()

    

        

if __name__ == '__main__':
    try:
        controller = coverageController()
        controller.spin()

    except rospy.ROSInterruptException: pass
