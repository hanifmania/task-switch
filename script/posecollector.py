#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from visualization_msgs.msg import Marker
from std_msgs.msg import Float32MultiArray, ColorRGBA
from geometry_msgs.msg import Pose, PoseStamped
from geometry_msgs.msg import PoseArray
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from task_switch.voronoi_main import Voronoi
from task_switch.voronoi_main import Field


import tf
import time
import numpy as np
import cv2 as cv
import warnings

from cv_bridge import CvBridge


class Collector:
    def __init__(self, agentName):
        self.ready = False

        topicName = rospy.get_param("~posestampedTopic")
        preTopicName = rospy.get_param("~preTopicName", "/")
        subTopic = preTopicName + agentName + topicName
        rospy.loginfo("topicName:" + subTopic)
        # subscriber for each agent's region
        rospy.Subscriber(subTopic, PoseStamped, self.poseStampedCallback, queue_size=1)
        # initialze with zeros
        self.pose = Pose()

    def poseStampedCallback(self, msg_data):
        if self.ready == False:
            self.ready = True
        self.pose = msg_data.pose

    def getPose(self):
        return self.pose

    def getReady(self):
        return self.ready


class poseCollector:
    def __init__(self):
        # ROS Initialize
        rospy.init_node("poseCollector", anonymous=True)

        # Number of Agents
        self.agentNum = rospy.get_param("/agentNum", 1)

        self.Collectors = []
        # create [Agent's number] subscriber
        for agentID in range(self.agentNum):
            agentName = "bebop10" + str(agentID + 1)
            collector = Collector(agentName)
            self.Collectors.append(collector)

        self.pub_allPose = rospy.Publisher("/allPose", PoseArray, queue_size=1)
        # node freq
        self.clock = rospy.get_param("/clock/posecollector")
        self.rate = rospy.Rate(self.clock)

    def spin(self):

        while not rospy.is_shutdown():
            poselist = []
            ready = True
            for agentID in range(self.agentNum):
                pose = self.Collectors[agentID].getPose()
                poselist.append(pose)
                ready = ready * self.Collectors[agentID].getReady()

            if ready:
                self.pub_allPose.publish(PoseArray(poses=poselist))

            self.rate.sleep()


if __name__ == "__main__":
    try:
        posecollector = poseCollector()
        posecollector.spin()

    except rospy.ROSInterruptException:
        pass
