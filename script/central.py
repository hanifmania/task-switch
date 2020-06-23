#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseArray
from task_switch.voronoi_main import Voronoi
from std_msgs.msg import Empty
from std_msgs.msg import Float32MultiArray,Int8MultiArray,MultiArrayLayout,MultiArrayDimension
import tf

import numpy as np
import cv2 as cv
import os


class Collector():
    
    def __init__(self,agentName,mesh_acc):
        subTopic = agentName + "/region"
        rospy.Subscriber(subTopic, Int8MultiArray, self.int8MultiArrayCallback, queue_size=1)
        self.region = np.zeros((mesh_acc[1],mesh_acc[0]),dtype=np.bool)

    def int8MultiArrayCallback(self,msg_data):
        region_vec = np.array(msg_data.data)
        region_ = np.reshape(region_vec,(msg_data.layout.dim[0].size, msg_data.layout.dim[1].size))
        self.region = region_.astype(np.bool)

    def allPositionGet(self):
        for i in range(self.agentNum):
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
                    self.allPositions[i] = position
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    continue
            # rospy.loginfo(self.allPositions)

    def getRegion(self):
        return self.region

class central():
    def __init__(self):
        # ROS Initialize
        rospy.init_node('central', anonymous=True)

        mesh_acc = [rospy.get_param("/mesh_acc/x",100),rospy.get_param("/mesh_acc/y",150)]

        self.agentNum = rospy.get_param("/agentNum",1)

        self.Collectors = []

        # subscriber
        for agentID in range(self.agentNum):
            agentName = "bebop10" + str(agentID+1)
            rospy.loginfo(agentName)
            collector = Collector(agentName,mesh_acc) 
            self.Collectors.append(collector)
            
        # publisher
        self.pub_info = rospy.Publisher('/info', Float32MultiArray, queue_size=1)


        #get_ROSparam
        self.clock = rospy.get_param("~clock",100)

        # param initialize
        self.rate = rospy.Rate(self.clock)
        
        self.phi = 0.5*np.ones((mesh_acc[1],mesh_acc[0]))

        # self.twist_from_controller = Twist()


        rospy.loginfo("starting node")
        # rospy.loginfo(self.agentID)


    def infoUpdate(self,Z,region):
        delta_decrease = 0.01
        delta_increase = 0.0001
        Z = Z-delta_decrease*region
        Z = np.where(Z<0.01,0.01,Z) 
        Z = Z + delta_increase*~region
        Z = np.where(Z>1,1,Z) 
        return Z

    def publishInfo(self,info):
        info_vec_ = np.reshape(info,(1,-1))
        dim_ = []
        dim_.append(MultiArrayDimension(label="y",size=info.shape[0],stride=info.shape[0]*info.shape[1]))
        dim_.append(MultiArrayDimension(label="x",size=info.shape[1],stride=info.shape[1]))
        layout_ = MultiArrayLayout(dim=dim_)
        info_int = info_vec_.astype(np.float32)
        info_vec = info_int.squeeze()
        info_for_pub = Float32MultiArray(data=info_vec.tolist(),layout=layout_)
        self.pub_info.publish(info_for_pub) 
        

    def spin(self):
        while not rospy.is_shutdown():
            region = self.Collectors[0].getRegion()
            for collector in self.Collectors:
                region = region + collector.getRegion()

            self.phi = self.infoUpdate(self.phi,region)
            self.publishInfo(self.phi)


            

            self.rate.sleep()

    

        

if __name__ == '__main__':
    try:
        central = central()
        central.spin()

    except rospy.ROSInterruptException: pass
