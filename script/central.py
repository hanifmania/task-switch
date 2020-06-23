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
        # subscriber for each agent's region
        rospy.Subscriber(subTopic, Int8MultiArray, self.int8MultiArrayCallback, queue_size=1)
        # initialze with zeros
        self.region = np.zeros((mesh_acc[1],mesh_acc[0]),dtype=np.bool)

    def int8MultiArrayCallback(self,msg_data):
        # msg_data is region data, which is vectorized.
        region_ = np.array(msg_data.data).reshape((msg_data.layout.dim[0].size, msg_data.layout.dim[1].size))
        # reset to boolean value
        self.region = region_.astype(np.bool)

    def getRegion(self):
        return self.region

class central():
    def __init__(self):
        # ROS Initialize
        rospy.init_node('central', anonymous=True)

        # field mesh accuracy
        mesh_acc = [rospy.get_param("/mesh_acc/x",100),rospy.get_param("/mesh_acc/y",150)]
        # Number of Agents
        self.agentNum = rospy.get_param("/agentNum",1)

        
        self.Collectors = []
        # create [Agent's number] subscriber 
        for agentID in range(self.agentNum):
            agentName = "bebop10" + str(agentID+1)
            rospy.loginfo(agentName)
            collector = Collector(agentName,mesh_acc) 
            self.Collectors.append(collector)
            
        # publisher for information density
        self.pub_info = rospy.Publisher('/info', Float32MultiArray, queue_size=1)
        # information density initialize
        self.phi = 0.5*np.ones((mesh_acc[1],mesh_acc[0]))


        # node freq
        self.clock = rospy.get_param("~clock",100)
        self.rate = rospy.Rate(self.clock)
        


        rospy.loginfo("starting central node")


    def infoUpdate(self,Z,region):
        # information reliability update

        # delta_decrease = 1/self.clock
        # delta_increase = 0.1/self.clock
        delta_decrease = 0.01
        delta_increase = 0.0001

        Z = Z-delta_decrease*region
        Z = np.where(Z<0.01,0.01,Z) 
        Z = Z + delta_increase*~region
        Z = np.where(Z>1,1,Z) 
        return Z

    def publishInfo(self,info):
        # publish information density

        # make multiarraydimension
        dim_ = []
        dim_.append(MultiArrayDimension(label="y",size=info.shape[0],stride=info.shape[0]*info.shape[1]))
        dim_.append(MultiArrayDimension(label="x",size=info.shape[1],stride=info.shape[1]))
        # make multiarraylayout
        layout_ = MultiArrayLayout(dim=dim_)

        # vectorize info(=phi), convert cast, delete size 1 dimension.
        info_vec = np.reshape(info,(1,-1)).astype(np.float32).squeeze()
        
        # make Float32multiarray. numpy to list convert
        info_for_pub = Float32MultiArray(data=info_vec.tolist(),layout=layout_)

        # publish
        self.pub_info.publish(info_for_pub) 
        

    def spin(self):
        while not rospy.is_shutdown():

            # initialize region
            region = self.Collectors[0].getRegion()

            # collect each agent's region by sum
            for collector in self.Collectors:
                region = region + collector.getRegion()

            # update information density phi according to region  
            self.phi = self.infoUpdate(self.phi,region)

            # publish updated information density
            self.publishInfo(self.phi)

            self.rate.sleep()

    

        

if __name__ == '__main__':
    try:
        central = central()
        central.spin()

    except rospy.ROSInterruptException: pass
