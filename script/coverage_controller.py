#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Empty
from std_msgs.msg import Int8MultiArray,MultiArrayLayout,MultiArrayDimension
from std_msgs.msg import Float32MultiArray

from task_switch.voronoi_main import Voronoi
from task_switch.voronoi_main import Field
import tf

import numpy as np
import cv2 as cv
import os

class Field(Field):
    def __init__(self,mesh_acc,xlimit,ylimit):
        self.mesh_acc = mesh_acc
        self.xlimit = xlimit
        self.ylimit = ylimit
        # dimension is inverse to X,Y
        self.phi = np.ones((self.mesh_acc[1],self.mesh_acc[0]))


class VoronoiCalc(Voronoi):
    def __init__(self,field):

        self.phi = field.getPhi()
        self.Grid = field.getGrid()
        self.X = self.Grid[0]
        self.Y = self.Grid[1]

        self.R = 0.6
        self.b = -(self.R**2)-1
        

        self.Pos = [0,0]
        self.listNeighborPos = []

class coverageController():

    def __init__(self):
        # ROS Initialize
        rospy.init_node('coverageController', anonymous=True)

        self.listner = tf.TransformListener()
        # subscriber
        rospy.Subscriber("pose", PoseStamped, self.poseStampedCallback, queue_size=1)
        rospy.Subscriber("/info", Float32MultiArray, self.Float32MultiArrayCallback, queue_size=1)
        # publisher
        self.pub_twist = rospy.Publisher('cmd_input', Twist, queue_size=1)
        self.pub_takeoff = rospy.Publisher('takeoff', Empty, queue_size=1)
        self.pub_land = rospy.Publisher('land', Empty, queue_size=1)
        self.pub_reset = rospy.Publisher('reset', Empty, queue_size=1)
        self.pub_region = rospy.Publisher('region', Int8MultiArray, queue_size=1)


        #get_ROSparam
        self.clock = rospy.get_param("~clock",100)
        self.agentID = rospy.get_param("~agentID",1)
        mesh_acc = [rospy.get_param("/mesh_acc/x",100),rospy.get_param("/mesh_acc/y",150)]
        xlimit = [rospy.get_param("/x_min",-1.0),rospy.get_param("/x_max",1.0)]
        ylimit = [rospy.get_param("/y_min",-1.0),rospy.get_param("/y_max",1.0)]

        # param initialize
        self.rate = rospy.Rate(self.clock)

        self.twist_from_controller = Twist()
        self.field = Field(mesh_acc,xlimit,ylimit)
        self.voronoi = VoronoiCalc(self.field)

        
        self.agentNum = rospy.get_param("/agentNum",1)

        self.allPositions = np.zeros((self.agentNum,3)) 


        
        rospy.loginfo("starting node")
        rospy.loginfo(self.agentID)
        rospy.sleep(1.0)
        
    def poseStampedCallback(self, pose_msg):
        self.position = np.array([pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z])
        self.orientation = np.array([pose_msg.pose.orientation.x, pose_msg.pose.orientation.y, \
                pose_msg.pose.orientation.z, pose_msg.pose.orientation.w])

    def Float32MultiArrayCallback(self,msg_data):
        info_vec = np.array(msg_data.data)
        info = np.reshape(info_vec,(msg_data.layout.dim[0].size, msg_data.layout.dim[1].size))
        self.field.updatePhi(info)

    def publishRegion(self,region):
        region_vec_ = np.reshape(region,(1,-1))
        dim_ = []
        dim_.append(MultiArrayDimension(label="y",size=region.shape[0],stride=region.shape[0]*region.shape[1]))
        dim_.append(MultiArrayDimension(label="x",size=region.shape[1],stride=region.shape[1]))
        layout_ = MultiArrayLayout(dim=dim_)
        region_int = region_vec_.astype(np.int8)
        region_vec = region_int.squeeze()
        region_for_pub = Int8MultiArray(data=region_vec.tolist(),layout=layout_)
        self.pub_region.publish(region_for_pub) 


    def commandCalc(self):
        pos = self.position[0:2]
        # rospy.loginfo(pos)
        self.voronoi.setPos(pos)
        

        neighborPos2d = np.delete(self.allPositions,2,axis=1)
        neighborPosOnly = np.delete(neighborPos2d,self.agentID-1,axis=0)
        self.voronoi.setNeighborPos(neighborPosOnly)

        # rospy.loginfo(self.field.getPhi())
        self.voronoi.setPhi(self.field.getPhi())
        self.voronoi.calcVoronoi()

        u = (-pos+self.voronoi.getCent()-self.voronoi.getExpand()/(2*self.voronoi.getMass()))*0.1

        self.publishRegion(self.voronoi.getRegion())

        self.twist_from_controller.linear.x = u[0]
        self.twist_from_controller.linear.y = u[1]
        


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


    

    def spin(self):
        while not rospy.is_shutdown():
            self.allPositionGet()
            self.commandCalc()
            self.pub_twist.publish(self.twist_from_controller)
            self.rate.sleep()

    

        

if __name__ == '__main__':
    try:
        controller = coverageController()
        controller.spin()

    except rospy.ROSInterruptException: pass
