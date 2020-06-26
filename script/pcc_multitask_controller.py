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

import dynamic_reconfigure.client

from task_switch.voronoi_main import Voronoi
from task_switch.voronoi_main import Field
from task_switch.cbf_qp_optimizer import CBFOptimizer
import tf


import numpy as np
import cv2 as cv
import os

# inherit
class Field(Field):

    # override
    def __init__(self,mesh_acc,xlimit,ylimit):
        self.mesh_acc = mesh_acc
        self.xlimit = xlimit
        self.ylimit = ylimit
        # dimension is inverse to X,Y
        self.phi = np.ones((self.mesh_acc[1],self.mesh_acc[0]))


# inherit
class VoronoiCalc(Voronoi):

    def update_param(self, R, b_):
        self.R = R
        self.b = -(R**2)-b_

# inherit
class CBFOptimizerROS(CBFOptimizer):
        pass

class coverageController():

    def __init__(self):
        # ROS Initialize
        rospy.init_node('coverageController', anonymous=True)



        #get_ROSparam
        self.agentID = rospy.get_param("~agentID",1)
        self.agentNum = rospy.get_param("/agentNum",1)
        mesh_acc = [rospy.get_param("/mesh_acc/x",100),rospy.get_param("/mesh_acc/y",150)]
        xlimit = [rospy.get_param("/x_min",-1.0),rospy.get_param("/x_max",1.0)]
        ylimit = [rospy.get_param("/y_min",-1.0),rospy.get_param("/y_max",1.0)]

        # param initialize
        self.clock = rospy.get_param("~clock",100)
        self.rate = rospy.Rate(self.clock)

        # initialize message
        self.twist_from_controller = Twist()

        # initialize field class
        self.field = Field(mesh_acc,xlimit,ylimit)
        # initialize voronoiCalc class
        self.voronoi = VoronoiCalc(self.field)
        # initialize neighborpos
        self.allPositions = np.zeros((self.agentNum,3)) 


        # listner for tf to get other agent's position
        self.listner = tf.TransformListener()
        # subscriber to get own pose
        rospy.Subscriber("pose", PoseStamped, self.poseStampedCallback, queue_size=1)
        # subscriber to get field information density
        rospy.Subscriber("/info", Float32MultiArray, self.Float32MultiArrayCallback, queue_size=1)
        # publisher for agent control
        self.pub_twist = rospy.Publisher('cmd_input', Twist, queue_size=1)
        self.pub_takeoff = rospy.Publisher('takeoff', Empty, queue_size=1)
        self.pub_land = rospy.Publisher('land', Empty, queue_size=1)
        self.pub_reset = rospy.Publisher('reset', Empty, queue_size=1)
        # publisher for own region
        self.pub_region = rospy.Publisher('region', Int8MultiArray, queue_size=1)

        #dynamic_reconfigure
        self.dycon_client = dynamic_reconfigure.client.Client("/pcc_parameter", timeout=2, config_callback=self.config_callback)

        # controller gain. any number is ok because it will be overwritten by dycon
        self.k = 0.1
        

        self.optimizer = CBFOptimizerROS()

        
        rospy.loginfo("starting node")
        rospy.loginfo(self.agentID)

        # wait for tf 
        # rospy.sleep(1.0)

    def _update_config_params(self, config):
        self.k = config.controller_gain
        self.voronoi.update_param(config.agent_R,config.agent_b_)


    def set_config_params(self):
        config = self.dycon_client.get_configuration()
        self._update_config_params(config)
        rospy.loginfo("Dynamic Reconfigure Params SET")

    def config_callback(self,config):
        self._update_config_params(config)
        rospy.loginfo("Dynamic Reconfigure Params Update")
        
    def poseStampedCallback(self, pose_msg):
        self.position = np.array([pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z])
        self.orientation = np.array([pose_msg.pose.orientation.x, pose_msg.pose.orientation.y, \
                pose_msg.pose.orientation.z, pose_msg.pose.orientation.w])

    def Float32MultiArrayCallback(self,msg_data):
        # msg_data is information density, which is vectorized.
        info = np.array(msg_data.data).reshape((msg_data.layout.dim[0].size, msg_data.layout.dim[1].size))
        self.field.updatePhi(info)

    def publishRegion(self,region):
        # make multiarraydimension
        dim_ = []
        dim_.append(MultiArrayDimension(label="y",size=region.shape[0],stride=region.shape[0]*region.shape[1]))
        dim_.append(MultiArrayDimension(label="x",size=region.shape[1],stride=region.shape[1]))
        # make multiarraylayout
        layout_ = MultiArrayLayout(dim=dim_)

        # vectorize region, convert cast, delete size 1 dimension.
        region_vec = np.reshape(region,(1,-1)).astype(np.int8).squeeze()
        
        # make Int8multiarray. numpy to list convert
        region_for_pub = Int8MultiArray(data=region_vec.tolist(),layout=layout_)
        # publish
        self.pub_region.publish(region_for_pub) 


    def velCommandCalc(self):
        # calculate command for agent

        # extract x,y position from x,y,z position data
        pos = self.position[0:2]

        # extract x,y position from list of x,y,z position
        neighborPos2d = np.delete(self.allPositions,2,axis=1)
        # delete THIS agent position
        neighborPosOnly = np.delete(neighborPos2d,self.agentID-1,axis=0)

        # set my x,y position, and neighbor's position
        self.voronoi.setPos(pos)
        self.voronoi.setNeighborPos(neighborPosOnly)

        # set information density 
        self.voronoi.setPhi(self.field.getPhi())
        # rospy.loginfo(self.field.getPhi())

        # calculate voronoi region
        self.voronoi.calcVoronoi()

        # calculate command for agent
        u_nom2d = (-pos+self.voronoi.getCent()-self.voronoi.getExpand()/(2*self.voronoi.getMass()))*self.k

        
        u_nom = np.array( [ [u_nom2d[0]], [u_nom2d[1]], [0.], [0.], [0.], [0.] ]  )
        AgentPos = [pos[0], pos[1], 0., 0., 0., 0.]
        u = self.optimizer.optimize(u_nom, AgentPos)

        # set command to be published
        self.twist_from_controller.linear.x = u[0]
        self.twist_from_controller.linear.y = u[1]
        

    def allPositionGet(self):
        # get every agent's position

        for i in range(self.agentNum):
            if i+1 == self.agentID:
                # don't get own position
                pass
            else:
                try:
                    # agent i's tf prefix
                    agenttf = "/bebop10" + str(i+1) + "/rigidmotion"

                    # get agent i's tf from world
                    (position, orientation) = self.listner.lookupTransform(
                        "/world",
                         agenttf,
                          rospy.Time(0))
                    # Save agent position
                    self.allPositions[i] = position
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    continue
            # rospy.loginfo(self.allPositions)
    
    def check_start(self):
        for i in range(self.agentNum):
            while not rospy.is_shutdown():
                try:
                    # agent i's tf prefix
                    agenttf = "/bebop10" + str(i+1) + "/rigidmotion"

                    # get agent i's tf from world
                    (_position, _orientation) = self.listner.lookupTransform(
                        "/world",
                         agenttf,
                          rospy.Time(0))
                    break

                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    continue
        rospy.loginfo("All tf is ready")
    

    def spin(self):
        self.set_config_params()
        self.check_start()
        while not rospy.is_shutdown():
            # get neighbor position
            self.allPositionGet()
            # calculate voronoi region and input velocity
            self.velCommandCalc()
            
            # publish vel command
            self.pub_twist.publish(self.twist_from_controller)
            # publish my region
            self.publishRegion(self.voronoi.getRegion())
            self.rate.sleep()

    

        

if __name__ == '__main__':
    try:
        controller = coverageController()
        controller.spin()

    except rospy.ROSInterruptException: pass
