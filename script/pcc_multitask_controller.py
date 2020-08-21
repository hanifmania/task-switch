#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Empty, String, ColorRGBA
from std_msgs.msg import Int8MultiArray,MultiArrayLayout,MultiArrayDimension
from std_msgs.msg import Bool,Float32,Float32MultiArray
from jsk_rviz_plugins.msg import *

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

    # override
    def calcVoronoi(self):
        super(VoronoiCalc,self).calcVoronoi()

        ################### add the calculation of terms for PCC CBF ############################

        voronoiX = self.X*self.Region  
        voronoiY = self.Y*self.Region  
        # calc J = - \sum \int_{S_i} \|q-p_i\|^2\phi(q,t)dq + b\int_{Q...} \phi(q,t)dq
        #        = - \sum \int_{S_i} (\|q-p_i\|^2 + b) \phi(q,t)dq + b\int_{Q} \phi(q,t)dq
        #                 ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ <- here calc this term
        self.JintSPlusb = np.sum( ((self.X - self.Pos[0])**2 + (self.Y - self.Pos[1])**2 + self.b ) * self.Region * self.phi * self.pointDense )

        # calc \xi
        agentNum = self.listNeighborPos.shape[0] + 1
        term1 = (self.delta_decrease - self.k) * np.sum( ((self.X - self.Pos[0])**2 + (self.Y - self.Pos[1])**2 ) * self.Region * self.phi * self.pointDense )
        term2 = np.sum( self.delta_increase + ((self.k - self.delta_increase) * self.phi) ) * self.pointDense  / agentNum 
        term3 = np.sum((self.delta_increase + ((self.k - self.delta_increase) * self.phi)) * self.Region) * self.pointDense  
        term4 = self.k*self.gamma/agentNum
        self.xi = term1 + self.b * (term2 - term3) - term4

    def update_agentParam(self, R, b_):
        self.R = R
        self.b = -(R**2)-b_

    def update_fieldParam(self, delta_decrease, delta_increase):
        self.delta_decrease = delta_decrease
        self.delta_increase = delta_increase

    def update_PccCBFParam(self,gamma,k):
        self.gamma = gamma
        self.k = k

    def getJintSPlusb(self):
        return self.JintSPlusb

    def getXi(self):
        return self.xi

# inherit
class CBFOptimizerROS(CBFOptimizer):

    # override
    def __init__(self):
        super(CBFOptimizerROS,self).__init__()
        
        # cbf activations, these will be overwritten by dycon 
        self.activate_cbf = True
        self.activate_fieldcbf = False
        self.activate_chargecbf = False
        self.activate_pcccbf = True
        self.activate_staycbf = False
        self.activate_collisioncbf = True
        # if any cbf are not activated, set active_cbf False
        if not any([self.activate_fieldcbf,self.activate_chargecbf,self.activate_pcccbf,self.activate_collisioncbf]):
            self.activate_cbf = False 

        
        # cbf slack weight, these will be overwritten by dycon 
        self.fieldcbf_slack_weight = 1.0
        self.chargecbf_slack_weight = 1.0
        self.pcccbf_slack_weight = 1.0
        self.staycbf_slack_weight = 1.0

        # input range constraint, these will be overwritten by dycon
        self.activate_umax = True
        self.umax = 2.0
        self.umin = -self.umax 




    def updateCbfConfig(self,config):
        # update cbf activations status
        self.activate_cbf = config.activate_cbf
        self.activate_fieldcbf = config.activate_fieldcbf
        self.activate_chargecbf = config.activate_chargecbf
        self.activate_pcccbf = config.activate_pcccbf
        self.activate_staycbf = config.activate_staycbf
        self.activate_collisioncbf = config.activate_collisioncbf
        # if any cbf are not activated, set active_cbf False
        if not any([self.activate_fieldcbf,self.activate_chargecbf,self.activate_pcccbf,self.activate_collisioncbf,self.activate_staycbf]):
            self.activate_cbf = False 

        # update cbf slack weight
        self.fieldcbf_slack_weight = config.fieldcbf_slack_weight
        self.chargecbf_slack_weight = config.chargecbf_slack_weight
        self.pcccbf_slack_weight = config.pcccbf_slack_weight
        self.staycbf_slack_weight = config.staycbf_slack_weight

        # update input range constraint
        self.updateInputRange(config.activate_umax,config.umax,-config.umax)


# class CBFVisualizer(object):
#     def __init__(self,agentID):
#         self.basename = "agent" + str(agentID)
#         # self.pubtopic = str(self.basename)+"/marker"
#         self.collisioncbf_pubtopic = "/visualization/agentSphere"


#         self.red = rospy.get_param("~red", default=1)
#         self.green = rospy.get_param("~green", default=0)
#         self.blue = rospy.get_param("~blue", default=0)
#         self.alpha = rospy.get_param("~alpha", default=.5)
        
#         self.marker = Marker()
#         self.marker.ns = str(self.basename)
#         self.marker.id = 0
#         self.marker.color.r = self.red
#         self.marker.color.g = self.green
#         self.marker.color.b = self.blue
#         self.marker.color.a = self.alpha


#         self.markerpub = rospy.Publisher(self.pubtopic, Marker, queue_size=10)



#     def setCollisionMarker(self, pose_msg, collisionR):
        
#         self.marker.header.frame_id = "/world"
#         self.marker.header.stamp = rospy.Time.now()

#         self.marker.action = Marker.ADD

#         self.marker.pose = pose_msg.pose


#         self.marker.scale.x = collisionR*2
#         self.marker.scale.y = collisionR*2
#         self.marker.scale.z = collisionR*2

#         self.marker.lifetime = rospy.Duration()
#         self.marker.type = Marker.SPHERE

#     def publishCollisionCBF(self):
#         self.markerpub.publish(self.marker)



class coverageController():

    def __init__(self):
        # ROS Initialize
        rospy.init_node('coverageController', anonymous=True)

        # wait for pose array to get neighbor position
        self.checkNeighborStart = False


        #get_ROSparam
        # self.agentID = rospy.get_param("agentID",1)
        # self.agentNum = rospy.get_param("agentNum",1)
        # mesh_acc = [rospy.get_param("mesh_acc/x",100),rospy.get_param("mesh_acc/y",150)]
        # xlimit = [rospy.get_param("x_min",-1.0),rospy.get_param("x_max",1.0)]
        # ylimit = [rospy.get_param("y_min",-1.0),rospy.get_param("y_max",1.0)]
        self.agentNum = rospy.get_param("/agentNum")
        mesh_acc = [rospy.get_param("/mesh_acc/x"),rospy.get_param("/mesh_acc/y")]
        xlimit = [rospy.get_param("/x_min"),rospy.get_param("/x_max")]
        ylimit = [rospy.get_param("/y_min"),rospy.get_param("/y_max")]
        self.agentID = rospy.get_param("agentID")

        # param initialize
        self.clock = rospy.get_param("clock",100)
        self.rate = rospy.Rate(self.clock)


        # initialize field class
        self.field = Field(mesh_acc,xlimit,ylimit)
        # initialize voronoiCalc class
        self.voronoi = VoronoiCalc(self.field)
        # initialize neighborpos
        # allPositions includes myself position
        self.allPositions = np.zeros((self.agentNum,3)) 

        # initialize CBF Optimizer
        self.optimizer = CBFOptimizerROS()

        # subscriber to get own pose
        rospy.Subscriber("posestamped", PoseStamped, self.poseStampedCallback, queue_size=1)
        # subscriber to get own energy
        rospy.Subscriber("energy", Float32, self.energyCallback, queue_size=1)
        # subscriber to get field information density
        rospy.Subscriber("/info", Float32MultiArray, self.Float32MultiArrayCallback, queue_size=1)
        # subscriber to other agent's position
        rospy.Subscriber("/allPose", PoseArray, self.poseArrayCallback, queue_size=1)
        # publisher for agent control
        self.pub_twist = rospy.Publisher('cmd_input', Twist, queue_size=1)
        self.pub_takeoff = rospy.Publisher('takeoff', Empty, queue_size=1)
        self.pub_land = rospy.Publisher('land', Empty, queue_size=1)
        self.pub_reset = rospy.Publisher('reset', Empty, queue_size=1)
        # publisher for own region
        self.pub_region = rospy.Publisher('region', Int8MultiArray, queue_size=1)
        # publisher for own coverage performance value
        self.pub_JintSPlusb = rospy.Publisher('JintSPlusb', Float32, queue_size=1)
        # publisher for energy drain rate
        self.pub_drainRate = rospy.Publisher('drainRate', Float32, queue_size=1)
        # publisher to display current CBF optimization status
        self.pub_optStatus = rospy.Publisher('optStatus', OverlayText, queue_size=1)


        #dynamic_reconfigure
        self.pcc_dycon_client = dynamic_reconfigure.client.Client("/pcc_parameter", timeout=2, config_callback=self.pcc_config_callback)
        self.charge_dycon_client = dynamic_reconfigure.client.Client("/charge_parameter", timeout=2, config_callback=self.charge_config_callback)
        self.cbf_dycon_client = dynamic_reconfigure.client.Client("/cbf_parameter", timeout=2, config_callback=self.cbf_config_callback)

        # controller gain. any number is ok because it will be overwritten by dycon
        self.controllerGain = 0.1
        # pcc CBF parameter settings.any number is ok because it will be overwritten by dycon
        k = 1.
        gamma = -7.

        # charging configs. any number is ok because it will be overwitten by dycon
        self.maxEnergy = 4000
        minEnergy = 1500        
        Kd = 50 # per seconds
        k_charge = 0.15

        # init enegy
        self.energy = 0
        # current charging or not charged status
        self.charging = False
        
        # collision avoidance radius
        # each agents keeps collisionR * 2 distance from other agents
        self.collisionR = 0.5

        # field setting
        # prohibit from going out of field
        centPos = [sum(xlimit)/len(xlimit),sum(ylimit)/len(ylimit)]
        theta = 0
        norm = 6
        width = [(xlimit[1]-xlimit[0])/2,(ylimit[1]-ylimit[0])/2]
        keepInside = True
        self.optimizer.setFieldArea(centPos,theta,norm,width,keepInside)

        # charging station position and radius
        chargePos = [rospy.get_param("charge_station/x",0.),rospy.get_param("charge_station/y",0.)]
        radiusCharge = rospy.get_param("charge_station/r",0.2) 

        # charging station configs.
        self.optimizer.setChargeStation(chargePos,radiusCharge)

        # charging configs. any number is ok because it will be overwitten by dycon
        self.optimizer.setChargeSettings(minEnergy,Kd,k_charge)


        rospy.loginfo("starting node:agent"+str(self.agentID))

    ###################################################################
    ### dycon update functions 
    ###################################################################

    ############## PCC dycon ##########################################
    def pcc_update_config_params(self, config):
        self.controllerGain = config.controller_gain
        self.collisionR = config.collisionR
        agent_R = config.agent_R
        agent_b_ = config.agent_b_
        delta_decrease = config.delta_decrease
        delta_increase = config.delta_increase
        self.voronoi.update_agentParam(agent_R,agent_b_)
        self.voronoi.update_fieldParam(delta_decrease,delta_increase)

    def pcc_set_config_params(self):
        config = self.pcc_dycon_client.get_configuration()
        self.pcc_update_config_params(config)
        rospy.loginfo("Dynamic Reconfigure Pcc Params SET in agent"+str(self.agentID))

    def pcc_config_callback(self,config):
        self.pcc_update_config_params(config)
        rospy.loginfo("Dynamic Reconfigure Pcc Params Update in agent"+str(self.agentID))

    ############## charge dycon ##########################################
    def charge_update_config_params(self, config):
        self.maxEnergy = config.maxEnergy
        self.optimizer.setChargeSettings(config.minEnergy,config.Kd,config.k_charge)

    def charge_set_config_params(self):
        config = self.charge_dycon_client.get_configuration()
        self.charge_update_config_params(config)
        rospy.loginfo("Dynamic Reconfigure Charge Params SET in agent"+str(self.agentID))

    def charge_config_callback(self,config):
        self.charge_update_config_params(config)
        rospy.loginfo("Dynamic Reconfigure Charge Params Update in agent"+str(self.agentID))

    ############## cbf dycon ##########################################
    def cbf_update_config_params(self, config):
        self.optimizer.updateCbfConfig(config)
        self.voronoi.update_PccCBFParam(config.gamma,config.pcc_CBF_h_gain_k)

    def cbf_set_config_params(self):
        config = self.cbf_dycon_client.get_configuration()
        self.cbf_update_config_params(config)
        rospy.loginfo("Dynamic Reconfigure cbf Params SET in agent"+str(self.agentID))

    def cbf_config_callback(self,config):
        self.cbf_update_config_params(config)
        rospy.loginfo("Dynamic Reconfigure cbf Params Update in agent"+str(self.agentID))
        


    ###################################################################
    ### subscriber callback functions 
    ###################################################################

    def poseStampedCallback(self, pose_msg):
        # subscriber to get own pose(position)
        self.position = np.array([pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z])
        self.orientation = np.array([pose_msg.pose.orientation.x, pose_msg.pose.orientation.y, \
                pose_msg.pose.orientation.z, pose_msg.pose.orientation.w])

    def energyCallback(self, msg):
        # subscriber to get own energy
        self.energy = msg.data

    def Float32MultiArrayCallback(self,msg_data):
        # subscriber to get information density
        # msg_data is information density, which is vectorized.
        info = np.array(msg_data.data).reshape((msg_data.layout.dim[0].size, msg_data.layout.dim[1].size))
        self.field.updatePhi(info)

    def poseArrayCallback(self,msg):
        # subscriber to get every agent's position
        arraynum = len(msg.poses)
        if (self.checkNeighborStart == False) and (arraynum == self.agentNum):
            # if the pose array contains every agent's pose, set the checkNeighborstart flag as True
            self.checkNeighborStart = True
            rospy.loginfo("NeighborStart")

        elif self.checkNeighborStart:
            for i in range(arraynum):
                pos = [msg.poses[i].position.x, msg.poses[i].position.y, msg.poses[i].position.z]
                self.allPositions[i] = pos

    ###################################################################
    ### publisher functions 
    ###################################################################

    def publishRegion(self,region):
        # publish my sensing region
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

    def publishJintSPlusb(self,JintSPlusb):
        # publish my coverage performance
        self.pub_JintSPlusb.publish(Float32(data=JintSPlusb))

    def publishOptStatus(self,optStatus):
        # publish my CBF optimization status
        showText = OverlayText()
        showText.text = "agent" + str(self.agentID) + "'s optimization: " + optStatus 
        if optStatus == "optimal":
            showText.fg_color = ColorRGBA(25/255.0, 1.0, 240.0/255.0, 1.0)
        elif optStatus == "error":
            showText.fg_color = ColorRGBA(240.0/255.0, 0.0, 240.0/255.0, 1.0)
        else:
            showText.fg_color = ColorRGBA(240.0/255.0, 1.0, 25/255.0, 1.0)
        showText.bg_color = ColorRGBA(0.0,0.0,0.0,0.2)
        self.pub_optStatus.publish(showText)

    def publishDrainRate(self,drainRate):
        # publish my current energy drain rate
        self.pub_drainRate.publish(Float32(data=drainRate))

    def publishTwist(self,twist):
        # publish my current energy drain rate
        self.pub_twist.publish(twist)

    ###################################################################
    ### voronoi region calculation function 
    ###################################################################

    def calcVoronoiRegion(self):
        # extract x,y position from x,y,z position data
        pos = self.position[0:2]
        # print pos

        # extract x,y position from list of x,y,z position
        allPos2d = np.delete(self.allPositions,2,axis=1)
        # delete THIS agent position
        neighborPosOnly = np.delete(allPos2d,self.agentID-1,axis=0)
        # print str(self.agentID)+"'s pos: "+str(pos)+ " neighborpos: "+str(neighborPosOnly)

        # set my x,y position, and neighbor's position
        self.voronoi.setPos(pos)
        self.voronoi.setNeighborPos(neighborPosOnly)

        # set information density 
        self.voronoi.setPhi(self.field.getPhi())
        self.voronoi.calcVoronoi()

    ###################################################################
    ### velocity command calculation function 
    ###################################################################

    def velCommandCalc(self):
        # calculate command for agent

        pos = self.position[0:2]
        AgentPos = [pos[0], pos[1], 0., 0., 0., 0.]
        currentEnergy = self.energy

        # extract x,y position from list of x,y,z position
        allPos2d = np.delete(self.allPositions,2,axis=1)
        # delete THIS agent position
        neighborPosOnly = np.delete(allPos2d,self.agentID-1,axis=0)

        
        #### normal persistent coverage ##################################
        # calculate command for agent
        # different from sugimoto san paper at divided 2mass.(probably dividing 2mass is true)
        # u_nom2d = (-pos+self.voronoi.getCent()-self.voronoi.getExpand()/(2*self.voronoi.getMass()))*self.controllerGain
        # u_nom = np.array( [ [u_nom2d[0]], [u_nom2d[1]], [0.], [0.], [0.], [0.] ]  )
        # dJdp = [0., 0., 0., 0., 0., 0.]
        # xi = [0.]

        #### cbf persistent coverage #####################################
        u_nom = np.array( [ [0.], [0.], [0.], [0.], [0.], [0.] ]  )
        dJdp2d = 2*self.voronoi.getMass()*(self.voronoi.getCent()-pos)-self.voronoi.getExpand()
        dJdp = [dJdp2d[0], dJdp2d[1], 0., 0., 0., 0.]
        xi = [self.voronoi.getXi()]

        u, opt_status = self.optimizer.optimize(u_nom, AgentPos, currentEnergy, dJdp, xi,neighborPosOnly,self.collisionR)

        return u[0], u[1], opt_status


    ###################################################################
    ### judge charging state and return energy drain rate
    ###################################################################
        
    def judgeCharge(self):
        pos = self.position[0:2]
        currentEnergy = self.energy
        minEnergy, Kd, k_charge, chargePos, radiusCharge = self.optimizer.getChargeStation()
        isInStation = ( (pos[0]-chargePos[0])**2 + (pos[1]-chargePos[1])**2 < radiusCharge**2 ) 

        lastChargeState = self.charging 

        if isInStation or self.charging:
            if (currentEnergy<= minEnergy+500):
                self.charging = True
            
            if (currentEnergy>=self.maxEnergy):
                self.charging = False

        if self.charging:
            drainRate = -5*Kd # minus drainrate means battery is being charged
            if lastChargeState == False:
                rospy.loginfo("start charge")
        else:
            drainRate = Kd*self.optimizer.activate_chargecbf
            if lastChargeState == True:
                rospy.loginfo("end charge")

        return drainRate

            
    ###################################################################
    ### for the object detection, set detected object position
    ###################################################################
    def setObjectPosition(self):
        objectPos = [0.0]

        centPos = objectPos
        theta = 0
        norm = 2.
        width = [.5,.5]
        keepInside = True
        self.optimizer.setStayArea(centPos,theta,norm,width,keepInside)
        
    
    
    ###################################################################
    ### loop
    ###################################################################

    def spin(self):
        rospy.wait_for_message("/info", Float32MultiArray)
        rospy.wait_for_message("posestamped", PoseStamped)

        self.pcc_set_config_params()
        self.charge_set_config_params()
        self.cbf_set_config_params()
        
        # initialize message
        twist = Twist()

        while not rospy.is_shutdown():
            if self.checkNeighborStart:# wait for all neighbor start

                # calculate voronoi region
                self.calcVoronoiRegion()
                # calculate voronoi region and input velocity
                if self.charging and self.optimizer.activate_chargecbf:
                    twist.linear.x = 0.
                    twist.linear.y = 0.
                else:
                    ux, uy, opt_status = self.velCommandCalc()
                    twist.linear.x = ux
                    twist.linear.y = uy



                # publish drain rate
                drainRate = self.judgeCharge()
                self.publishDrainRate(drainRate)

                # publish optimization status
                self.publishOptStatus(opt_status)

                # publish my region
                self.publishRegion(self.voronoi.getRegion())
                # publish vel command
                self.publishTwist(twist)

                # publish my coverage performance
                JintSPlusb = self.voronoi.getJintSPlusb()
                self.publishJintSPlusb(JintSPlusb)

            self.rate.sleep()

    

        

if __name__ == '__main__':
    try:
        controller = coverageController()
        controller.spin()

    except rospy.ROSInterruptException: pass
