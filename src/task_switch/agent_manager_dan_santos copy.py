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
from vision_msgs.msg import Detection2DArray
from jsk_rviz_plugins.msg import *

import dynamic_reconfigure.client

import tf

from task_switch.transformations import *

import numpy as np
import cv2 as cv
import os

from task_switch.agent_manager_dan import coverageController
from task_switch.agent_manager_dan import VoronoiCalc

class VoronoiCalcSantos2019(VoronoiCalc):
    def __init__(self, field):
        super(VoronoiCalcSantos2019,self).__init__(field)

        Xrange = self.X.max() - self.X.min()

        Yrange = self.Y.max() - self.Y.min()
        self._grid_span = [Xrange/ (field.mesh_acc[0] - 1), Yrange/ (field.mesh_acc[1] - 1)]

        self._arc_width = np.sqrt (self._grid_span[0] **2 + self._grid_span[1] **2)
        
    def calc4CBF(self):
        #### calc dGdx

        ##### calc arc
        pos = self.Pos
        distance = np.sqrt((self.X-pos[0])**2 + (self.Y-pos[1])**2)
        normal_voronoi_region = np.ones_like(self.X, dtype=np.bool)  
        arcs = []
        for neighborPos in self.listNeighborPos:
            neighborDistance = np.sqrt((self.X-neighborPos[0])**2 + (self.Y-neighborPos[1])**2) 
            nearRegion = neighborDistance > distance
            normal_voronoi_region = normal_voronoi_region*nearRegion

            nearRegion2 = neighborDistance - self._arc_width > distance
            arcs.append(nearRegion * ~nearRegion2)
            

        weightedX = self.X*self.phi*normal_voronoi_region
        weightedY = self.Y*self.phi*normal_voronoi_region

        # calculate mass and cent
        mass = np.sum(self.phi*normal_voronoi_region)*self.pointDense
        cent = np.array([weightedX.sum(), weightedY.sum()])*self.pointDense/mass

        # print("onEdgePhi", onEdgePhi)
        ######################## r-limited voronoi version
        # onEdgePhi = self._onEdgePhi
        # mass = self.getMass()
        # cent = self.getCent()
        ######################## r-limited voronoi version
        dGdx = np.zeros((2,2))
        for neighbor, arc in zip(self.listNeighborPos, arcs):

            neighborDistance = np.sqrt((self.X-neighborPos[0])**2 + (self.Y-neighborPos[1])**2)
            onEdgePhi = normal_voronoi_region * arc *self.phi
            
            dist = np.sqrt((pos[0] - neighbor[0])**2 +(pos[1] - neighbor[1])**2)  
            temp = (self.X - cent[0]) * (self.X - pos[0]) * onEdgePhi/ dist
            dGdx[0,0] += np.sum(temp)

            temp = (self.X - cent[0]) * (self.Y - pos[1]) * onEdgePhi/ dist
            dGdx[0,1] += np.sum(temp)

            temp = (self.Y - cent[1]) * (self.X - pos[0]) * onEdgePhi/ dist
            dGdx[1,0] += np.sum(temp)

            temp = (self.Y - cent[1]) * (self.Y - pos[1]) * onEdgePhi/ dist
            dGdx[1,1] += np.sum(temp)
        # print("dGdx before", dGdx)
        dGdx =dGdx/mass*self.pointDense
        # print("dGdx")
        # print(dGdx)
        # print("mass",mass)

        self._dJdp = - np.dot(pos-cent, np.identity(2) - dGdx)
        # print("temp1")
        # print(temp1)
        # print("np.identity(2) - dGdx")
        # print(np.identity(2) - dGdx)
        # print("self._dJdp")
        # print(self._dJdp)

        ### calc dGdt
        # delta_increase_region = normal_voronoi_region*~self.Region
        # dphidt_increase = self.delta_increase * (1 - self.phi) *delta_increase_region
        # increase_x = (self.X - cent[0])* dphidt_increase
        # increase_y = (self.Y - cent[1])* dphidt_increase

        # print("delta_increase_region")
        # print(delta_increase_region)
        # print("dphidt_increase")
        # print(dphidt_increase)
        # print("increase_x")
        # print(increase_x)


        dphidt_decrease = -self.delta_decrease * self.phi*self.Region
        decrease_x = (self.X  - cent[0])* dphidt_decrease
        decrease_y = (self.Y  - cent[1])* dphidt_decrease

        # print("np.sum(increase_x)")
        # print(np.sum(increase_x))
        # print("np.sum(decrease_x)")
        # print(np.sum(decrease_x))
        # print("np.sum(increase_x + decrease_x)")
        # print(np.sum(increase_x + decrease_x))
        ######################## r-limited voronoi version
        increase_x = 0
        increase_y=0
        ######################## r-limited voronoi version
        dGdt = np.array([[np.sum(increase_x + decrease_x)],[np.sum(increase_y + decrease_y)]])/mass*self.pointDense
        
        ### calc xi
        J_i = np.linalg.norm(pos-cent)**2 / 2.0
        # print("pos,cent", pos, cent)
        # print("J_i")
        # print(J_i)
        
        self._xi = -self.k * J_i +  np.dot(pos - cent, dGdt)
        # print("self.k ")
        # print(self.k )
        # print("dGdt")
        # print(dGdt)
        # print("self._xi")
        # print(self._xi)
        # rospy.sleep(20)

class coverageControllerSantos2019(coverageController):
    def __init__(self):
        super(coverageControllerSantos2019, self).__init__()
        self.voronoi = VoronoiCalcSantos2019(self.field)

    ###################################################################
    ### velocity command calculation function 
    ###################################################################

    def Vel2dCommandCalc(self):
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
            
        self.voronoi.calc4CBF()
        dJdp2d = self.voronoi._dJdp

        # dJdp2d = 2*self.voronoi.getMass()*(self.voronoi.getCent()-pos)-self.voronoi.getExpand()
        dJdp = [dJdp2d[0], dJdp2d[1], 0., 0., 0., 0.]
        xi = [self.voronoi.getXi()]

        u, opt_status, task = self.optimizer.optimize(u_nom, AgentPos, currentEnergy, dJdp, xi,neighborPosOnly,self.collisionR)
        # print("{}:{:.3f},{:.3f}".format(self.agentID, u[0][0], u[1][0]))
        return u[0], u[1], opt_status, task
        # return u_nom2d[0], u_nom2d[1], None, None

    def publishOptStatus(self,optStatus,task):
        pass

        

if __name__ == '__main__':
    try:
        controller = coverageControllerSantos2019()
        controller.spin()

    except rospy.ROSInterruptException: pass
