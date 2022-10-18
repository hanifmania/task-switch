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


class coverageControllerGrad(coverageController):

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
        u_nom2d = (-pos+self.voronoi.getCent()-self.voronoi.getExpand()/(2*self.voronoi.getMass()))*self.controllerGain
        # u_nom = np.array( [ [u_nom2d[0]], [u_nom2d[1]], [0.], [0.], [0.], [0.] ]  )
        # dJdp = [0., 0., 0., 0., 0., 0.]
        # xi = [0.]

        #### cbf persistent coverage #####################################
        # u_nom = np.array( [ [0.], [0.], [0.], [0.], [0.], [0.] ]  )
        # dJdp2d = 2*self.voronoi.getMass()*(self.voronoi.getCent()-pos)-self.voronoi.getExpand()
        # dJdp = [dJdp2d[0], dJdp2d[1], 0., 0., 0., 0.]
        # xi = [self.voronoi.getXi()]

        # u, opt_status, task = self.optimizer.optimize(u_nom, AgentPos, currentEnergy, dJdp, xi,neighborPosOnly,self.collisionR)

        # return u[0], u[1], opt_status, task
        # print("{}:{:.3f},{:.3f}".format(self.agentID, u_nom2d[0], u_nom2d[1]))
        return u_nom2d[0], u_nom2d[1], None, None

    def publishOptStatus(self,optStatus,task):
        pass

        

if __name__ == '__main__':
    try:
        controller = coverageControllerGrad()
        controller.spin()

    except rospy.ROSInterruptException: pass
