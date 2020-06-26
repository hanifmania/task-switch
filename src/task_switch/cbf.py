#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
import math
import cv2 as cv
from task_switch.se3_operations import *
import matplotlib.pyplot as plt

class CBF(object):
    """
    Returns:
        <G>:constraint matrix(dh/dx)(list, 1x6)
        <h>:constraint value(=alpha(h(x)))(list, 1x1)
    """
    def __init__(self):
        self.G = [0.] * 6
        self.h = [0.]
        self.alpha = 1

    def setAlpha(self,alpha):
        self.alpha = alpha
        
    
    def calcConstraintMatrix(self):
        # calc G
        pass
    
    def calcConstraintValue(self):
        # calc h
        pass

    def getConstraintSetting(self):
        return self.G, self.h

class pnorm2dCBF(CBF):
    # if ConstraintValue( =h(x) ) satisfies h(x)>0
    # then, agent is inside of ellipsoid
    """
    Args: 
        <CentPos>:center of ellipsoid (list, 1x2)
        <theta>:rotation angle (rad)
        <norm>:p-norm
        <width>:width and height of ellipsoid(list, 1x2)
        <AgentPos>:position of agent (list, 1x6)
        <inside>:True->prohibit going outside of ellipsoid, False->prohibit entering inside of ellipsoid
    Returns:
        <G>:constraint matrix(dh/dx)(list, 1x6)
        <h>:constraint value(=alpha(h(x)))(list, 1x1)
    """



    def setPnormSetting(self,centPos,theta,norm,width,inside=True):
        self.centPos = centPos
        self.theta = theta
        self.norm = norm
        self.width = width
        if inside:
            self.sign = 1
        else:
            self.sign = -1


    def getCommonValue(self,AgentPos):
        # calculate p-exponential's base

        # to improve readability
        x = AgentPos[0]
        y = AgentPos[1]
        xc = self.centPos[0]
        yc = self.centPos[1]
        theta = self.theta
        norm = self.norm
        xw = self.width[0]
        yw = self.width[1]

        base0 = ( (x-xc)*math.cos(theta)+(y-yc)*math.sin(theta))/xw
        base1 = (-(x-xc)*math.sin(theta)+(y-yc)*math.cos(theta))/yw
        base = [base0,base1]
        return base
        
   
    def calcConstraintMatrix(self,AgentPos):
        # to improve readability
        theta = self.theta
        norm = self.norm
        xw = self.width[0]
        yw = self.width[1]

        # calculate common coefficients in advance
        base = self.getCommonValue(AgentPos)
        coef1 = -norm*(base[0])**(norm-1)
        coef2 = -norm*(base[1])**(norm-1)


        self.G[0] = (coef1*math.cos(theta)/xw + coef2*math.sin(theta)/yw) * self.sign
        self.G[1] = (coef1*math.sin(theta)/xw + coef2*math.cos(theta)/yw) * self.sign
        
    def calcConstraintValue(self,AgentPos):
        # to improve readability
        norm = self.norm

        # calculate common coefficients in advance
        base = self.getCommonValue(AgentPos)
        self.h[0] = (-base[0]**norm - base[1]**norm + 1) * self.sign

    def getConstraintSetting(self,AgentPos):
        self.calcConstraintMatrix(AgentPos)
        self.calcConstraintValue(AgentPos)
        return self.G, self.h
    


if __name__ == '__main__':
    AgentPos = [0.0,1.0,0.0,0.0,0.0,0.0]
    centPos = [0.0,0.0]
    theta = 0
    norm = 2
    width = [1.0,2.0]
    pnormcbf = pnorm2dCBF()
    pnormcbf.setPnormSetting(centPos,theta,norm,width)
    dhdp, h = pnormcbf.getConstraintSetting(AgentPos)
    print dhdp, h

    
    # plt.show()
    # vbwc = np.c_[np.array([1, 2, 3, 4, 5, 6])]
    # g_rel = np.vstack([np.hstack([np.eye(3),np.c_[np.array([0, 0, 10])]]), np.hstack([np.zeros((1,3)),np.ones((1,1))])])

    # fovcbf = FoVCBF()
    # fovcbf.constraint_matrix(vbwc,g_rel)

