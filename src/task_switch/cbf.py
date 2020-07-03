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
        <q>:element of slack_variable_weight_matrix(Q) (scalar)
        <r>:element of soft constraint flag matrix(R) (scalar)
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

    def calcConstraint(self):
        # calc G and h by call calcConstraintMatrix, and calcConstraintValue.
        pass

    def getConstraint(self):
        return self.G, self.h

class generalCBF(CBF):
    # keep ConstraintValue( =h(x) ) satisfying h(x)>0
    """
    Args: 
        <G>:constraint matrix(dh/dx)(list, 1x6)
        <h>:constraint value(=alpha(h(x)))(list, 1x1)
    Returns:
        <G>:constraint matrix(dh/dx)(list, 1x6)
        <h>:constraint value(=alpha(h(x)))(list, 1x1)
    """

    def calcConstraintMatrix(self,G):
        self.G = G

    def calcConstraintValue(self,h):
        self.h = h

    def calcConstraint(self,G,h):
        self.calcConstraintMatrix(G)
        self.calcConstraintValue(h)

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
        <keepInside>:True->prohibit going outside of ellipsoid, False->prohibit entering inside of ellipsoid
    Returns:
        <G>:constraint matrix(dh/dx)(list, 1x6)
        <h>:constraint value(=alpha(h(x)))(list, 1x1)
    """



    def setPnormSetting(self,centPos,theta,norm,width,keepInside=True):
        self.centPos = centPos
        self.theta = theta
        self.norm = norm
        self.width = width
        if keepInside:
            self.sign = 1.
        else:
            self.sign = -1.


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


        self.G[0] = self.sign * (coef1*math.cos(theta)/xw + coef2*math.sin(theta)/yw) ** self.alpha
        self.G[1] = self.sign * (coef1*math.sin(theta)/xw + coef2*math.cos(theta)/yw) ** self.alpha
        
    def calcConstraintValue(self,AgentPos):
        # to improve readability
        norm = self.norm

        # calculate common coefficients in advance
        base = self.getCommonValue(AgentPos)
        self.h[0] = self.sign * (-base[0]**norm - base[1]**norm + 1) ** self.alpha

    def calcConstraint(self,AgentPos):
        self.calcConstraintMatrix(AgentPos)
        self.calcConstraintValue(AgentPos)

    
class chargeCBF(CBF):
    """
    Args: 
        <AgentPos>:position of agent (list, 1x6)
        <Kd>:battery drain rate (scalar)
        <k_charge>:gain (scalar)
        <chargePos>:position of charging station (list, 1x2)
        <radiusCharge>:radius of charging station (scalar)
    Returns:
        <G>:constraint matrix(dh/dx)(list, 1x6)
        <h>:constraint value(=alpha(h(x)))(list, 1x1)
    """
    def setChargeSettings(self, minEnergy, Kd, k_charge, chargePos, radiusCharge):
        self.minEnergy = minEnergy
        self.Kd = Kd
        self.k_charge = k_charge
        self.chargePos = np.array(chargePos)
        self.radiusCharge = radiusCharge

    def calcConstraintMatrix(self,AgentPos,energy):
        AgentPos2d = np.array((AgentPos[0],AgentPos[1]))
        norm = np.linalg.norm(AgentPos2d - self.chargePos)
        G_np = - (self.Kd/self.k_charge) * (AgentPos2d- self.chargePos)/norm
        self.G[0] = G_np.tolist()[0]
        self.G[1] = G_np.tolist()[1]

        
    
    def calcConstraintValue(self,AgentPos,energy):
        AgentPos2d = np.array((AgentPos[0],AgentPos[1]))
        norm = np.linalg.norm(AgentPos2d - self.chargePos)
        self.h[0] = (energy - self.minEnergy - (self.Kd/self.k_charge) * (norm - self.radiusCharge) - self.Kd ) ** self.alpha

    def calcConstraint(self,AgentPos,energy):
        self.calcConstraintMatrix(AgentPos,energy)
        self.calcConstraintValue(AgentPos,energy)


# class Collision2dCBF(pnorm2dCBF):

#     def __init__(self):
#         self.neighborPosOnly = np.array([[1.,1.],[-1.,-1.],[1.,-1.]])
#         self.agent_R = .3
#         self.G = [[0.] * 6] * self.neighborPosOnly.shape[0] # the number of neighbor * 6 matrix
#         self.h = [[0.]] * self.neighborPosOnly.shape[0]
#         self.alpha = 1


if __name__ == '__main__':
    AgentPos = [0.0,1.0,0.0,0.0,0.0,0.0]
    centPos = [0.0,0.0]
    theta = 0
    norm = 2
    width = [1.0,2.0]
    # pnormcbf = pnorm2dCBF()
    # pnormcbf.setPnormSetting(centPos,theta,norm,width)
    # dhdp, h = pnormcbf.getConstraintSetting(AgentPos)
    # print dhdp, h

    minEnergy = 1500
    Kd = 50
    k_charge = 0.15
    chargePos = [2.0,2.0]
    radiusCharge = 0.2
    energy = 3000
    chargecbf = ChargeCBF(minEnergy, Kd, k_charge, chargePos, radiusCharge)
    dhdp, h = chargecbf.getConstraintSetting(AgentPos,energy)
    print dhdp, h

    
    # plt.show()
    # vbwc = np.c_[np.array([1, 2, 3, 4, 5, 6])]
    # g_rel = np.vstack([np.hstack([np.eye(3),np.c_[np.array([0, 0, 10])]]), np.hstack([np.zeros((1,3)),np.ones((1,1))])])

    # fovcbf = FoVCBF()
    # fovcbf.constraint_matrix(vbwc,g_rel)

