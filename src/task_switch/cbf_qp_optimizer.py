#!/usr/bin/env python
# -*- coding: utf-8 -*-
import warnings
import numpy as np

from task_switch.se3_operations import *

from task_switch.cbf_slack_qp_solver import CBFSLACKQPSolver

from task_switch.cbf import generalCBF,pnorm2dCBF,chargeCBF
# from vfc_cbf_controller.fov_cbf import FoVCBF
# from vfc_cbf_controller.collision_cbf import CollisionCBF
# from vfc_cbf_controller.attitude_cbf import AttitudeCBF



class CBFOptimizer(object):
    
    def __init__(self): 
        
        self.slack_qp_solver = CBFSLACKQPSolver()

        # self.set_optimize_weights([1.0, 1.0, 1.0, 50.0, 1.0, 1.0])
        self.input_weight_matrix = self.get_weight_matrix([1.0, 1.0, 1.0, 1.0, 1.0, 1.0])

        self.u_nominal = np.zeros((6,1))
        self.u_optimal = np.zeros((6,1))
        self.G_list = []
        self.h_list = []
        self.NeighborPos = [] # will be AgentNum-1 x 2 matrix
    
        
        self.activate_cbf = True
        self.activate_pnormcbf = True
        self.activate_chargeCBF = False
        self.activate_pccCBF = False
        # self.activate_fov = False
        # self.activate_ca = False
        # self.activate_aa = False


        # CBF instances
        self.pnormcbf = pnorm2dCBF()
        self.chargecbf = chargeCBF()
        self.pcccbf = generalCBF()

        # self.fov_cbf = FoVCBF(po)
        # self.collision_cbf = CollisionCBF()
        # self.attitude_cbf = AttitudeCBF()


        # pnorm initialize (must be overwritten)
        centPos = [0.0,0.0]
        theta = 0
        norm = 2
        width = [.5,.5]
        keepInside = False
        self.setPnormArea(centPos,theta,norm,width,keepInside)

        # energy initialize (must be overwritten)
        minEnergy = 1500
        Kd = 50
        k_charge = 0.15
        chargePos = [2.0,2.0]
        radiusCharge = 0.2
        self.setChargeStation(chargePos,radiusCharge)
        self.setChargeSettings(minEnergy,Kd,k_charge)

    def setPnormArea(self,centPos,theta,norm,width,keepInside):
        """
        Args: 
            <CentPos>:center of ellipsoid (list, 1x2)
            <theta>:rotation angle (rad)
            <norm>:p-norm
            <width>:half width and half height of ellipsoid(list, 1x2)
            <AgentPos>:position of agent (list, 1x6)
            <keepInside>:True->prohibit going outside of ellipsoid, False->prohibit entering inside of ellipsoid
        """
        self.centPos = centPos
        self.theta = theta
        self.norm = norm
        self.width = width
        self.keepInside = keepInside

    def getPnormArea(self):
        return self.centPos, self.theta, self.norm, self.width, self.keepInside

    def calcPnormConstraint(self,AgentPos):
        centPos,theta,norm,width,keepInside = self.getPnormArea()
        self.pnormcbf.setPnormSetting(centPos,theta,norm,width,keepInside)
        self.pnormcbf.calcConstraint(AgentPos)

    def getPnormConstraint(self):
        G, h = self.pnormcbf.getConstraint()
        return G, h



    def setChargeStation(self, chargePos, radiusCharge):
        self.chargePos = chargePos
        self.radiusCharge = radiusCharge

    def setChargeSettings(self,minEnergy, Kd, k_charge):
        self.minEnergy = minEnergy
        self.Kd = Kd
        self.k_charge = k_charge

    def getChargeStation(self):
        return self.minEnergy, self.Kd, self.k_charge, self.chargePos, self.radiusCharge

    def calcChargeConstraint(self,AgentPos,energy):
        minEnergy, Kd, k_charge, chargePos, radiusCharge = self.getChargeStation()
        self.chargecbf.setChargeSettings(minEnergy, Kd, k_charge, chargePos, radiusCharge)
        self.chargecbf.calcConstraint(AgentPos,energy)

    def getChargeConstraint(self):
        G, h = self.chargecbf.getConstraint()
        return G, h




    def setPccConstraint(self,G,h):
        self.pcccbf.setConstraintMatrix(G)
        self.pcccbf.setConstraintValue(h)

    def getPccConstraint(self):
        G, h = self.pcccbf.getConstraintSetting()
        return G, h


        



    def get_weight_matrix(self, weight_list):
        weight_matrix = np.diag([w for w in weight_list])
        return weight_matrix

    def set_optimize_slack_weights(self, weight_list):
        self.slack_weight_matrix = np.diag([w for w in weight_list])

    def set_qp_problem(self):
        """
        define the following optimization problem

        min_{u,w} (1/2) * (u-u_nom)^T*P*(u-u_nom) + (1/2)*w^T*Q*w
        subject to G*u + h >= Hw

        all are numpy
        Args: m is the number of constraints
            <u_nom>:nominal_input(6 x 1)
            <P>:optimization_weight_matrix for input(6 x 6)
            <Q>:slack_variable_weight_matrix (m x m)
            <G>:constraint_matrix (m x 6)
            <h>:constraint_matrix (m x 1)
            <H>:soft constraint flag matrix (m x m)
        Returns:
            <u_optimal>:optimal_output(6 x 1)
            <w>:optimal_slack(m x 1)
        """

        self.P = self.input_weight_matrix   


        # self.G_list = np.empty(0)
        # self.h_list = np.empty(0)
        # self.Q = np.empty(0)
        # self.H = np.empty(0)
        G_list = [] 
        h_list = [] 
        Q = [] 
        H = [] 
        slack_weight_list = []
        slack_flag_list = []

        if self.activate_cbf == True:

            if self.activate_pnormcbf == True:
                """
                Args: 
                    <CentPos>:center of ellipsoid (list, 1x2)
                    <theta>:rotation angle (rad)
                    <norm>:p-norm
                    <width>:half width and half height of ellipsoid(list, 1x2)
                    <AgentPos>:position of agent (list, 1x6)
                    <keepInside>:True->prohibit going outside of ellipsoid, False->prohibit entering Inside of ellipsoid
                """
                dhdp, h = self.getPnormConstraint()
                G_list.append(dhdp)
                h_list.append(h)
                slack_weight_list.append(1.)
                slack_flag_list.append(0.) # if soft constraint -> 1. if hard -> 0.

            if self.activate_chargeCBF == True:
                dhdp, h = self.getChargeConstraint()
                G_list.append(dhdp)
                h_list.append(h)
                slack_weight_list.append(1.)
                slack_flag_list.append(0.) # if soft constraint -> 1. if hard -> 0.

            # if self.activate_ca == True :
            #     if len(self.g_adj_list) > 0:
            #         G_ca, h_ca = self.collision_cbf.constraint_matrix(self.g_adj_list)
            #         self.G_list.append(G_ca)
            #         self.h_list.append(h_ca)

            if self.activate_pccCBF == True :
                dhdp, h = self.getPccConstraint()
                G_list.append(dhdp)
                h_list.append(h)
                slack_weight_list.append(1.)
                slack_flag_list.append(0.) # if soft constraint -> 1. if hard -> 0.

                
            self.h_list = np.array(h_list)
            self.G_list = np.array(G_list)

            # resize G matrix for soft constraint elements
            # if len(self.G_list) > 0:
            #     for G in self.G_list:
            #         G.resize(max([G.shape for G in self.G_list]), refcheck=True)
            m = self.h_list.shape[0]

            # print slack_weight_list

            self.Q = self.get_weight_matrix(slack_weight_list)
            self.H = self.get_weight_matrix(slack_flag_list)
            # print m, self.Q


    def optimize(self,u_nom, AgentPos, energy):
        self.calcChargeConstraint(AgentPos,energy)
        self.calcPnormConstraint(AgentPos)
        self.set_qp_problem()
        # u_optimal is optimized input, delta is slack variables value for soft constraints
        self.u_optimal, self.delta = self.slack_qp_solver.optimize(u_nom, self.P, self.Q, self.G_list, self.h_list, self.H)
        return self.u_optimal


if __name__ == '__main__':
    optimizer = CBFOptimizer()

    # pnorm set 
    centPos = [0.0,0.0]
    theta = 0
    norm = 2
    width = [.5,.5]
    keepInside = True
    optimizer.setPnormArea(centPos,theta,norm,width,keepInside)

    # energy station set
    minEnergy = 1500
    Kd = 50
    k_charge = 0.15
    chargePos = [2.0,2.0]
    radiusCharge = 0.2
    optimizer.setChargeStation(chargePos,radiusCharge)
    optimizer.setChargeSettings(minEnergy,Kd,k_charge)

    u_nom = np.array([0., 0., 0. ,0., 0., 0.]).reshape(-1,1)
    AgentPos = [3., 0., 0., 0., 0., 0.]
    energy = 1000
    print optimizer.optimize(u_nom, AgentPos, energy)
    

