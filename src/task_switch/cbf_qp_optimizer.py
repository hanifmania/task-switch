#!/usr/bin/env python
# -*- coding: utf-8 -*-
import warnings
import numpy as np
import copy

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
        # print self.neighborPosOnlyList[0]
    
        
        self.activate_cbf = True
        self.activate_pnormcbf = False 
        self.activate_chargeCBF = True
        self.activate_pccCBF = True
        self.activate_collisionCBF = False
        self.pnormcbf_slack_weight = 1.0
        self.chargecbf_slack_weight = 1.0
        self.pcccbf_slack_weight = 1.0

        # input range constraint
        self.activate_umax = True

        self.umax = 2.0
        self.umin = -2.0

        
        # self.activate_fov = False
        # self.activate_ca = False
        # self.activate_aa = False


        # CBF instances
        self.pnormcbf = pnorm2dCBF()
        self.chargecbf = chargeCBF()
        self.pcccbf = generalCBF()
        self.collisioncbf = pnorm2dCBF()

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




    def calcPccConstraint(self,G,h):
        self.pcccbf.calcConstraint(G,h)

    def getPccConstraint(self):
        G, h = self.pcccbf.getConstraint()
        return G, h






    def updateInputRange(self,flag,umax,umin):
        self.activate_umax = flag
        self.umax = umax
        self.umin = umin

        
    def listAppend(self,G_list,h_list,slack_weight_list,slack_flag_list,dhdp,h,weight):
        G_list.append(dhdp)
        h_list.append(h)

        # if slack weight is 0, then consider the constraint as "HARD"(never allowed to violate)
        # otherwise, "Soft"(allow to violate)
        if weight > 0.:
            slack_weight_list.append(weight)
            slack_flag_list.append(1.)# if soft constraint -> 1. if hard -> 0.
        else:
            slack_weight_list.append(1.)
            slack_flag_list.append(0.) # if soft constraint -> 1. if hard -> 0.

        return G_list, h_list, slack_weight_list, slack_flag_list


    def get_weight_matrix(self, weight_list):
        weight_matrix = np.diag([w for w in weight_list])
        return weight_matrix

    def set_optimize_slack_weights(self, weight_list):
        self.slack_weight_matrix = np.diag([w for w in weight_list])

    def set_qp_problem(self):
        """
        define the following optimization problem

        min_{u,w} (1/2) * (u-u_nom)^T*P*(u-u_nom) + (1/2)*w^T*Q*w
        subject to G*u + h >= Rw

        all are numpy
        Args: m is the number of constraints
            <u_nom>:nominal_input(6 x 1)
            <P>:optimization_weight_matrix for input(6 x 6)
            <Q>:slack_variable_weight_matrix (m x m)
            <G>:constraint_matrix (m x 6)
            <h>:constraint_matrix (m x 1)
            <R>:soft constraint flag matrix (m x m)
        Returns:
            <u_optimal>:optimal_output(6 x 1)
            <w>:optimal_slack(m x 1)
        """

        self.P = self.input_weight_matrix   


        # self.G_list = np.empty(0)
        # self.h_list = np.empty(0)
        # self.Q = np.empty(0)
        # self.R = np.empty(0)
        G_list = [] 
        h_list = [] 
        Q = [] 
        R = [] 
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
                weight = self.pnormcbf_slack_weight
                G_list, h_list, slack_weight_list, slack_flag_list \
                        = self.listAppend(G_list, h_list, slack_weight_list, slack_flag_list, dhdp, h, weight)

            if self.activate_chargeCBF == True:
                dhdp, h = self.getChargeConstraint()
                weight = self.chargecbf_slack_weight
                G_list, h_list, slack_weight_list, slack_flag_list \
                        = self.listAppend(G_list, h_list, slack_weight_list, slack_flag_list, dhdp, h, weight)


            if self.activate_collisionCBF == True:
                G_list.extend(self.ColG_list)
                h_list.extend(self.Colh_list)
                slack_weight_list.extend(self.Colslack_weight_list)
                slack_flag_list.extend(self.Colslack_flag_list)
                

                # if len(self.g_adj_list) > 0:
                #     G_ca, h_ca = self.collision_cbf.constraint_matrix(self.g_adj_list)
                #     self.G_list.append(G_ca)
                #     self.h_list.append(h_ca)


            if self.activate_pccCBF == True:
                dhdp, h = self.getPccConstraint()
                weight = self.pcccbf_slack_weight
                G_list, h_list, slack_weight_list, slack_flag_list \
                        = self.listAppend(G_list, h_list, slack_weight_list, slack_flag_list, dhdp, h, weight)

            if self.activate_umax == True:
                # replace by np.eyes? 
                G = [[-1.,0.,0.,0.,0.,0.],[0.,-1.,0.,0.,0.,0.],[1.,0.,0.,0.,0.,0.],[0.,1.,0.,0.,0.,0.]]
                # h becomes [[umax],[umax],[umin],[umin]]
                h = [[self.umax]]*2 + [[-self.umin]]*2

                G_list.extend(G)
                h_list.extend(h)

                # treat as hard constraint
                slack_weight_list.extend([1.]*len(h))
                slack_flag_list.extend([0.]*len(h))


            

                
            self.h_list = np.array(h_list)
            self.G_list = np.array(G_list)

            # resize G matrix for soft constraint elements
            # if len(self.G_list) > 0:
            #     for G in self.G_list:
            #         G.resize(max([G.shape for G in self.G_list]), refcheck=True)
            m = self.h_list.shape[0]

            # print slack_weight_list

            self.Q = self.get_weight_matrix(slack_weight_list)
            self.R = self.get_weight_matrix(slack_flag_list)
            # print m, self.Q


    def optimize(self,u_nom, AgentPos, energy, dJdp, xi):
        if self.activate_cbf == True:
            self.calcChargeConstraint(AgentPos,energy)
            self.calcPnormConstraint(AgentPos)
            self.calcPccConstraint(dJdp,xi)

            # 一番近いのだけ衝突判定すればよいのでは？
            self.neighborPosOnly = np.array([[1.,1.],[3.5,-0.3],[1.,-1.]])
            self.neighborPosOnlyList = self.neighborPosOnly.tolist()
            ColG_list = []
            Colh_list = []
            Colslack_weight_list = []
            Colslack_flag_list = []
            for eachNeighborPos in self.neighborPosOnlyList:
                G, h = self.calcNeighborCBF(AgentPos,eachNeighborPos)
                weight = 0.
                ColG_list, Colh_list, Colslack_weight_list, Colslack_flag_list \
                        = self.listAppend(ColG_list, Colh_list, Colslack_weight_list, Colslack_flag_list, G, h, weight)
            
            print ColG_list
            print Colh_list
            self.ColG_list = ColG_list
            self.Colh_list = Colh_list
            self.Colslack_weight_list = Colslack_weight_list
            self.Colslack_flag_list = Colslack_flag_list
            

            self.set_qp_problem()
            # u_optimal is optimized input, delta is slack variables value for soft constraints
            self.u_optimal, self.delta, self.status = self.slack_qp_solver.optimize(u_nom, self.P, self.Q, self.G_list, self.h_list, self.R)
            return self.u_optimal, self.status
        else:
            return u_nom

    def calcNeighborCBF(self,AgentPos,eachNeighborPos):
        agent_R = .3*2
        centPos = eachNeighborPos
        theta = 0.
        norm = 2
        width = [agent_R, agent_R]
        self.collisioncbf.setPnormSetting(centPos,theta,norm,width,False)
        self.collisioncbf.calcConstraint(AgentPos)
        G, h = self.collisioncbf.getConstraint()
        A = copy.copy(G)
        b = copy.copy(h)
        return A,b

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
    dJdp = [0.] * 6
    xi = [0.]

    print optimizer.optimize(u_nom, AgentPos, energy, dJdp, xi)
    

