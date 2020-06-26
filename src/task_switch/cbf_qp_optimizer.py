#!/usr/bin/env python
# -*- coding: utf-8 -*-
import warnings
import numpy as np

from task_switch.se3_operations import *

from task_switch.cbf_slack_qp_solver import CBFSLACKQPSolver

from task_switch.cbf import pnorm2dCBF
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
        # self.activate_fov = False
        # self.activate_ca = False
        # self.activate_aa = False

        # CBF instances
        self.pnormcbf = pnorm2dCBF()
        # self.fov_cbf = FoVCBF(po)
        # self.collision_cbf = CollisionCBF()
        # self.attitude_cbf = AttitudeCBF()

    def get_weight_matrix(self, weight_list):
        weight_matrix = np.diag([w for w in weight_list])
        return weight_matrix

    def set_optimize_slack_weights(self, weight_list):
        self.slack_weight_matrix = np.diag([w for w in weight_list])

    def set_qp_problem(self, AgentPos):
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
        self.G_list = [] 
        self.h_list = [] 
        self.Q = [] 
        self.H = [] 
        self.slack_weight_list = []
        self.slack_flag_list = []

        if self.activate_cbf == True:
            # if self.activate_fov == True:

            #     if g_target is not None:
            #         G_hoge, h_hoge
            #         G_fov, h_fov = self.fov_cbf.constraint_matrix(g_target, observer_input)
            #         self.G_list.append(G_fov)
            #         self.h_list.append(h_fov)
            #     else :
            #         warnings.warn("g_target is None, fovcbf is not activated")

            # if self.activate_ca == True :
            #     if len(self.g_adj_list) > 0:
            #         G_ca, h_ca = self.collision_cbf.constraint_matrix(self.g_adj_list)
            #         self.G_list.append(G_ca)
            #         self.h_list.append(h_ca)

            if self.activate_pnormcbf == True:
                """
                Args: 
                    <CentPos>:center of ellipsoid (list, 1x2)
                    <theta>:rotation angle (rad)
                    <norm>:p-norm
                    <width>:half width and half height of ellipsoid(list, 1x2)
                    <AgentPos>:position of agent (list, 1x6)
                    <inside>:True->prohibit going outside of ellipsoid, False->prohibit entering inside of ellipsoid
                """
                centPos = [0.0,0.0]
                theta = 0
                norm = 2
                width = [.5,.5]
                inside = False
                self.pnormcbf.setPnormSetting(centPos,theta,norm,width,inside)
                dhdp, h = self.pnormcbf.getConstraintSetting(AgentPos)
                self.G_list.append(dhdp)
                self.h_list.append(h)
                
            self.h_list = np.array(self.h_list)
            self.G_list = np.array(self.G_list)

            # resize G matrix for soft constraint elements
            # if len(self.G_list) > 0:
            #     for G in self.G_list:
            #         G.resize(max([G.shape for G in self.G_list]), refcheck=True)
            m = self.h_list.shape[0]

            slack_weight_list = np.ones((1,m))
            slack_flag_list = np.ones((1,m))

            self.Q = self.get_weight_matrix(slack_weight_list)
            self.H = self.get_weight_matrix(slack_flag_list)



    def optimize(self,u_nom, AgentPos):
        self.set_qp_problem(AgentPos)
        # u_optimal is optimized input, delta is slack variables value for soft constraints
        self.u_optimal, self.delta = self.slack_qp_solver.optimize(u_nom, self.P, self.Q, self.G_list, self.h_list, self.H)
        return self.u_optimal


if __name__ == '__main__':
    optimizer = CBFOptimizer()
    u_nom = np.array([0., 0., 0. ,0., 0., 0.]).reshape(-1,1)
    AgentPos = [3., 0., 0., 0., 0., 0.]
    print optimizer.optimize(u_nom, AgentPos)
    

