#!/usr/bin/env python
# -*- coding: utf-8 -*-
import warnings
import numpy as np

from task_switch.se3_operations import *

from task_switch.cbf_slack_qp_solver import CBFSLACKQPSolver

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
        self.activate_fov = False
        self.activate_ca = False
        self.activate_aa = False

        # CBF instances
        self.fov_cbf = FoVCBF(po)
        self.collision_cbf = CollisionCBF()
        self.attitude_cbf = AttitudeCBF()

    def get_weight_matrix(self, weight_list):
        weight_matrix = np.diag([w for w in weight_list])
        return weight_matrix

    def set_optimize_slack_weights(self, weight_list):
        self.slack_weight_matrix = np.diag([w for w in weight_list])

    def set_qp_problem(self, u_nominal, g_target, observer_input=None):
        """
        define the following optimization problem

        min_{u,w} (1/2) * (u-u_nom)^T*P*(u-u_nom) + (1/2)*w^T*Q*w
        subject to G*u + h >= Hw

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

        self.u_nom = u_nominal

        self.P = self.input_weight_matrix   


        self.G_list = np.Empty(0)
        self.h_list = np.Empty(0)
        self.Q = np.Empty(0)
        self.H = np.Empty(0)
        self.slack_weight_list = []
        self.slack_flag_list = []

        if self.activate_cbf == True:
            if self.activate_fov == True:
                if g_target is not None:
                    G_hoge, h_hoge
                    G_fov, h_fov = self.fov_cbf.constraint_matrix(g_target, observer_input)
                    self.G_list.append(G_fov)
                    self.h_list.append(h_fov)
                else :
                    warnings.warn("g_target is None, fovcbf is not activated")

            if self.activate_ca == True :
                if len(self.g_adj_list) > 0:
                    G_ca, h_ca = self.collision_cbf.constraint_matrix(self.g_adj_list)
                    self.G_list.append(G_ca)
                    self.h_list.append(h_ca)

            if self.activate_aa == True :
                if len(self.g_adj_list) > 0:
                    G_aa, h_aa, P_aa_add, q_aa_add = self.attitude_cbf.constraint_matrix(self.g_adj_list)
                    self.G_list.append(G_aa)
                    self.h_list.append(h_aa)

                    self.P = np.vstack([
                                np.hstack([
                                    self.P,
                                    np.zeros([len(self.P), len(P_aa_add)])
                                    ]),
                                np.hstack([
                                    np.zeros([len(P_aa_add), 6]),
                                    P_aa_add
                                    ])
                            ])
                    self.q = np.vstack([
                                self.q,
                                q_aa_add
                            ])

                    G_length_max = max([G.shape[1] for G in self.G_list]) 
                    for i, G in enumerate(self.G_list):
                        if G.shape[1] < G_length_max:
                            self.G_list[i] = np.hstack([
                                                G,
                                                np.zeros([G.shape[0], (G_length_max - G.shape[1])])
                                            ])

            # resize G matrix for soft constraint elements
            # if len(self.G_list) > 0:
            #     for G in self.G_list:
            #         G.resize(max([G.shape for G in self.G_list]), refcheck=True)
            m = h_list.shape[0]

            slack_weight_list = np.ones((1,m))
            slack_weight_list = np.ones((1,m))

            self.Q = self.get_weight_matrix(slack_weight_list)
            self.H = self.get_weight_matrix(slack_flag_list)

        return self.u_nom, self.P, self.Q, self.G_list, self.h_list, self.H


    def optimize(self):
        self.u_optimal, self.delta = self.slack_qp_solver.optimize(self.u_nom, self.P, self.Q, self.G_list, self.h_list, self.H)
        return self.u_optimal


