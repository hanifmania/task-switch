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
        self.set_optimize_weights([1.0, 1.0, 1.0, 1.0, 1.0, 1.0])

        self.u_nominal = np.zeros((6,1))
        self.u_optimal = np.zeros((6,1))
        self.G_list = []
        self.h_list = []
    
        self.g_adj_list = []
        
        self.activate_cbf = True
        self.activate_fov = False
        self.activate_ca = False
        self.activate_aa = False

        po = np.zeros([4,3])
        # CBF instances
        self.fov_cbf = FoVCBF(po)
        self.collision_cbf = CollisionCBF()
        self.attitude_cbf = AttitudeCBF()

    def set_optimize_weights(self, weight_list):
        self.weight_matrix = np.diag([w** 2 for w in weight_list])

    def set_qp_variable(self, u_nominal, g_target, observer_input=None):
        self.u_nominal = u_nominal

        self.P = self.weight_matrix   
        self.q = (-1 * u_nominal.T.dot(self.weight_matrix)).T 

        self.G_list = []
        self.h_list = []

        if self.activate_cbf == True:
            if self.activate_fov == True:
                if g_target is not None:
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

        return self.P, self.q, self.G_list, self.h_list


    def optimize(self):
        self.u_optimal, self.delta = self.qp_solver.optimize(self.P, self.q, self.G_list, self.h_list)
        return self.u_optimal


