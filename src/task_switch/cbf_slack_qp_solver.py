#!/usr/bin/env python
# -*- coding: utf-8 -*-
import warnings
import numpy as np
import cvxopt
from cvxopt import matrix

from task_switch.se3_operations import *

from task_switch.cbf_qp_solver import CBFQPSolver

# from vfc_cbf_controller.fov_cbf import FoVCBF
# from vfc_cbf_controller.collision_cbf import CollisionCBF
# from vfc_cbf_controller.attitude_cbf import AttitudeCBF

class CBFSLACKQPSolver(CBFQPSolver):
    def __qp_solver(self, P_np, q_np, G_np, h_np):
        P=matrix(P_np)
        q=matrix(q_np)
        G=matrix(G_np)
        h=matrix(h_np)
        # print "P = \n", P, "\n q = \n", q, "\n G = \n", G, "\n h = \n", h
        sol=cvxopt.solvers.coneqp(P,q,G,h)
        return sol

    def optimize(self, u_nom, P, Q, G, h, H):
        """
        solve the following optimization problem

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
        minimize_scale = 1
        constraint_scale = 1

        m = h.shape[0]
        
       
        P_np = np.block([[P, np.zeros((6,m))], [np.zeros((m,6)), Q]])
        q_np = np.vstack( [-np.dot(P.T,u_nom), np.zeros((m,1))] )
        G_np = np.hstack([-G, H])
        h_np = h

        # print "P="
        # print P_np
        # print "q="
        # print q_np
        # print "G="
        # print G_np
        # print "h="
        # print h_np
        
        sol = self.__qp_solver(P_np * minimize_scale, q_np * minimize_scale, G_np * constraint_scale, h_np * constraint_scale)

        u_optimal = np.c_[np.array(sol['x'])[:6]]
        w = np.c_[np.array(sol['x'])[6:]]
        self.solver_state = sol['status']

        return u_optimal, w 

def main():
    slack_qp_solver = CBFSLACKQPSolver()
    u_nom = np.array([0.,0.,0.,0.,0.,0.]).reshape(-1,1)
    P = np.eye(6)
    Q = 100*np.eye(4)
    G = np.array([[1.,0.,0.,0.,0.,0.],[0.,1.,0.,0.,0.,0.],[1.,1.,0.,0.,0.,0.],[-1.,0.,0.,0.,0.,0.]])
    h = np.array([0.,0.,-1.,0.3]).reshape(-1,1)
    H = np.diag([0,0,1,0])

    (u_opt, w) = slack_qp_solver.optimize(u_nom, P, Q, G, h, H)
    print "u_opt = \n", u_opt, "\n w = \n", w

if __name__ == '__main__':
    main()

