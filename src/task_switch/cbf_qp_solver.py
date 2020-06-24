#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
import cvxopt
from cvxopt import matrix

from task_switch.se3_operations import *

class CBFQPSolver():
    
    def __init__(self): 
        
        # set configrations
        self.set_solver_options(False, 1e1, 1e1, 1e-50, 100, 0)

        self.solver_state = None
 
    def set_solver_options(self, show_progress, abstol, reltol, feastol, maxiters, refinement):
        cvxopt.solvers.options['show_progress'] = show_progress #default Flase
        cvxopt.solvers.options['abstol'] = abstol #default 1e-2
        cvxopt.solvers.options['reltol'] = reltol #default 1e-2
        cvxopt.solvers.options['feastol'] = feastol #default 1e-7
        cvxopt.solvers.options['maxiters'] = maxiters #default 100
        cvxopt.solvers.options['refinement'] = refinement #default 0
   
    
    def __qp_solver(self, P_np, q_np, G_np, h_np):
        P=matrix(P_np)
        q=matrix(q_np)
        G=matrix(G_np)
        h=matrix(h_np)
        # print "P = \n", P, "\n q = \n", q, "\n G = \n", G, "\n h = \n", h
        sol=cvxopt.solvers.coneqp(P,q,G,h)
        return sol

    # def optimize(self, u_nominal, weight, G_list, h_list):
    def optimize(self, P, q, G_list, h_list):
        """
        Args:
            <u_nominal>:nominal_input(6x1)
            <weight>:weight_matrix(6x6)
            <G_lsit>:constraint_matrix_list[(1x6),...]
            <h_lsit>:constraint_matrix_lsit[(1x1),...]
        Returns:
            <u_optimal>:optimal_output(6x1)
        """
        minimize_scale = 1
        constraint_scale = 1
       
        P_np = P
        q_np = q
        G_np = np.concatenate(G_list, 0)
        h_np = np.concatenate(h_list, 0)

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
        delta = np.c_[np.array(sol['x'])[6:]]
        self.solver_state = sol['status']

        return u_optimal, delta


