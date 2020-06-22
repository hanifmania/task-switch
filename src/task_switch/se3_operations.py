#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
from transformations import *

def wedge(vec):
    return np.array([ 
                        [0, -1*vec[2], vec[1]],
                        [vec[2], 0, -1*vec[0]],
                        [-1*vec[1], vec[0], 0]
                    ])

def vec6_to_mat44(vec):
    return np.vstack([
                        np.hstack([wedge(vec[3:6]),vec[0:3]]),
                        np.hstack([np.zeros((1,3)),np.zeros((1,1))])
                        ])

def skew(R33):
    return 0.5 * (R33 - R33.T)
    
def vee(R33):
    vec = np.zeros((3,1))
    vec[0] = R33[2,1]
    vec[1] = R33[0,2]
    vec[2] = R33[1,0]
    return vec

def g_to_vec6(g):
    vec = np.zeros((6,1))
    vec[0:3] = g[0:3,3:4]
    vec[3:6] = vee(skew(g[0:3,0:3]))
    return vec

def adjoint_matrix(mat):
    # cf. passivity-book p91
    ad = np.zeros((6,6))

    if mat.shape == (4,4):
        ad[0:3,0:3] = mat[0:3,0:3]
        ad[3:6,3:6] = mat[0:3,0:3]
        ad[0:3,3:6] = wedge(mat[0:3,3]).dot(mat[0:3,0:3])
    elif mat.shape == (3,3):
        ad[0:3,0:3] = mat[0:3,0:3]
        ad[3:6,3:6] = mat[0:3,0:3]
    elif mat.shape == (3,1):
        ad[0:3,0:3] = np.eye(3)
        ad[3:6,3:6] = np.eye(3)
        ad[0:3,3:6] = wedge(mat)
    else:
        raise TypeError("input matrix dimension is incorrect")
    return ad

def vec6_adj_transformation(vec6, mat33):
    adj = adjoint_matrix(mat33)
    return adj.dot(vec6)

def point_rpy_to_gmatrix(x,y,z,R,P,Y, axes='sxyz'):
    
    gmat = np.eye(4)
    R33 = euler_matrix(R,P,Y, axes)[0:3,0:3]
    vec3 = np.c_[np.array([x,y,z])]
    gmat[0:3,0:3] = R33
    gmat[0:3,3:4] = vec3
    return gmat

def g_to_axis_angle(g_mat):
    rot_info = rotation_from_matrix(g_mat)
    return rot_info[0]

def g_reprojection(g_mat): # orthogonal projection (passivity book p243)
    rot_mat = g_mat[0:3,0:3]
    U, s, V = np.linalg.svd(rot_mat, full_matrices=True) # singular value decomposition
    g_mat[0:3,0:3] = U.dot(V)
    return g_mat

def g_reprojection2(g_mat): # matix -> axis angle -> matrix
    rot_info = transformations.rotation_from_matrix(g_mat)
    re_pro_mat = transformations.rotation_matrix(rot_info[0], rot_info[1],rot_info[2])
    g_mat[0:3,0:3] = re_pro_mat[0:3,0:3]
    return g_mat

def g_reprojection3(g_mat): # matrix -> quaternion -> matrix
    quaternion = transformations.quaternion_from_matrix(g_mat)
    re_pro_mat = transformations.quaternion_matrix(quaternion)
    g_mat[0:3,0:3] = re_pro_mat[0:3,0:3]
    return g_mat

