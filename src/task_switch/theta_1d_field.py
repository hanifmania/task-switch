#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
from task_switch.voronoi_main import Field


class Theta1dField(Field):
    def __init__(self, mesh_acc, xlimit, ylimit):
        self.mesh_acc = mesh_acc
        self.xlimit = xlimit
        self.ylimit = ylimit
        # dimension is inverse to X,Y
        self.phi = np.ones((self.mesh_acc[1], self.mesh_acc[0]))
