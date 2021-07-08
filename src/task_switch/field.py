#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np


class Field:
    def __init__(self, param):
        self._param = param
        mesh_acc = np.array(self._param["mesh_acc"])
        dim = len(mesh_acc)
        self._mesh_acc = mesh_acc
        self._limit = np.array(self._param["limit"])
        # dimension is inverse to X,Y
        self._grid_shape = list(reversed(mesh_acc))
        self._phi = np.ones(self._grid_shape)

        ### make mesh
        linspace = [
            np.linspace(
                self._param["limit"][i][0], self._param["limit"][i][1], mesh_acc[i]
            )
            for i in range(dim)
        ]
        temp = dim - 2
        linspace_sorted = linspace[temp:] + list(reversed(linspace[:temp]))

        # if dim == 3:
        #     linspace = linspace[1:] + linspace[:1]
        # elif dim == 4:
        #     linspace = linspace[2:] + linspace[1:2] + linspace[:1]
        # elif dim == 5:
        #     linspace = linspace[2:] + linspace[1:2] + linspace[:1]
        self._grid = np.meshgrid(*linspace_sorted)
        self._grid = list(reversed(self._grid[2:])) + self._grid[:2]
        # if dim == 3:
        #     self._grid = self._grid[2:] + self._grid[:2]
        # elif dim == 4:
        #     self._grid = self._grid[3:] + self._grid[2:3] +  self._grid[:2]

        self._zero_index = [np.where(linspace[i] == 0) for i in range(dim)]
        # print(self._zero_index)
        # self._zero_index = np.stack(self._zero_index)
        self._point_dense = self.getGridSpan().prod()

    def setPhi(self, phi):
        self._phi = phi

    def getPhi(self, region=None):
        if region is None:
            ret = self._phi
        else:
            ret = self._phi * region
        return ret

    def getGrid(self, region=None):
        if region is None:
            ret = self._grid
        else:
            ret = self._grid * region
        return ret

    def getGridSpan(self):
        return (self._limit[:, 1] - self._limit[:, 0]) / (self._mesh_acc - 1)

    def getLimit(self, axes):
        return self._limit[axes]

    def getPointDense(self):
        return self._point_dense

    def getZeroIndex(self):
        return self._zero_index
