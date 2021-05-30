#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np


class Field:
    def __init__(self, param):
        self._param = param
        dim = param["dim"]
        mesh_acc = self._param["mesh_acc"]
        # dimension is inverse to X,Y
        self._grid_shape = list(reversed(mesh_acc))
        self._phi = np.ones(self._grid_shape)
        linspace = [
            np.linspace(
                self._param["limit"][i][0], self._param["limit"][i][1], mesh_acc[i]
            )
            for i in range(dim)
        ]
        # self._grid = np.meshgrid(linspace[0], linspace[1])
        self._grid = np.meshgrid(*linspace)

        self._point_dense = 1
        for i in range(dim):
            self._point_dense *= self.getGridSpan(i)

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
            ret = self._grid[0] * region, self._grid[1] * region
        return ret

    def getGridSpan(self, i):
        return (
            float(self._param["limit"][i][1] - self._param["limit"][i][0])
            / self._param["mesh_acc"][i]
        )

    def getLimit(self, axes):
        return self._param["limit"][axes]

    def getPointDense(self):
        return self._point_dense
