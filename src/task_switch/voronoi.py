#!usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import math
from scipy.stats import norm


class VoronoiBase(object):
    def __init__(self, field):
        self._field = field
        self.R = 1000  # if you use r-limited voronoi, change self.R

    def setPos(self, pos):
        self._pos = pos

    def setNeighborPos(self, NeighborPos):
        self._neighbor_pos_list = NeighborPos

    def setPhi(self, phi):
        self._field.setPhi(phi)

    def getRegion(self):
        return self._region

    def getdJdp(self):
        return self._dJdp

    def calcdJdp(self):
        pass

    def getCenter(self):
        return self.cent

    def calcRegion(self):
        x_grid, y_grid = self._field.getGrid()
        point_dense = self._field.getPointDense()
        phi = self._field.getPhi()
        region = (x_grid - self._pos[0]) ** 2 + (
            y_grid - self._pos[1]
        ) ** 2 < self.R ** 2
        # create R+margin circle
        outside = (x_grid - self._pos[0]) ** 2 + (y_grid - self._pos[1]) ** 2 < (
            1.1 * self.R
        ) ** 2

        # calculate distance to each point in R+margin circle
        distance = (x_grid * outside - self._pos[0]) ** 2 + (
            y_grid * outside - self._pos[1]
        ) ** 2

        # eliminate R circle from outside (make donut)
        arc = outside * ~region

        for neighborPos in self._neighbor_pos_list:
            # distance to each point from neighbor
            neighborDistance = (x_grid * region - neighborPos[0]) ** 2 + (
                y_grid * region - neighborPos[1]
            ) ** 2
            # nearer points in R+margin circle
            nearRegion = neighborDistance > distance
            # delete points near to neighbor from my Region
            region = region * nearRegion
            # delete points near to neighbor from arc
            arc = arc * nearRegion

        # X-Y cordinates of my region which is weighted
        weightedX = x_grid * phi * region
        weightedY = y_grid * phi * region

        # calculate mass and cent
        self._region = region
        self.mass = np.sum(phi * region) * point_dense
        self.cent = (
            np.array([weightedX.sum(), weightedY.sum()]) * point_dense / self.mass
        )


class VoronoiTheta1d(VoronoiBase):
    def setSigma(self, sigma):
        self._sigma = sigma

    @classmethod
    def getDist(self, p, q):
        return np.abs(p[0] - self.profection(q))

    @classmethod
    def profection(self, q):
        return q[0] - np.tan(q[1])

    def calcRegion(self):
        grid = self._field.getGrid()
        my_dist2 = self.getDist(self._pos, grid)
        region = np.ones(grid[0].shape, dtype=bool)
        for neighborPos in self._neighbor_pos_list:
            neighbor_dist2 = self.getDist(neighborPos, grid)
            my_region = my_dist2 < neighbor_dist2
            region = region * my_region
        self._region = region

    def calcdJdp(self):
        region = self._region
        grid = self._field.getGrid(region)
        # dist = self.getDist(self._pos, grid)
        p_q = self._pos[0] - self.profection(grid)

        temp = (
            -norm.pdf(p_q, scale=self._sigma)
            / (self._sigma ** 2)
            * p_q
            * self._field.getPhi(region)
        )

        self._dJdp = [np.sum(temp) * self._field.getPointDense(), 0]
        # print(temp)
        # print(self.getDist(self._pos[0], x_grid, y_grid))
        # print("self._dJdp", self._dJdp[0])
        # print(np.sum(self._field.getPhi(region) > 0))


class VoronoiTheta2d(VoronoiTheta1d):
    @classmethod
    def getDist(self, p, q):
        return np.sqrt((p[0] - q[0]) ** 2 + (p[1] - q[1]) ** 2)

    @classmethod
    def profection(self, q):
        return q

    def calcdJdp(self):
        region = self._region
        grid = self._field.getGrid(region)
        dist = self.getDist(self._pos, grid)

        temp = (
            -norm.pdf(dist, scale=self._sigma)
            / (self._sigma ** 2)
            * self._field.getPhi(region)
            * self._field.getPointDense()
        )
        q = self.profection(grid)

        self._dJdp = [
            np.sum(temp * (self._pos[0] - q[0])),
            np.sum(temp * (self._pos[1] - q[1])),
        ]
