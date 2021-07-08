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

    def getXi(self):
        return self._xi

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
    def __init__(self, field):
        super(VoronoiTheta1d, self).__init__(field)
        self._dHdt = 0
        self._temp2 = 0
        self._u = [1, 0]

    def setSigma(self, sigma):
        self._sigma = sigma

    def setGamma(self, gamma):
        self._gamma = gamma

    def getGamma(self):
        return self._gamma

    def setDelta(self, delta):
        self._delta = delta

    @classmethod
    def getDist(self, p, q):
        return np.abs(p[0] - self.projection(q))

    @classmethod
    def projection(self, q):
        z = 1.0
        return q[0] - z*np.tan(q[1])

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
        phi = self._field.getPhi(region)
        # dist = self.getDist(self._pos, grid)
        p_q = self._pos[0] - self.projection(grid)
        h_i = norm.pdf(p_q, scale=self._sigma) * math.sqrt(2 * math.pi) * self._sigma
        temp = -p_q * h_i * phi / (self._sigma ** 2)

        ####### h = 1 or 0
        # epsilon = 0.01
        # temp_minus = self.getDist(self._pos - epsilon, grid) < self._sigma
        # temp_plus = self.getDist(self._pos + epsilon, grid) < self._sigma
        # temp = np.sum(temp_plus * self._field.getPhi(region)) - np.sum(
        #     temp_minus * self._field.getPhi(region)
        # )
        # temp = temp / (2 * epsilon)
        ########

        dJdp = self._delta * np.sum(temp) * self._field.getPointDense()
        self._dJdp = [dJdp, 0]
        dHdt = -self._delta * np.sum(h_i * phi) * self._field.getPointDense()

        self._temp2 = (
            (self._delta ** 2) * np.sum(h_i * h_i * phi) * self._field.getPointDense()
        )

        H = np.sum(phi) * self._field.getPointDense()
        ### dHdt<=-gamma*H CBF
        # self._xi = (
        #     -self._temp2 + self._delta * (-self._gamma * H - dHdt) - self._gamma * dHdt
        # )
        ###dHdt <= -gamma CBF
        self._xi = -self._temp2 + self._delta * (-self._gamma - dHdt)
        # self._xi = -self._temp2 - 2 * self._delta * dHdt - self._delta * self._delta * H # H<=0 HOCBF

        clock = 20
        self._d2Hdt2 = (dHdt - self._dHdt) * clock
        self._dHdt = dHdt

    def getdHdt(self):
        return self._dHdt, self._d2Hdt2

    def getH(self):
        phi = self._field.getPhi()
        ret = np.sum(phi) * self._field.getPointDense()
        return ret

    def setU(self, u):
        self._u = u


class VoronoiTheta2d(VoronoiTheta1d):
    @classmethod
    def getDist(self, p, q):
        return np.sqrt((p[0] - q[0]) ** 2 + (p[1] - q[1]) ** 2)

    @classmethod
    def projection(self, q):
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
        q = self.projection(grid)

        self._dJdp = [
            np.sum(temp * (self._pos[0] - q[0])),
            np.sum(temp * (self._pos[1] - q[1])),
        ]
