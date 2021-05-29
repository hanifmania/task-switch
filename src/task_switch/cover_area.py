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


class CoverAreaBase(VoronoiBase):
    ### these might be not good to base class
    def calcRegion(self):
        # self.Region = self.getParallelogram(self.Pos[0], self._epsilon)
        self._region = self._getSp(self._pos[0])

        neighbor_regions = []
        for neighborPos in self._neighbor_pos_list:
            neighbor_regions.append(self._getSp(neighborPos[0]))
        self._neighbor_regions = neighbor_regions

    def _eliminateNeighbors(self, origin):
        ret = origin
        for neighbor_region in self._neighbor_regions:
            ret = ret * ~neighbor_region
        return ret


class CoverAreaTheta1d(CoverAreaBase):
    def __init__(self, field):
        super(CoverAreaTheta1d, self).__init__(field)
        self._z = 1
        self._A = np.array([1, -self._z])
        self._epsilon = 0.1
        self._dp = field.getGridSpan(0)  # x span

    def _getSp(self, p):
        # warn: the width of result is "width"*2
        x_grid, y_grid = self._field.getGrid()
        width = self._epsilon
        tmp = self._A[0] * x_grid + self._A[1] * y_grid
        ret1 = tmp <= p + width
        ret2 = p - width <= tmp
        ret = ret1 * ret2
        return ret

    def update_fieldParam(self, delta_decrease, delta_increase):
        self.delta_decrease = delta_decrease
        self.delta_increase = delta_increase

    def update_PccCBFParam(self, gamma, k):
        self.gamma = gamma
        self.k = k

    def getXi(self):
        # return self.xi
        return -self.gamma * self.delta_decrease
        # return (self.k-self.delta_decrease) *

    def calcdJdp(self):
        phi = self._field.getPhi()
        region_plus = self._getSp(self._pos[0] + self._dp)
        region_plus = self._eliminateNeighbors(region_plus)
        region_minus = self._getSp(self._pos[0] - self._dp)
        region_minus = self._eliminateNeighbors(region_minus)
        # print(phi.shape)  # , size(region_plus), size(region_minus))
        temp = np.sum(phi * region_plus) - np.sum(phi * region_minus)
        # x_grid, y_grid = self._field.getGrid()
        # J_plus = phi * ~region_plus / ((self._pos[0] - x_grid * ~region_plus) **2 +(self._pos[1] - y_grid * ~region_plus) **2)
        # J_minus = phi * ~region_minus / ((self._pos[0] - x_grid * ~region_minus) **2 +(self._pos[1] - y_grid * ~region_minus) **2)
        # temp = np.sum(J_plus) - np.sum(J_minus)
        self._dJdp = [temp * self._field.getPointDense() / (2 * self._dp), 0]


class VoronoiTheta1d(CoverAreaTheta1d):
    # def __init__(self, field):
    #     super(VoronoiTheta1d, self).__init__(field)
    #     self._alpha = 1
    # self._voronoi = VoronoiBase(field)
    # print("self._A[1]", self._A[1])

    def getDist(self, p_x, q_x, q_theta):
        return p_x - q_x - np.tan(q_theta) * self._A[1]

    def _calcVoronoiRegion(self):
        x_grid, y_grid = self._field.getGrid()
        my_dist2 = self.getDist(self._pos[0], x_grid, y_grid) ** 2
        # print("my_dist2", my_dist2)
        region = np.ones(x_grid.shape, dtype=bool)
        for neighborPos in self._neighbor_pos_list:
            # distance to each point from neighbor
            neighbor_dist2 = self.getDist(neighborPos[0], x_grid, y_grid) ** 2
            my_region = my_dist2 < neighbor_dist2
            region = region * my_region
        # region = region * (self.getDist(self._pos[0], x_grid, y_grid) < 0)
        self._voronoi_region = region
        # print("sum(self._voronoi_region): ", np.sum(self._voronoi_region))

    def calcRegion(self):
        # super(VoronoiBase, self).calcRegion()
        self._calcVoronoiRegion()
        super(VoronoiTheta1d, self).calcRegion()

    def calcdJdp(self):
        # region = self._voronoi_region * ~self._region
        # x_grid, y_grid = self._field.getGrid(region)
        # dist = self.getDist(self._pos[0], x_grid, y_grid)
        # temp = -self._field.getPhi(region) / (dist * np.abs(dist))

        # self._dJdp = [np.sum(temp) * self._field.getPointDense(), 0]
        # dist = self.getDist(self._pos[0], x_grid, y_grid)

        region = self._voronoi_region
        self._tmp = region
        x_grid, y_grid = self._field.getGrid(region)
        dist = self.getDist(self._pos[0], x_grid, y_grid)

        scale = 0.1
        temp = (
            -norm.pdf(dist, scale=scale)
            / (scale ** 2)
            # * (math.sqrt(2 * math.pi) * scale)
            * dist
            * self._field.getPhi(region)
        )

        self._dJdp = [np.sum(temp) * self._field.getPointDense(), 0]
        dist = self.getDist(self._pos[0], x_grid, y_grid)

        # print(temp)
        # print(self.getDist(self._pos[0], x_grid, y_grid))
        # print("self._dJdp", self._dJdp[0])
        # print(np.sum(self._field.getPhi(region) > 0))


class CoverAreaTheta2d(CoverAreaTheta1d):
    def __init__(self, field):
        super(CoverAreaTheta2d, self).__init__(field)
        self._z = 1
        _, theta_max = field.getLimit(2)
        self._A = np.array([1, -self._z * math.tan(theta_max) / theta_max])
        self._epsilon = 0.1
        self._dp = field.getGridSpan(0)  # x span

    def _getSp(self, p):
        # warn: the width of result is "width"*2
        x_grid, y_grid = self._field.getGrid()
        width = self._epsilon
        tmp = self._A[0] * x_grid + self._A[1] * y_grid
        ret1 = tmp <= p + width
        ret2 = p - width <= tmp
        ret = ret1 * ret2
        return ret

    def update_fieldParam(self, delta_decrease, delta_increase):
        self.delta_decrease = delta_decrease
        self.delta_increase = delta_increase

    def update_PccCBFParam(self, gamma, k):
        self.gamma = gamma
        self.k = k

    def getXi(self):
        # return self.xi
        return -self.gamma * self.delta_decrease
        # return (self.k-self.delta_decrease) *

    def calcdJdp(self):
        phi = self._field.getPhi()
        region_plus = self._getSp(self._pos[0] + self._dp)
        region_plus = self._eliminateNeighbors(region_plus)
        region_minus = self._getSp(self._pos[0] - self._dp)
        region_minus = self._eliminateNeighbors(region_minus)
        # print(phi.shape)  # , size(region_plus), size(region_minus))
        temp = np.sum(phi * region_plus) - np.sum(phi * region_minus)
        self._dJdp = [temp * self._field.getPointDense() / (2 * self._dp), 0]
