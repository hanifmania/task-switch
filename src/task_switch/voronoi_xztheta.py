#!usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import math
from scipy.stats import norm
from task_switch.voronoi import VoronoiTheta1d


class VoronoiXThetaCompressed(VoronoiTheta1d):
    @classmethod
    def getDist(self, p, q):
        return p[0] - q

    @classmethod
    def projection(self, q):
        return q


class VoronoiXYZTheta(VoronoiTheta1d):
    @classmethod
    def getDist(self, p, q):
        return np.sqrt((p[0] - q[0]) ** 2 + (p[1] - q[1]) ** 2)

    def calcdJdp(self):
        region = self._region
        grid = self._field.getGrid(region)
        phi = self._field.getPhi(region)
        dist = self.getDist(self._pos, grid)
        h_i = norm.pdf(dist, scale=self._sigma) * math.sqrt(2 * math.pi) * self._sigma
        x_q = self._pos[0] - grid[0]
        y_q = self._pos[1] - grid[1]
        temp = (
            self._delta * h_i * phi / (self._sigma ** 2) * self._field.getPointDense()
        )
        # print(-x_q * temp)
        dJdp_x = np.sum(-x_q * temp)
        dJdp_y = np.sum(-y_q * temp)

        self._dJdp = [dJdp_x, dJdp_y]
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
        self._xi = -self._temp2 + self._delta * (-self._gamma / self._agent_num - dHdt)
        # self._xi = -self._temp2 - 2 * self._delta * dHdt - self._delta * self._delta * H # H<=0 HOCBF

        clock = 20
        self._d2Hdt2 = (dHdt - self._dHdt) * clock
        self._dHdt = dHdt

        near_field = dist < 2 * self._sigma
        self._near_psi_sum = np.sum(near_field * phi) * self._field.getPointDense()
