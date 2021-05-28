#!usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import math

from task_switch.voronoi_main import Voronoi


class Theta1dVoronoi(Voronoi):
    def __init__(self, field):
        super(Theta1dVoronoi, self).__init__(field)
        self._z = 1
        self._theta_max = math.pi / 4
        self._A = np.array([1, -self._z * math.tan(self._theta_max) / self._theta_max])
        A_norm = np.linalg.norm(self._A, ord=2)
        self._coefficient = 1 / A_norm  # * 500
        self._epsilon = 0.1
        self._line_width = float(self.X.max() - self.X.min()) / field.mesh_acc[0]

    def getParallelogram(self, center_x, width):
        # warn: the width of result is "width"*2
        tmp = self._A[0] * self.X + self._A[1] * self.Y
        ret1 = tmp <= center_x + width
        ret2 = center_x - width <= tmp
        ret = ret1 * ret2
        return ret

    def calcVoronoi(self):
        self.Region = self.getParallelogram(self.Pos[0], self._epsilon)

    def calcdJdp(self):
        right_line = self.getParallelogram(
            self.Pos[0] + self._epsilon, self._line_width
        )
        left_line = self.getParallelogram(self.Pos[0] - self._epsilon, self._line_width)

        right_sum = np.sum(self.phi * right_line) * self.pointDense
        left_sum = np.sum(self.phi * left_line) * self.pointDense
        # print(right_sum, left_sum)
        is_right_unique = 1
        is_left_unique = 1
        for neighborPos in self.listNeighborPos:
            if 0 < self.Pos[0] - neighborPos[0] < 2 * self._epsilon:
                is_left_unique = 0
            elif 0 < -(self.Pos[0] - neighborPos[0]) < 2 * self._epsilon:
                is_right_unique = 0

        self._dJdp = (
            # self._coefficient
            (is_right_unique * right_sum - is_left_unique * left_sum)
            / (2 * self._line_width)
        )

    def getdJdp(self):
        return self._dJdp

    def update_agentParam(self, R, b_):
        self.R = R
        self.b = -(R ** 2) - b_

    def update_fieldParam(self, delta_decrease, delta_increase):
        self.delta_decrease = delta_decrease
        self.delta_increase = delta_increase

    def update_PccCBFParam(self, gamma, k):
        self.gamma = gamma
        self.k = k

    def getJintSPlusb(self):
        return self.JintSPlusb

    def getXi(self):
        # return self.xi
        return -self.gamma * self.delta_decrease
        # return (self.k-self.delta_decrease) *
