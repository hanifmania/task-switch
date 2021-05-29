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
