#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
from numpy.core.numeric import zeros_like
import pandas as pd

linspace = np.linspace(-2.0, 2.0, 5)
# zero_index = [np.where(linspace == 0) for i in range(1)]
# print(linspace)


# grid = np.array([[-0.51, 0, -1.4], [1.51, 1.4, 2]])
# print(grid)

# grid_span = np.array([4.0 / (5 - 1)])
# print(grid_span)

# target_index = np.round(grid / grid_span) + zero_index
# target_index = target_index.astype(np.int64)
# print(target_index)

# # comp = np.zeros_like(linspace)
# # for i, val in zip(target_index, grid):
# #     print(i, val)
# #     comp[i] += val
# # print(comp)
# np.where(target_index)
# print(linspace[target_index])

# df = pd.DataFrame({"index": target_index.reshape(-1), "value": grid.reshape(-1)})
# df = pd.DataFrame({"index": target_index.reshape(-1), "value": grid.reshape(-1)})

# print(df.groupby("index").apply(lambda x: x.sum()).drop("index", axis=1).reset_index())


linspace = np.linspace(-0.8, 0.8, 1)
print(linspace)
linspace2 = np.linspace(0, 2.0, 2)
linspace3 = np.linspace(0, 3.0, 2)
linspace3 = np.linspace(0, 4.0, 2)

lins = [linspace, linspace2, linspace3]

lins = [np.linspace(i, i + 1, 1) for i in range(5)]
# lins = lins[1:] + lins[:1]
a = np.meshgrid(*lins)
# print(a)
print(np.stack(a))

a = [[[[0, 1], [2, 3]]], [[[10, 11], [12, 13]]]]
a = np.array(a)
# print(a.reshape(2, -1))
# b = a[2:] + a[:2]
# print(b)
