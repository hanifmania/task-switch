#!/usr/bin/env python
# -*- coding: utf-8 -*-

from task_switch.central_theta1d import CentralTheta1d
from task_switch.voronoi import VoronoiTheta1d

if __name__ == "__main__":
    central = CentralTheta1d(VoronoiTheta1d.getDist)
    central.spin()
