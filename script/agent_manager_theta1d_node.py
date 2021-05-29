#!/usr/bin/env python
# -*- coding: utf-8 -*-
from task_switch.voronoi import VoronoiTheta1d
from task_switch.agent_manager_theta1d import AgentManagerTheta1d

if __name__ == "__main__":
    agent = AgentManagerTheta1d(VoronoiTheta1d)
    agent.spin()
