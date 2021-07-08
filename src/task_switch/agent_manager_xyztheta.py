#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np

from task_switch.agent_manager_xztheta import AgentManagerXTheta
from task_switch.voronoi_xztheta import VoronoiXYZTheta


class AgentManagerXYZTheta(AgentManagerXTheta):
    def Float32MultiArrayCallback(self, msg_data):
        # subscriber to get information density
        # msg_data is information density, which is vectorized.
        info = np.array(msg_data.data).reshape(
            (msg_data.layout.dim[0].size, msg_data.layout.dim[1].size)
        )
        # rospy.loginfo("agent")
        # rospy.loginfo(info)
        self.field.setPhi(info)


if __name__ == "__main__":
    agent = AgentManagerXYZTheta(VoronoiXYZTheta)
    agent.init()
    agent.spin()
