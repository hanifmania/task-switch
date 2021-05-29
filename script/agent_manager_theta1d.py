#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import numpy as np
from std_msgs.msg import Int8MultiArray, MultiArrayLayout, MultiArrayDimension
from task_switch.agent_manager_base import AgentManagerBase
from task_switch.field import Field2d
from task_switch.cover_area import CoverAreaTheta1d, VoronoiTheta1d


class AgentManagerTheta1d(AgentManagerBase):
    def Vel2dCommandCalc(self):
        # calculate command for agent

        pos = self.position[0:2]
        AgentPos = [pos[0], pos[1], 0.0, 0.0, 0.0, 0.0]
        currentEnergy = self.energy

        # extract x,y position from list of x,y,z position
        allPos2d = np.delete(self.allPositions, 2, axis=1)
        # delete THIS agent position
        neighborPosOnly = np.delete(allPos2d, self.agentID - 1, axis=0)

        #### normal persistent coverage ##################################
        # calculate command for agent
        # different from sugimoto san paper at divided 2mass.(probably dividing 2mass is true)
        # u_nom2d = (
        #     -pos
        #     + self.voronoi.getCent()
        #     - self.voronoi.getExpand() / (2 * self.voronoi.getMass())
        # ) * self.controllerGain
        # u_nom = np.array( [ [u_nom2d[0]], [u_nom2d[1]], [0.], [0.], [0.], [0.] ]  )
        dJdp = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        xi = [0.0]

        #### cbf persistent coverage #####################################
        self.voronoi.calcdJdp()
        dJdp_x, dJdp_y = self.voronoi.getdJdp()
        # xi = [-0.1]
        u_0 = 4 / dJdp_x if dJdp_x != 0 else 0
        u_nom = np.array(
            [
                [u_0],
                [0.0],
                [0.0],
                [0.0],
                [0.0],
                [0.0],
            ]
        )
        # u_nom = np.array(
        #     [
        #         [dJdp_x],
        #         [dJdp_y],
        #         [0.0],
        #         [0.0],
        #         [0.0],
        #         [0.0],
        #     ]
        # )
        # dJdp2d = 2*self.voronoi.getMass()*(self.voronoi.getCent()-pos)-self.voronoi.getExpand()
        # dJdp = [dJdp2d[0], dJdp2d[1], 0., 0., 0., 0.]
        # dJdp = [1, 0, 0.0, 0.0, 0.0, 0.0]

        # dJdp = [dJdp_x, dJdp_y, 0, 0, 0, 0]
        # xi = [self.voronoi.getXi()]
        # xi = [self.voronoi.getXi() / self.agentNum]
        # xi = [(self.voronoi.k * (self.voronoi.gamma - 2 * self.J))]

        u, opt_status, task = self.optimizer.optimize(
            u_nom, AgentPos, currentEnergy, dJdp, xi, neighborPosOnly, self.collisionR
        )
        rospy.loginfo(
            "ID,p, dJdp, u : {}, {:.5f},{:.5f},{:.5f}".format(
                self.agentID, pos[0], dJdp_x, u[0][0]
            )
        )
        # self._log.append(
        #     [
        #         self.voronoi.getdJdp(),
        #         # self._voronoi2.getdJdp(),
        #         u[0][0],
        #         self.J - self.voronoi.gamma,
        #     ]
        # )
        # self.J_tilde_log.append(self.J - self.voronoi.gamma)
        return u[0], u[1], opt_status, task
        # return u_nom2d[0], u_nom2d[1], opt_status, task
        # return dJdp_x * 10, dJdp_y, None, None

    def publishRegion(self, region, target=None):
        # publish my sensing region
        # make multiarraydimension
        dim_ = []
        dim_.append(
            MultiArrayDimension(
                label="y",
                size=region.shape[0],
                stride=region.shape[0] * region.shape[1],
            )
        )
        dim_.append(
            MultiArrayDimension(label="x", size=region.shape[1], stride=region.shape[1])
        )
        # make multiarraylayout
        layout_ = MultiArrayLayout(dim=dim_)

        # vectorize region, convert cast, delete size 1 dimension.
        region_vec = np.reshape(region, (1, -1)).astype(np.int8).squeeze()

        # make Int8multiarray. numpy to list convert
        region_for_pub = Int8MultiArray(data=region_vec.tolist(), layout=layout_)
        # publish
        if target is None:
            self.pub_region.publish(region_for_pub)
        else:
            target.publish(region_for_pub)


if __name__ == "__main__":
    agent = AgentManagerTheta1d(Field2d, VoronoiTheta1d)
    agent.spin()
