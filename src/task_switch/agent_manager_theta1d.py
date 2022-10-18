#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import numpy as np
import os
import pandas as pd
import datetime
from std_msgs.msg import (
    Int8MultiArray,
    MultiArrayLayout,
    MultiArrayDimension,
    Float32MultiArray,
)
from task_switch.agent_manager_base import AgentManagerBase

from task_switch.voronoi import VoronoiTheta1d

from task_switch.field import Field


class AgentManagerTheta1d(AgentManagerBase):
    def __init__(self, VoronoiClass):
        super(AgentManagerTheta1d, self).__init__(VoronoiClass)
        rospy.Subscriber(
            "/info", Float32MultiArray, self.Float32MultiArrayCallback, queue_size=1
        )
        sigma = rospy.get_param("/sigma")
        self.voronoi.setSigma(sigma)
        self._u_old = [[0], [0]]
        self._H_old = 0
        self._p_old = [0, 0]
        self._dHdt_old = 0

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
        dJdp = [dJdp_x, dJdp_y, 0, 0, 0, 0]
        xi = [self.voronoi.getXi()]

        # xi = [-0.1]

        # if 0 < dJdp_x < 0.000001:
        #     print("dJdp_x 0")
        #     dJdp_x = 0.000001
        # elif -0.000001 < dJdp_x < 0:
        #     print("dJdp_x 0")
        #     dJdp_x = -0.000001
        # u_0x = xi[0] / dJdp_x if dJdp_x != 0 else 0
        # u_0y = gamma / dJdp_y if dJdp_y != 0 else 0
        # u_0x, u_0y = 0, 0
        self._u_base = 0.5
        u_nom_x = self._u_old[0][0]
        # u_nom_x = np.tanh(dJdp_x) * self._u_base  # self._u_old[0][0]
        u_nom_x = 0.0  # np.sign(dJdp_x) * self._u_base
        u_nom = np.array(
            [
                [u_nom_x],
                [0.0],
                [0.0],
                [0.0],
                [0.0],
                [0.0],
            ]
        )
        # u_nom = np.array(
        #     [
        #         [self._u_old[0][0]],
        #         [self._u_old[1][0]],
        #         [0.0],
        #         [0.0],
        #         [0.0],
        #         [0.0],
        #     ]
        # )
        # dJdp2d = 2*self.voronoi.getMass()*(self.voronoi.getCent()-pos)-self.voronoi.getExpand()
        # dJdp = [dJdp2d[0], dJdp2d[1], 0., 0., 0., 0.]
        # dJdp = [1, 0, 0.0, 0.0, 0.0, 0.0]

        # xi = [self.voronoi.getXi() / self.agentNum]
        # xi = [(self.voronoi.k * (self.voronoi.gamma - 2 * self.J))]

        u, opt_status, task = self.optimizer.optimize(
            u_nom, AgentPos, currentEnergy, dJdp, xi, neighborPosOnly, self.collisionR
        )
        rospy.loginfo(
            "ID,p, dJdp, u : {}, {:.4f},{:.4f},{:.4f}, {:.4f}".format(
                self.agentID, pos[0], dJdp_x, u[0][0], self.clock
            )
        )
        H = self.voronoi.getH()  # calc H by drone

        dHdt = (H - self._H_old) * self.clock
        d2Hdt2 = (dHdt - self._dHdt_old) * self.clock
        self._log.append(
            [
                H,
                u[0][0],
                dJdp_x,
                self.optimizer.delta[0][0],
                dHdt,
                -self.voronoi.getGamma(),
                xi[0],
                u_nom_x,
                self.voronoi.getdHdt()[0],
                d2Hdt2,
                -dJdp_x * u[0][0] + self.voronoi._temp2,
                (pos[0] - self._p_old[0]) * self.clock,
            ]
        )
        # self.J_tilde_log.append(self.J - self.voronoi.gamma)
        self._p_old = pos
        self._u_old = u
        self._H_old = H
        self._dHdt_old = dHdt
        self.voronoi.setU(u[0])
        # self.voronoi.myUpdatePhi()
        return u[0], u[1], opt_status, task
        # return u_nom2d[0], u_nom2d[1], opt_status, task
        # return dJdp_x * 10, dJdp_y, None, None

    def savelog(self, other_str):
        CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
        data_dir = CURRENT_DIR + "/../../data/"
        now = datetime.datetime.now()
        filename = data_dir + now.strftime("%Y%m%d_%H%M%S") + other_str
        # df = pd.DataFrame(
        #     data={
        #         "u": self.u_log,
        #         "J~": self.J_tilde_log,
        #     },
        #     columns=["u", "J~"],
        # )
        df = pd.DataFrame(
            data=self._log,
            columns=[
                "H",
                "u",
                "dJdp",
                "slack",
                "dHdt",
                "gamma",
                "xi",
                "u_nom_x",
                "dHdt_predict",
                "d2Hdt2",
                "d2Hdt2_predict",
                "dpdt",
            ],
        )
        df.to_csv(filename + ".csv", index=True)
        rospy.loginfo("save " + filename)

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
    agent = AgentManagerTheta1d(VoronoiTheta1d)
    agent.spin()
