#!/usr/bin/env python
# -*- coding: utf-8 -*-


import numpy as np
import math
import pandas as pd
import datetime
from scipy.stats import norm

import rospy
import rospkg
from std_msgs.msg import (
    Float32MultiArray,
    MultiArrayLayout,
    MultiArrayDimension,
    Float32,
)
import dynamic_reconfigure.client

from sensor_msgs.msg import Joy
from task_switch.central_xtheta import CentralBase
from task_switch.field import Field

class CentralOriginShow(CentralBase):
    def __init__(self):
        rospy.init_node("central", anonymous=True)

        self._target_xyz = rospy.get_param("~target")
        # Number of Agents
        self.agentNum = rospy.get_param("/agentNum", 1)
        self.allPositions = np.zeros((self.agentNum, 3))
        self._sigma = rospy.get_param("/sigma")
        self._observe_field = Field(rospy.get_param("/theta_field"))
        self._drone_field = Field(rospy.get_param("/drone_field"))
        self._rviz_field = Field(rospy.get_param("/rviz_field"))
        self._pub_phi = rospy.Publisher("/phi", Float32MultiArray, queue_size=1)
        self._pub_compressed_phi = rospy.Publisher(
            "/compressed_phi", Float32MultiArray, queue_size=1
        )
        # dynamic_reconfigure
        self.pcc_dycon_client = dynamic_reconfigure.client.Client(
            "/pcc_parameter", timeout=2, config_callback=self.pccConfigCallback
        )

        # print("phi shape", q)
        # print("_projected_field shape", self._projected_field.shape)
        self._start = False
        rospy.Subscriber("/joy", Joy, self.joy_callback, queue_size=1)

        self.pub_J = rospy.Publisher("/J", Float32, queue_size=1)
        self._log = []
        rospy.on_shutdown(self.savelog)
        q = self._observe_field.getGrid()
        self._projected_field = self.q2p(q)
        self._pub_theta_phi = rospy.Publisher(
            "theta_phi", Float32MultiArray, queue_size=1
        )

    def publishJ(self):
        pass
    
    def q2p(self, q):
        z = 1
        temp = (z - self._target_xyz["z"]) * np.tan(math.pi/2 - q[0])

        return np.stack([self._target_xyz["x"] - temp * np.cos(q[1]), self._target_xyz["y"] - temp * np.sin(q[1])])

    def updatePhi(self):
        # information reliability update
        currentTime = rospy.Time.now().to_sec()
        dt = currentTime - self.previousInfoUpdateTime
        self.previousInfoUpdateTime = currentTime
        Z_min = 0.0
        p = self._projected_field

        Z = self._observe_field.getPhi()


        dists = [
            np.sqrt((pos[0] - p[0]) ** 2 + (pos[1] - p[1]) ** 2)
            for pos in self.allPositions
        ]
        # dists = [np.abs(pos[0] - self._projected_field) for pos in self.allPositions]
        dist2 = np.stack(dists)
        min_dist = dist2.min(axis=0)
        J = (
            norm.pdf(min_dist, scale=self._sigma)
            * Z
            * math.sqrt(2 * math.pi)
            * self._sigma
        )
        Z = Z - self._delta_decrease * J * dt
        Z = np.where(Z < Z_min, Z_min, Z)
        # Z = Z + self.delta_increase *phiUpdate (1 - Z) * dt * ~region
        # Z = np.where(Z > 1.0, 1.0, Z)
        self._observe_field.setPhi(Z)

    def publishPhi2Rviz(self):
        # publish information density
        # make multiarraydimension
        info = self._observe_field.getPhi()
        dim_ = []
        dim_.append(
            MultiArrayDimension(
                label="y", size=info.shape[0], stride=info.shape[0] * info.shape[1]
            )
        )
        dim_.append(
            MultiArrayDimension(label="x", size=info.shape[1], stride=info.shape[1])
        )
        # make multiarraylayout
        layout_ = MultiArrayLayout(dim=dim_)

        # vectorize info(=phi), convert cast, delete size 1 dimension.
        info_vec = np.reshape(info, (1, -1)).astype(np.float32).squeeze()

        # make Float32multiarray. numpy to list convert
        info_for_pub = Float32MultiArray(data=info_vec.tolist(), layout=layout_)

        # publish
        self._pub_theta_phi.publish(info_for_pub)


if __name__ == "__main__":
    central = CentralOriginShow()
    central.init()
    rospy.spin()
