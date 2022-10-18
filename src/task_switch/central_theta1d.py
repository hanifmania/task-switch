#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import math
from scipy.stats import norm

from std_msgs.msg import Float32MultiArray, MultiArrayLayout, MultiArrayDimension
from geometry_msgs.msg import PoseArray

from task_switch.central_base import CentralBase


class CentralTheta1d(CentralBase):
    def __init__(self, get_dist_method):
        super(CentralTheta1d, self).__init__()
        self.allPositions = np.zeros((self.agentNum, 3))
        rospy.Subscriber("/allPose", PoseArray, self.poseArrayCallback, queue_size=1)
        self._sigma = rospy.get_param("/sigma")
        self._get_dist_method = get_dist_method

    def publishInfo(self, info):
        # publish information density

        # make multiarraydimension
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
        self.pub_info.publish(info_for_pub)

    def joy_callback(self, data):
        button_is_pushed = data.buttons[self.buttons_enable_info_update]
        # button_is_pushed = True
        if button_is_pushed:
            # rospy.loginfo("start")
            self.enable_info_update(True)
            self.previousInfoUpdateTime = rospy.Time.now().to_sec()

    def infoUpdate(self, Z, region):
        # information reliability update

        currentTime = rospy.Time.now().to_sec()
        dt = currentTime - self.previousInfoUpdateTime
        self.previousInfoUpdateTime = currentTime
        Z_min = 0

        grid = self.field.getGrid()
        dists = [self._get_dist_method(pos, grid) for pos in self.allPositions]

        dist2 = np.stack(dists)

        min_dist = dist2.min(axis=0)
        # print(min_dist)
        ##### h = norm
        J = (
            norm.pdf(min_dist, scale=self._sigma)
            * Z
            * math.sqrt(2 * math.pi)
            * self._sigma
        )
        ##### h = 1 or 0
        # J = min_dist < self._sigma

        Z = Z - self.delta_decrease * J * dt
        Z = np.where(Z < Z_min, Z_min, Z)
        # Z = Z + self.delta_increase * (1 - Z) * dt * ~region
        Z = np.where(Z > 1.0, 1.0, Z)

        return Z

    def getDist(self, p_x, q_x, q_theta):
        return p_x - q_x - np.tan(q_theta) * -1  # self._A[1]

    def poseArrayCallback(self, msg):
        # subscriber to get every agent's position
        arraynum = len(msg.poses)
        for i in range(arraynum):
            pos = [
                msg.poses[i].position.x,
                msg.poses[i].position.y,
                msg.poses[i].position.z,
            ]
            self.allPositions[i] = pos
