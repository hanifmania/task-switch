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

from task_switch.central_xtheta import CentralXTheta
from task_switch.field import Field


class CentralXYZTheta(CentralXTheta):
    def __init__(self):
        super(CentralXYZTheta, self).__init__()
        self.compress2drone()
        self._rviz2max1_val = 1 / np.amax(self._drone_field.getPhi())

    def q2p(self, q):
        z = 1
        temp = (z - q[2]) * np.tan(np.pi/2 - q[3])

        return np.stack([q[0] - temp * np.cos(q[4]), q[1] - temp * np.sin(q[4])])

    def compress2drone(self):
        zero_index = self._drone_field.getZeroIndex()
        grid_span = self._drone_field.getGridSpan()

        target_index = [
            np.round(self._projected_field[i] / grid_span[i]) + zero_index[i]
            for i in range(2)
        ]
        # target_index = np.round(self._projected_field / grid_span) + zero_index
        target_index = np.stack(target_index)
        target_index = target_index.astype(np.int64).reshape(2, -1)
        observe_phi = self._observe_field.getPhi().reshape(-1)
        # print("observe_phi", observe_phi)
        # print("target_index", target_index)
        compressed_phi = np.zeros_like(self._drone_field.getPhi())
        shape = compressed_phi.shape
        for x, y, val in zip(target_index[0], target_index[1], observe_phi):
            if 0 <= y < shape[0] and 0 <= x < shape[1]:
                compressed_phi[y, x] += val
        # for index_row, val_row in zip(target_index[0], observe_phi):
        #     for i, val in zip(index_row, val_row):
        #         # print(i, val)
        #         compressed_phi[i] += val
        # print(np.sum(observe_phi), np.sum(compressed_phi))
        compressed_phi = compressed_phi * self._observe_field.getPointDense()
        self._drone_field.setPhi(compressed_phi)

    def updatePhi(self):
        # information reliability update

        currentTime = rospy.Time.now().to_sec()
        dt = currentTime - self.previousInfoUpdateTime
        self.previousInfoUpdateTime = currentTime
        Z_min = 0.0

        Z = self._observe_field.getPhi()
        dists = [
            np.sqrt(
                (pos[0] - self._projected_field[0]) ** 2
                + (pos[1] - self._projected_field[1]) ** 2
            )
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
        Z = np.where(Z > 1.0, 1.0, Z)

        self._observe_field.setPhi(Z)

    def compress2rviz(self):
        temp = self._drone_field.getPhi() * self._rviz2max1_val
        self._rviz_field.setPhi(temp)

    def publishCompressedPhi(self):
        # publish information density
        # make multiarraydimension
        info = self._drone_field.getPhi()
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
        self._pub_compressed_phi.publish(info_for_pub)

    def publishPhi2Rviz(self):
        # publish information density
        # make multiarraydimension
        info = self._rviz_field.getPhi()
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
        self._pub_phi.publish(info_for_pub)


class CentralXYZThetaCompress(CentralXYZTheta):
    def __init__(self):
        super(CentralXYZThetaCompress, self).__init__()
        self.compress2drone_only_first()
        self._rviz2max1_val = 1 / np.amax(self._drone_field.getPhi())
        self.previousInfoUpdateTime = rospy.Time.now().to_sec()
        # self._projected_field = self._drone_field.getGrid()
        # print(self._observe_field.getGrid())
        print(
            self._observe_field.getPointDense() * np.sum(self._observe_field.getPhi())
        )
        self._old_J = 0
        rospy.loginfo("compress finish")

    def init(self):
        super(CentralXYZThetaCompress, self).init()

    def compress2drone(self):

        pass

    def updatePhi(self):
        # information reliability update

        currentTime = rospy.Time.now().to_sec()
        dt = currentTime - self.previousInfoUpdateTime
        self.previousInfoUpdateTime = currentTime
        self._dt = dt
        Z_min = 0.0
        grid = self._drone_field.getGrid()
        Z = self._drone_field.getPhi()
        dists = [
            np.sqrt((pos[0] - grid[0]) ** 2 + (pos[1] - grid[1]) ** 2)
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
        self._drone_field.setPhi(Z)

        J_sum = np.sum(Z)
        dJdt = (J_sum - self._old_J) / dt
        self._log.append([currentTime - self._start_time, J_sum, dJdt])
        self._old_J = J_sum

    def compress2drone_only_first(self):
        zero_index = self._drone_field.getZeroIndex()
        grid_span = self._drone_field.getGridSpan()

        target_index = [
            np.round(self._projected_field[i] / grid_span[i]) + zero_index[i]
            for i in range(2)
        ]
        # target_index = np.round(self._projected_field / grid_span) + zero_index
        target_index = np.stack(target_index)
        target_index = target_index.astype(np.int64).reshape(2, -1)
        observe_phi = self._observe_field.getPhi().reshape(-1)
        # print("observe_phi", observe_phi)
        # print("target_index", target_index)
        compressed_phi = np.zeros_like(self._drone_field.getPhi())
        shape = compressed_phi.shape
        for x, y, val in zip(target_index[0], target_index[1], observe_phi):
            if 0 <= y < shape[0] and 0 <= x < shape[1]:
                compressed_phi[y, x] += val
        # for index_row, val_row in zip(target_index[0], observe_phi):
        #     for i, val in zip(index_row, val_row):
        #         # print(i, val)
        #         compressed_phi[i] += val
        # print(np.sum(observe_phi), np.sum(compressed_phi))
        compressed_phi = compressed_phi * self._observe_field.getPointDense()
        self._drone_field.setPhi(compressed_phi)

    def publishJ(self):
        J = np.sum(self._drone_field.getPhi())
        self.pub_J.publish(Float32(data=J))

    def savelog(self, other_str="_central"):
        rospack = rospkg.RosPack()
        # get the file path for rospy_tutorials
        pkg_path = rospack.get_path("task_switch")

        data_dir = pkg_path + "/data/"
        now = datetime.datetime.now()
        filename = data_dir + now.strftime("%Y%m%d_%H%M%S") + other_str
        df = pd.DataFrame(
            data=self._log,
            columns=[
                "t",
                "J",
                "dJdt",
            ],
        )
        df.to_csv(filename + ".csv", index=False)
        rospy.loginfo("save " + filename)


class CentralXYZThetaShowTheta(CentralXYZThetaCompress):
    def __init__(self):
        super(CentralXYZThetaShowTheta, self).__init__()
        param = rospy.get_param("/theta_field")
        self._show_theta_field = Field(param)
        self._pub_theta_phi = rospy.Publisher(
            "/theta_phi", Float32MultiArray, queue_size=1
        )

        grid = self._show_theta_field.getGrid()
        q = [0, 0, 0] + grid
        p = self.q2p(q)
        rospy.loginfo(p)

    def updatePhi(self):
        super(CentralXYZThetaShowTheta, self).updatePhi()
        # information reliability update
        dt = self._dt
        Z_min = 0.0
        grid = self._show_theta_field.getGrid()
        q = [0, 0, 0] + grid
        p = self.q2p(q)

        Z = self._show_theta_field.getPhi()
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
        self._show_theta_field.setPhi(Z)

    def publishPhi2Rviz(self):
        super(CentralXYZThetaShowTheta, self).publishPhi2Rviz()
        # publish information density
        # make multiarraydimension
        info = self._show_theta_field.getPhi()
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
    central = CentralXYZThetaCompress()
    central.init()
    rospy.spin()
