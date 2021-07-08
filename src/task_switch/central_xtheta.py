#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import math
from scipy.stats import norm
from abc import ABCMeta, abstractmethod

import dynamic_reconfigure.client
from std_msgs.msg import (
    Float32MultiArray,
    MultiArrayLayout,
    MultiArrayDimension,
    Float32,
)
from geometry_msgs.msg import PoseArray
from sensor_msgs.msg import Joy
from task_switch.field import Field


class CentralBase(object):
    __metaclass__ = ABCMeta

    def __init__(self):
        rospy.init_node("central", anonymous=True)

        # Number of Agents
        self.agentNum = rospy.get_param("/agentNum", 1)
        self.allPositions = np.zeros((self.agentNum, 3))
        self._sigma = rospy.get_param("/sigma")
        self._observe_field = Field(rospy.get_param("/observe_field"))
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
        q = self._observe_field.getGrid()
        self._projected_field = self.q2p(q)

        # print("phi shape", q)
        # print("_projected_field shape", self._projected_field.shape)
        self._start = False
        rospy.Subscriber("/joy", Joy, self.joy_callback, queue_size=1)

        self.pub_J = rospy.Publisher("/J", Float32, queue_size=1)

    def init(self):
        # second init after __init__
        rospy.loginfo("Init central")
        config = self.pcc_dycon_client.get_configuration()
        self.pccConfigCallback(config)
        rospy.Subscriber("/allPose", PoseArray, self.mainCallback, queue_size=1)

    def mainCallback(self, msg):
        self.msg2pos(msg)
        if self._start:
            self.updatePhi()
            self.compress2drone()
            self.compress2rviz()
            self.publishCompressedPhi()
            self.publishPhi2Rviz()
            self.publishJ()

    def publishJ(self):
        J = np.sum(self._observe_field.getPhi()) * self._observe_field.getPointDense()
        self.pub_J.publish(Float32(data=J))

    def msg2pos(self, msg):
        # subscriber to get every agent's position
        arraynum = len(msg.poses)
        for i in range(arraynum):
            pos = [
                msg.poses[i].position.x,
                msg.poses[i].position.y,
                msg.poses[i].position.z,
            ]
            self.allPositions[i] = pos

    def joy_callback(self, data):
        # button_is_pushed = data.buttons[2]
        button_is_pushed = True
        if button_is_pushed:
            rospy.loginfo("start")
            self._start = True
            self.previousInfoUpdateTime = rospy.Time.now().to_sec()

    def updatePhi(self):
        # information reliability update

        currentTime = rospy.Time.now().to_sec()
        dt = currentTime - self.previousInfoUpdateTime
        self.previousInfoUpdateTime = currentTime
        Z_min = 0.0

        Z = self._observe_field.getPhi()
        dists = [np.abs(pos[0] - self._projected_field) for pos in self.allPositions]
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

    def pccConfigCallback(self, config):
        self._delta_decrease = config.delta_decrease

    @abstractmethod
    def q2p(self, q):
        pass

    @abstractmethod
    def compress2drone(self):
        pass

    @abstractmethod
    def compress2rviz(self):
        pass

    @abstractmethod
    def publishCompressedPhi(self):
        pass

    @abstractmethod
    def publishPhi2Rviz(self):
        pass


class CentralXTheta(CentralBase):
    def __init__(self):
        super(CentralXTheta, self).__init__()
        self._pub_phi = rospy.Publisher("/info", Float32MultiArray, queue_size=1)

    def q2p(self, q):
        z = 1
        return q[0] - (z) * np.tan(q[1])

    def compress2rviz(self):
        self._rviz_field.setPhi(self._observe_field.getPhi())

    def compress2drone(self):
        zero_index = self._drone_field.getZeroIndex()
        grid_span = self._drone_field.getGridSpan()
        target_index = np.round(self._projected_field / grid_span) + zero_index
        target_index = target_index.astype(np.int64).reshape(-1)
        observe_phi = self._observe_field.getPhi().reshape(-1)
        # print("observe_phi", observe_phi)
        # print("target_index", target_index)
        compressed_phi = np.zeros_like(self._drone_field.getPhi())

        for i, val in zip(target_index, observe_phi):
            compressed_phi[i] += val
        # for index_row, val_row in zip(target_index[0], observe_phi):
        #     for i, val in zip(index_row, val_row):
        #         # print(i, val)
        #         compressed_phi[i] += val
        # print(np.sum(observe_phi), np.sum(compressed_phi))
        compressed_phi = compressed_phi * self._observe_field.getPointDense()
        self._drone_field.setPhi(compressed_phi)

    def publishCompressedPhi(self):
        # publish information density
        # make multiarraydimension
        info = self._drone_field.getPhi()
        dim_ = []
        dim_.append(
            MultiArrayDimension(label="x", size=info.shape[0], stride=info.shape[0])
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
        self._pub_phi.publish(info_for_pub)


if __name__ == "__main__":
    central = CentralXTheta()
    central.init()
    rospy.spin()
