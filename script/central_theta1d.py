#!/usr/bin/env python
# -*- coding: utf-8 -*-

from matplotlib.pyplot import axis
import rospy
import numpy as np
import math
from scipy.stats import norm

from std_msgs.msg import Float32MultiArray, MultiArrayLayout, MultiArrayDimension
from task_switch.central_base import CentralBase
from task_switch.field import Field2d
from geometry_msgs.msg import PoseArray


class CentralTheta1d(CentralBase):
    def __init__(self, FieldClass):
        super(CentralTheta1d, self).__init__(FieldClass)
        self.allPositions = np.zeros((self.agentNum, 3))
        rospy.Subscriber("/allPose", PoseArray, self.poseArrayCallback, queue_size=1)

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

    # def infoUpdate(self, Z, region):
    #     # information reliability update

    #     # delta_decrease = 0.01
    #     # delta_increase = 0.0001
    #     currentTime = rospy.Time.now().to_sec()
    #     dt = currentTime - self.previousInfoUpdateTime
    #     self.previousInfoUpdateTime = currentTime
    #     Z_min = 0.0000001
    #     Z = Z - self.delta_decrease * Z * dt * region
    #     # Z = Z * ~region
    #     Z = np.where(Z < Z_min, Z_min, Z)
    #     # Z = Z + self.delta_increase * (1 - Z) * dt * ~region
    #     Z = np.where(Z > 1.0, 1.0, Z)

    #     return Z
    def infoUpdate(self, Z, region):
        # information reliability update

        # delta_decrease = 0.01
        # delta_increase = 0.0001
        currentTime = rospy.Time.now().to_sec()
        dt = currentTime - self.previousInfoUpdateTime
        self.previousInfoUpdateTime = currentTime
        Z_min = 0.0000001

        x_grid, y_grid = self.field.getGrid()
        dists = [
            np.abs(self.getDist(pos[0], x_grid, y_grid)) for pos in self.allPositions
        ]

        dist2 = np.stack(dists)

        min_dist = dist2.min(axis=0)
        # print(min_dist)
        scale = 0.1
        J = norm.pdf(min_dist, scale=scale) * Z  # * (math.sqrt(2 * math.pi) * scale)
        # print(J)
        # print(self.allPositions[0][0], self.allPositions[1][0])
        Z = Z - self.delta_decrease * J * dt
        # Z = Z * ~region
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


if __name__ == "__main__":
    central = CentralTheta1d(Field2d)
    central.spin()
