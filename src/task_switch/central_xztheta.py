#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np

from std_msgs.msg import Float32MultiArray, MultiArrayLayout, MultiArrayDimension
from task_switch.central_xtheta import CentralXTheta


class CentralXZTheta(CentralXTheta):
    def __init__(self):
        super(CentralXZTheta, self).__init__()

    def q2p(self, q):
        z = 1
        return q[0] - (z - q[2]) * np.tan(q[1])

    def publishPhi2Rviz(self):
        # publish information density
        info = self._rviz_field.getPhi()
        # make multiarraydimension
        dim_ = []
        dim_.append(
            MultiArrayDimension(
                label="z",
                size=info.shape[0],
                stride=info.shape[0] * info.shape[1] * info.shape[2],
            )
        )
        dim_.append(
            MultiArrayDimension(
                label="y", size=info.shape[1], stride=info.shape[1] * info.shape[2]
            )
        )
        dim_.append(
            MultiArrayDimension(label="x", size=info.shape[2], stride=info.shape[2])
        )
        layout_ = MultiArrayLayout(dim=dim_)
        info_vec = np.reshape(info, (1, -1)).astype(np.float32).squeeze()
        info_for_pub = Float32MultiArray(data=info_vec.tolist(), layout=layout_)
        self._pub_phi.publish(info_for_pub)


if __name__ == "__main__":
    central = CentralXZTheta()
    central.init()
    rospy.spin()
