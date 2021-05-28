#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray, MultiArrayLayout, MultiArrayDimension
from task_switch.central_base import CentralBase
from task_switch.field import Field2d


class CentralTheta1d(CentralBase):
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

        # delta_decrease = 0.01
        # delta_increase = 0.0001
        currentTime = rospy.Time.now().to_sec()
        dt = currentTime - self.previousInfoUpdateTime
        self.previousInfoUpdateTime = currentTime
        Z_min = 0.0000001
        Z = Z - self.delta_decrease * Z * dt * region
        # Z = Z * ~region
        Z = np.where(Z < Z_min, Z_min, Z)
        # Z = Z + self.delta_increase * (1 - Z) * dt * ~region
        Z = np.where(Z > 1.0, 1.0, Z)

        return Z


if __name__ == "__main__":
    central = CentralTheta1d(Field2d)
    central.spin()
