#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Float32MultiArray, ColorRGBA
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np
import matplotlib.pyplot as plt
from task_switch.field import Field


class PointCloudXTheta(object):
    """
    Visualization class of surf plot in rviz(only colored 2D plot using CubeList).
    It's used for visualization of importance density function in coverage control.

    Subscribe : The values of the function of x and y.(Float32MultiArray)
        The grid point for each function value is assumed to be like this:
        [(-1.0,-1.0), (-0.9,-1.0), ... (1.0,-1.0)
         (-1.0,-0.9), (-0.9,-0.9), ... (1.0,-0.9)
         .                              .
         .                              .
         .                              .
         (-1.0,1.0), (-0.9,1.0), ... (1.0,1.0)]

    Publish : Marker (CubeList) with gradation color along to the value of function.
    """

    def __init__(self, Hz=100):
        rospy.init_node("create_cloud_xyzrgb", anonymous=True)
        field_param = rospy.get_param("/rviz_field")
        self.field = Field(field_param)
        self.max_value = rospy.get_param("~max", default=1)
        self.min_value = rospy.get_param("~min", default=0)

        self.X, self.Y = self.field.getGrid()
        # the height to be aligned for pointcloud2
        self.Z = np.zeros((self.X.size, 1))

        self.points = np.hstack(
            [
                self.X.reshape([-1, 1]),
                self.Y.reshape([-1, 1]),
                self.Z.reshape([-1, 1]),
            ]
        )

        # Data size of message
        self.function_size_x = 0
        self.function_size_y = 0

        # ROS set up
        self.pub_pointcloud = rospy.Publisher("point_cloud2", PointCloud2, queue_size=1)
        # self.marker_pub = rospy.Publisher("field_visualization", Marker, queue_size=1)
        self.surf_value_sub = rospy.Subscriber(
            "surf_value", Float32MultiArray, self.Float32MultiArrayCallback
        )

        # param initialize
        self.clock = rospy.get_param("~clock", Hz)
        self.rate = rospy.Rate(self.clock)

    def Float32MultiArrayCallback(self, msg_data):
        # msg_data is information density, which is vectorized.
        info = np.array(msg_data.data).reshape(
            (msg_data.layout.dim[0].size, msg_data.layout.dim[1].size)
        )
        self.field.setPhi(info)

    def xyzrgb_array_to_pointcloud2(
        self, points, colors, stamp=None, frame_id=None, seq=None
    ):
        """
        Create a sensor_msgs.PointCloud2 from an array
        of points.
        """
        msg = PointCloud2()
        assert points.shape == colors.shape

        buf = []

        if stamp:
            msg.header.stamp = stamp
        if frame_id:
            msg.header.frame_id = frame_id
        if seq:
            msg.header.seq = seq
        if len(points.shape) == 3:
            msg.height = points.shape[1]
            msg.width = points.shape[0]
        else:
            N = len(points)
            xyzrgb = np.array(np.hstack([points, colors]), dtype=np.float32)
            msg.height = 1
            msg.width = N

        msg.fields = [
            PointField("x", 0, PointField.FLOAT32, 1),
            PointField("y", 4, PointField.FLOAT32, 1),
            PointField("z", 8, PointField.FLOAT32, 1),
            PointField("r", 12, PointField.FLOAT32, 1),
            PointField("g", 16, PointField.FLOAT32, 1),
            PointField("b", 20, PointField.FLOAT32, 1),
        ]
        msg.is_bigendian = False
        msg.point_step = 24
        msg.row_step = msg.point_step * N
        msg.is_dense = True
        msg.data = xyzrgb.tostring()

        return msg

    def createPointCloudMsg(self):

        phi = self.field.getPhi().reshape([-1, 1])
        colorrgb_ = plt.get_cmap("jet")(phi)
        colorrgb = colorrgb_.squeeze()

        self.points = np.hstack(
            [self.X.reshape([-1, 1]), self.Y.reshape([-1, 1]), self.Z.reshape([-1, 1])]
        )
        # colorrgb = np.hstack([r,g,b])
        msg = self.xyzrgb_array_to_pointcloud2(
            self.points, colorrgb[:, 0:3], frame_id="world"
        )
        return msg

    def publishPointCloud(self):
        pointcloudmsg = self.createPointCloudMsg()
        self.pub_pointcloud.publish(pointcloudmsg)

    def spin(self):
        rospy.wait_for_message("surf_value", Float32MultiArray)
        while not rospy.is_shutdown():
            self.publishPointCloud()
            self.rate.sleep()


if __name__ == "__main__":

    surf = PointCloudXTheta(20)
    surf.spin()
