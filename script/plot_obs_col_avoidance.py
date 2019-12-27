#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
from task_switch.msg import PlotData
from geometry_msgs.msg import PoseArray
from visualization_msgs.msg import Marker
import tf
import numpy as np


class PlotObs_col:
    def __init__(self):
        self.listner = tf.TransformListener()
        self.plot_data_sub = rospy.Subscriber("/visualize/plot_data", PlotData, self.callbackPlotData)

        self.PlotData = PlotData()
        self.is_start = 0

        # Which robot we use
        self.bebop = rospy.get_param("~bebop", default=1)
        self.crazyflie = rospy.get_param("~crazyflie", default=0)

        # Get lookup tf name
        self.agenttf = rospy.get_param("~agenttf", default="/visualize/agent")

        # For saving agent positions
        self.positions = {}

        self.rate = rospy.Rate(10)
        self.markerpub = rospy.Publisher("marker", Marker, queue_size=10)

    def setCubeMarker(self,i):
        pose = self.PlotData.obstacle_positions.poses[i]
        xwidth = self.PlotData.obstacle_xwidth[i]
        ywidth = self.PlotData.obstacle_ywidth[i]
        red = green = blue = alpha = 0

        marker = Marker()

        marker.header.frame_id = "/world"
        marker.header.stamp = rospy.Time.now()

        marker.ns = "obs_pos"+str(i)
        marker.id = i

        marker.action = Marker.ADD

        marker.pose = pose

        marker.color.r = red
        marker.color.g = green
        marker.color.b = blue
        marker.color.a = alpha


        marker.scale.x = 0.4
        marker.scale.y = 0.4
        marker.scale.z = 0.4

        marker.lifetime = rospy.Duration()
        marker.type = Marker.MESH_RESOURCE
        marker.mesh_use_embedded_materials = True
        marker.mesh_resource = "package://task_switch/meshes/StopSign/stopsign.dae"
        # # 正確な四角じゃないので0.8がけくらいにしておく
        # marker.scale.x = xwidth*2*0.8
        # marker.scale.y = ywidth*2*0.8
        # marker.scale.z = 2
        #
        # marker.lifetime = rospy.Duration()
        # marker.type = Marker.CUBE

        return marker


    def callbackPlotData(self, data):
        self.PlotData = data
        self.is_start = 1


    def Plot(self):
        rospy.loginfo("Plotting.....")


    def start(self):
        while not rospy.is_shutdown():
            if self.is_start:
                for i in self.PlotData.robot_num:
                    i_str = str(i)

                    try:
                        (position, orientation) = self.listner.lookupTransform(
                            "/world",
                             self.agenttf + i_str,
                              rospy.Time(0))

                        # Save agent position
                        self.positions[i] = position
                    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                        continue

                self.Plot()
                for i in range(len(self.PlotData.obstacle_xwidth)):
                    marker_ = self.setCubeMarker(i)
                    self.markerpub.publish(marker_)
            self.rate.sleep()

if __name__ == "__main__":
    rospy.init_node('plot_node')
    pp = PlotObs_col()
    pp.start()
