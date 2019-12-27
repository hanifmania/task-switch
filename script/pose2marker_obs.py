#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped

class ObsMarker:
    def __init__(self):
        rospy.init_node("pose2marker")
        self.rate = rospy.Rate(10)
        self.basename = rospy.get_param("~markername")
        # self.pubtopic = str(self.basename)+"/marker"
        self.pubtopic = "/visualization/obs"
        self.markerpub = rospy.Publisher(self.pubtopic, Marker, queue_size=10)
        # self.posestampedsub = rospy.Subscriber("posestamped", PoseStamped, self.poseCallback)

        self.red = rospy.get_param("~red", default=0)
        self.green = rospy.get_param("~green", default=0)
        self.blue = rospy.get_param("~blue", default=0)
        self.alpha = rospy.get_param("~alpha", default=0)
        
        self.marker = Marker()
        self.marker.ns = str(self.basename)
        self.marker.id = 0
        self.marker.color.r = self.red
        self.marker.color.g = self.green
        self.marker.color.b = self.blue
        self.marker.color.a = self.alpha


        rospy.sleep(1.0)

    def main(self):
        while not rospy.is_shutdown():

            self.marker.header.frame_id = "/world"
            self.marker.header.stamp = rospy.Time.now()

            self.marker.action = Marker.ADD

            self.marker.pose.position.x = 0.09
            self.marker.pose.position.y = 0.0
            self.marker.pose.position.z = 0.4
            self.marker.pose.orientation.x = 0.0
            self.marker.pose.orientation.y = 1.0
            self.marker.pose.orientation.z = 0.0
            self.marker.pose.orientation.w = 0.04


            self.marker.scale.x = 0.5
            self.marker.scale.y = 0.5
            self.marker.scale.z = 0.5

            self.marker.lifetime = rospy.Duration()
            self.marker.type = Marker.MESH_RESOURCE
            self.marker.mesh_use_embedded_materials = True
            self.marker.mesh_resource = "package://task_switch/meshes/StopSign/stopsign.dae"
            self.markerpub.publish(self.marker)

            self.rate.sleep()
            rospy.spin()
if __name__ == '__main__':
   obsmarker = ObsMarker() 
   obsmarker.main()
