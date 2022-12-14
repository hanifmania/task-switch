#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped

class BebopMarker:
    def __init__(self):
        rospy.init_node("pose2marker")
        self.basename = rospy.get_param("markername")
        # self.pubtopic = str(self.basename)+"/marker"
        self.pubtopic = "/visualization/bebop2"
        # self.pubtopic = "/visualization/" + str(self.basename)
        self.markerpub = rospy.Publisher(self.pubtopic, Marker, queue_size=10)
        self.posestampedsub = rospy.Subscriber("posestamped", PoseStamped, self.poseCallback)

        self.red = rospy.get_param("red", default=.5)
        self.green = rospy.get_param("green", default=.5)
        self.blue = rospy.get_param("blue", default=.5)
        self.alpha = rospy.get_param("alpha", default=1)
        
        self.marker = Marker()
        self.marker.ns = str(self.basename)
        self.marker.id = 0
        self.marker.color.r = self.red
        self.marker.color.g = self.green
        self.marker.color.b = self.blue
        self.marker.color.a = self.alpha

    def setBebopMarker(self, msg):
        
        self.marker.header.frame_id = "/world"
        self.marker.header.stamp = rospy.Time.now()

        self.marker.action = Marker.ADD

        self.marker.pose = msg.pose


        self.marker.scale.x = 1.0
        self.marker.scale.y = 1.0
        self.marker.scale.z = 1.0

        self.marker.lifetime = rospy.Duration()
        self.marker.type = Marker.MESH_RESOURCE
        self.marker.mesh_use_embedded_materials = True
        # self.marker.mesh_use_embedded_materials = False
        self.marker.mesh_resource = "package://rotors_description/meshes/bebop2.dae"


    def poseCallback(self, msg):
        self.setBebopMarker(msg)
        self.markerpub.publish(self.marker)

    def main(self):
        while not rospy.is_shutdown():
            # self.rate.sleep()
            rospy.spin()

if __name__ == '__main__':
   bebopomarker = BebopMarker() 
   bebopomarker.main()
