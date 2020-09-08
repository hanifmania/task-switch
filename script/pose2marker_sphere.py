#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped
import dynamic_reconfigure.client

class BebopMarker:
    def __init__(self):
        rospy.init_node("pose2marker")
        agentID = rospy.get_param("agentID",1)
        self.basename = "agent" + str(agentID)
        # self.pubtopic = str(self.basename)+"/marker"
        self.pubtopic = "/visualization/agentSphere"

        self.pcc_dycon_client = dynamic_reconfigure.client.Client("/pcc_parameter", timeout=2, config_callback=self.pcc_config_callback)
        self.collisionR = .5 # this will be overwritten by dycon

        self.red = rospy.get_param("red", default=1)
        self.green = rospy.get_param("green", default=0)
        self.blue = rospy.get_param("blue", default=0)
        self.alpha = rospy.get_param("alpha", default=.5)
        
        self.marker = Marker()
        self.marker.ns = str(self.basename)
        self.marker.id = 0
        self.marker.color.r = self.red
        self.marker.color.g = self.green
        self.marker.color.b = self.blue
        self.marker.color.a = self.alpha


        self.markerpub = rospy.Publisher(self.pubtopic, Marker, queue_size=10)
        self.posestampedsub = rospy.Subscriber("posestamped", PoseStamped, self.poseCallback)

        self.clock = rospy.get_param("~clock",100)
        self.rate = rospy.Rate(self.clock)

    def pcc_update_config_params(self, config):
        self.collisionR = config.collisionR



    def pcc_set_config_params(self):
        config = self.pcc_dycon_client.get_configuration()
        self.pcc_update_config_params(config)

    def pcc_config_callback(self,config):
        self.pcc_update_config_params(config)

    def setBebopMarker(self, msg):
        
        self.marker.header.frame_id = "/world"
        self.marker.header.stamp = rospy.Time.now()

        self.marker.action = Marker.ADD

        self.marker.pose = msg.pose


        self.marker.scale.x = self.collisionR*2
        self.marker.scale.y = self.collisionR*2
        self.marker.scale.z = self.collisionR*2

        self.marker.lifetime = rospy.Duration()
        self.marker.type = Marker.SPHERE


    def poseCallback(self, msg):
        self.setBebopMarker(msg)
        self.markerpub.publish(self.marker)

    def main(self):
        self.pcc_set_config_params()
        while not rospy.is_shutdown():
            rospy.spin()
            self.rate.sleep()


if __name__ == '__main__':
   bebopomarker = BebopMarker() 
   bebopomarker.main()
