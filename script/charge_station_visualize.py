#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped

class BebopMarker:
    def __init__(self):
        rospy.init_node("charge_station_visualize")
        chargePos = [rospy.get_param("charge_station/x",0.),rospy.get_param("charge_station/y",0.)]
        chargeR = rospy.get_param("charge_station/r",0.)
        agentID = rospy.get_param("agentID",1)
        # self.pubtopic = str(self.basename)+"/marker"
        self.pubtopic = "/visualization/chargeStation"


        self.red = rospy.get_param("~red", default=0)
        self.green = rospy.get_param("~green", default=1)
        self.blue = rospy.get_param("~blue", default=0)
        self.alpha = rospy.get_param("~alpha", default=.5)
        
        self.marker = Marker()
        self.marker.ns = "charge_station"+str(agentID)
        self.marker.id = 0
        self.marker.pose.position.x = chargePos[0];
        self.marker.pose.position.y = chargePos[1];
        self.marker.pose.position.z = 0;
        self.marker.pose.orientation.x = 0;
        self.marker.pose.orientation.y = 0;
        self.marker.pose.orientation.z = 0;
        self.marker.pose.orientation.w = 1;
        self.marker.scale.x = chargeR*2
        self.marker.scale.y = chargeR*2
        self.marker.scale.z = .1
        self.marker.color.r = self.red
        self.marker.color.g = self.green
        self.marker.color.b = self.blue
        self.marker.color.a = self.alpha


        self.markerpub = rospy.Publisher(self.pubtopic, Marker, queue_size=10)

        self.clock = rospy.get_param("~clock",1)
        self.rate = rospy.Rate(self.clock)


    def publishChargeStation(self):
        
        self.marker.header.frame_id = "/world"
        self.marker.header.stamp = rospy.Time.now()

        self.marker.action = Marker.ADD


        self.marker.lifetime = rospy.Duration()
        self.marker.type = Marker.CYLINDER
        self.markerpub.publish(self.marker)



    def main(self):
        while not rospy.is_shutdown():
            self.publishChargeStation()

            self.rate.sleep()


if __name__ == '__main__':
   bebopomarker = BebopMarker() 
   bebopomarker.main()
