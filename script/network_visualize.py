#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import tf
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

from tf.transformations import *

class NetworkVisualizer(object):
    
    def __init__(self, tf_list):

        rospy.init_node("network_visualize")
        self.pub = rospy.Publisher("/visualization_marker", Marker, queue_size = 10)
        self.rate = rospy.Rate(100)
        # target_frame = rospy.get_param("~target_frame")

        self.tf_listner = tf.TransformListener()
        

        self.tf_list = tf_list

        self.time_sequence = 0.0
        self.time_sequence_max =80

        rospy.sleep(1.0) # tf_listnerがtfバッファを貯めるまでwait


    def tf_to_point(self, tf_listner, parent, child):
        try:
            (trans,rot) = tf_listner.lookupTransform(child, parent, rospy.Time(0))
            return trans
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn(e)


    def network_publish(self, tf_list, color_rgba = (0.9,0.9,0.1,0.5)):
        marker_data = Marker()
        marker_data.header.frame_id = "world"
        marker_data.header.stamp = rospy.Time.now()

        marker_data.ns = rospy.get_namespace() + rospy.get_name() + "network_line"
        marker_data.id = 0
        marker_data.action = Marker.ADD
        marker_data.lifetime = rospy.Duration(0.5)
        marker_data.type = 5 #line_list

        (marker_data.color.r, marker_data.color.g, marker_data.color.b, marker_data.color.a) = color_rgba

        scale = 0.01
        marker_data.scale.x = scale

        for tf in tf_list:
            p = Point()
            trans_parent = self.tf_to_point(self.tf_listner, tf[0], "world")
            (p.x, p.y, p.z) = trans_parent
            marker_data.points.append(p)

            p = Point()
            trans_child= self.tf_to_point(self.tf_listner,  tf[1], "world")
            (p.x, p.y, p.z) = trans_child
            marker_data.points.append(p)

        

        
        self.pub.publish(marker_data)
        

    def moving_sphere(self, tf_list, color_rgba = (0.1,0.1,0.9,1.0)):
        marker_data = Marker()
        marker_data.header.frame_id = "world"
        marker_data.header.stamp = rospy.Time.now()

        marker_data.ns = rospy.get_namespace() + rospy.get_name() + "sphere_line"
        marker_data.id = 0
        marker_data.action = Marker.ADD
        marker_data.lifetime = rospy.Duration(0.2)
        marker_data.type = 7 #sphere_list

        (marker_data.color.r, marker_data.color.g, marker_data.color.b, marker_data.color.a) = color_rgba

        (marker_data.pose.position.x, marker_data.pose.position.y, marker_data.pose.position.z)  = (0,0,0)
        (marker_data.pose.orientation.x, marker_data.pose.orientation.y, marker_data.pose.orientation.z, marker_data.pose.orientation.w) = (0,0,0,1)

        scale = 0.07
        marker_data.scale.x = scale
        marker_data.scale.y = scale

        for tf in tf_list:
            p = Point()
            trans_parent = self.tf_to_point(self.tf_listner, tf[0], "world")
            trans_child= self.tf_to_point(self.tf_listner,  tf[1], "world")
            rate = (self.time_sequence/self.time_sequence_max)
            (p.x, p.y, p.z) = (
                                trans_parent[0]*rate + trans_child[0]*(1-rate) ,
                                trans_parent[1]*rate + trans_child[1]*(1-rate),
                                trans_parent[2]*rate + trans_child[2]*(1-rate)
                                )
            marker_data.points.append(p)
            p = Point()
        
        self.time_sequence += 1
        # print self.time_sequence

        if self.time_sequence >= self.time_sequence_max:
            self.time_sequence = 0.0
        
        self.pub.publish(marker_data)
        # print marker_data

    
    def spin(self):
        while not rospy.is_shutdown():
            self.network_publish(self.tf_list)
            self.moving_sphere(self.tf_list)
            rospy.loginfo("spin node")
            self.rate.sleep()





if __name__ == '__main__':
    tf_list = [
                        ["bebop102","bebop101"],
                        ["bebop101","bebop102"],

                        ["bebop102","bebop103"],
                        ["bebop103","bebop102"],

                        ["bebop103","bebop101"],
                        ["bebop101","bebop103"],
                        # ["camera1/base_link","camera2/base_link"],
                        # ["camera2/base_link","camera1/base_link"],
                        #
                        # ["camera2/base_link","camera3/base_link"],
                        # ["camera3/base_link","camera2/base_link"],
                        #
                        # ["camera3/base_link","camera1/base_link"],
                        # ["camera1/base_link","camera3/base_link"]
                    ]
    try:
        nv = NetworkVisualizer(tf_list)
        nv.spin()
    except rospy.ROSInterruptException: pass
