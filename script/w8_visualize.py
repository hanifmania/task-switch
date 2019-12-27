#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point



def net_publish(point_list):
    marker_data = Marker()
    marker_data.header.frame_id = "world"
    marker_data.header.stamp = rospy.Time.now()

    marker_data.ns = "w8_3dmap"
    marker_data.id = 0
    marker_data.action = Marker.ADD

    for x in point_list:
        p = Point()
        p.x = x[0][0]
        p.y = x[0][1]
        p.z = x[0][2]
        marker_data.points.append(p)

        p = Point()
        p.x = x[1][0]
        p.y = x[1][1]
        p.z = x[1][2]
        marker_data.points.append(p)

    marker_data.color.r = 0.0
    marker_data.color.g = 1.0
    marker_data.color.b = 0.0
    marker_data.color.a = 1.0

    marker_data.scale.x = 0.01
    marker_data.lifetime = rospy.Duration()
    marker_data.type = 5

    pub.publish(marker_data)
    # rospy.loginfo("spin node")

def wall_publish(point_list):
    marker_data = Marker()
    marker_data.header.frame_id = "world"
    marker_data.header.stamp = rospy.Time.now()

    marker_data.ns = "w8_3dmap"
    marker_data.id = 1
    marker_data.action = Marker.ADD

    for x in point_list:
        p = Point()
        p.x = x[0][0]
        p.y = x[0][1]
        p.z = x[0][2]
        marker_data.points.append(p)

        p = Point()
        p.x = x[1][0]
        p.y = x[1][1]
        p.z = x[1][2]
        marker_data.points.append(p)

    marker_data.color.r = 0.6
    marker_data.color.g = 0.6
    marker_data.color.b = 0.6
    marker_data.color.a = 1.0

    marker_data.scale.x = 0.05
    marker_data.lifetime = rospy.Duration()
    marker_data.type = 5

    pub.publish(marker_data)
    # rospy.loginfo("spin node")
    

if __name__ == '__main__':
    try:
        rospy.init_node("w8_marker_pub")
        pub = rospy.Publisher("visualization_marker", Marker, queue_size = 10)
        rate = rospy.Rate(100)

        net_x = [-2.0, 2.3]
        net_y = [-1.5, 1.7]
        net_z = 3.5
        wall_x = net_x[1]+1
        wall_y = net_y[1] + 1
        wall_z = 4


        net_points = [
                        [[net_x[0],net_y[0],0],[net_x[1],net_y[0],0]], #床
                        [[net_x[1],net_y[0],0],[net_x[1],net_y[1],0]],
                        [[net_x[1],net_y[1],0],[net_x[0],net_y[1],0]],
                        [[net_x[0],net_y[1],0],[net_x[0],net_y[0],0]],

                        [[net_x[0],net_y[0],net_z],[net_x[1],net_y[0],net_z]],# 天井
                        [[net_x[1],net_y[0],net_z],[net_x[1],net_y[1],net_z]],
                        [[net_x[1],net_y[1],net_z],[net_x[0],net_y[1],net_z]],
                        [[net_x[0],net_y[1],net_z],[net_x[0],net_y[0],net_z]],

                        [[net_x[0],net_y[0],0],[net_x[0],net_y[0],net_z]],# 側面
                        [[net_x[1],net_y[0],0],[net_x[1],net_y[0],net_z]],
                        [[net_x[1],net_y[1],0],[net_x[1],net_y[1],net_z]],
                        [[net_x[0],net_y[1],0],[net_x[0],net_y[1],net_z]]
                    ]
        wall_points = [      
                        [[-wall_x,-wall_y,0],[wall_x,-wall_y,0]], #床
                        [[wall_x,-wall_y,0],[wall_x,wall_y,0]],
                        [[wall_x,wall_y,0],[-wall_x,wall_y,0]],
                        [[-wall_x,wall_y,0],[-wall_x,-wall_y,0]],

                        [[-wall_x,-wall_y,wall_z],[wall_x,-wall_y,wall_z]],# 天井
                        [[wall_x,-wall_y,wall_z],[wall_x,wall_y,wall_z]],
                        [[wall_x,wall_y,wall_z],[-wall_x,wall_y,wall_z]],
                        [[-wall_x,wall_y,wall_z],[-wall_x,-wall_y,wall_z]],

                        [[-wall_x,-wall_y,0],[-wall_x,-wall_y,wall_z]],# 側面
                        [[wall_x,-wall_y,0],[wall_x,-wall_y,wall_z]],
                        [[wall_x,wall_y,0],[wall_x,wall_y,wall_z]],
                        [[-wall_x,wall_y,0],[-wall_x,wall_y,wall_z]]
                        ]
        while not rospy.is_shutdown():
            net_publish(net_points)
            wall_publish(wall_points)
            rate.sleep()

    except rospy.ROSInterruptException: pass
