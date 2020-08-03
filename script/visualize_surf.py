#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from visualization_msgs.msg import Marker
from std_msgs.msg import Float32MultiArray, ColorRGBA
from geometry_msgs.msg import Point
import tf
import time
import numpy as np
import cv2

class SurfVisualize:
    '''
    Visualization class of surf plot in rviz(only colored 2D plot using CubeList).
    It's used for visualization of importance density function in coverage control.

    Subscribe : The values of the function of x and y.(Float32MultiArray)
        The grid point for each function value is assumed to be like this:
        [(-1.0,1.0), (-0.9,1.0), ... (1.0,1.0)
         (-1.0,0.9), (-0.9,0.9), ... (0.9,0.9)
         .                              .
         .                              .
         .                              .
         (-1.0,-1.0), (-0.9,-1.0), ... (0.9,-1.0)]

    Publish : Marker (CubeList) with gradation color along to the value of function.
    '''

    def __init__(self, x_region, y_region, min_value, max_value):
        # Initialize variables
        # Start flag
        self.pub_start = False
        # Subscribe message
        self.function_values = np.zeros(2)
        # Data size of message
        self.function_size_x = 0
        self.function_size_y = 0
        # Max and Min value of function
        self.min_value = min_value
        self.max_value = max_value
        # Map region
        self.x_region = x_region
        self.y_region = y_region

        # ROS set up
        self.marker_pub = rospy.Publisher("field_visualization", Marker, queue_size=1)
        self.surf_value_sub = rospy.Subscriber("surf_value", Float32MultiArray, self.callbackFunctionValue)

    def callbackFunctionValue(self, msg):
        if not self.pub_start:
            # Setting data size. Assuming dim[0].label="y" and dim[1].label="x".
            self.function_size_x = msg.layout.dim[1].size
            self.function_size_y = msg.layout.dim[0].size
            # Setting stride
            self.stride_x = msg.layout.dim[1].stride
            self.stride_y = msg.layout.dim[0].stride
            # Setting distance between each data point.
            self.x_distance = (self.x_region[1]-self.x_region[0]) / (self.function_size_x-1)
            self.y_distance = (self.y_region[1]-self.y_region[0]) / (self.function_size_y-1)
            # Start to publish marker.
            self.pub_start = True
            # Initialize function value. It's 3 dimension and uint8 because of cv2.applyColorMap()
            self.function_values = np.zeros((self.function_size_y, self.function_size_x, 1), np.uint8)

        self.function_values = np.array(msg.data).reshape((self.function_size_y, self.function_size_x))

    def CreateCubeList(self, values):
        # Setting marker message
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "surface_plot"
        marker.id = 0
        marker.type = Marker.CUBE_LIST
        marker.action = Marker.ADD
        marker.lifetime.secs = 0.1
        marker.pose.position.x = 0.
        marker.pose.position.y = 0.
        marker.pose.position.z = -0.1
        marker.pose.orientation.x = 0.
        marker.pose.orientation.y = 0.
        marker.pose.orientation.z = 0.
        marker.pose.orientation.w = 1.
        marker.scale.x = self.x_distance
        marker.scale.y = self.y_distance
        marker.scale.z = 0.01

        # Set values as 0 to 255
        mapped_values = np.zeros((values.shape[0], values.shape[1], 1), np.uint8)
        mapped_values[:, :, 0] = np.array(255*(values - self.min_value) / (self.max_value - self.min_value), np.uint8)
        # Color mapping
        colored_values = cv2.applyColorMap(mapped_values, cv2.COLORMAP_JET)

        # Set colors and points.
        for y in range(self.function_size_y):
            for x in range(self.function_size_x):
                color = ColorRGBA()
                # If the value is low, display nothing for visibility.(set alpha to 0)
                # if mapped_values[y, x, 0] < 40:
                #     color.a = 0
                # else:
                #     color.a = 0.7
                color.a = 0.2
                point = Point()
                # The data structure is assumed to be the same as x-y grid.
                point.y = self.y_region[1] - y * self.y_distance
                point.x = self.x_region[0] + self.x_distance*x
                color.r = colored_values[y, x, 2]/255.0
                color.g = colored_values[y, x, 1]/255.0
                color.b = colored_values[y, x, 0]/255.0
                marker.colors.append(color)
                marker.points.append(point)

        return marker

    def start(self, hz):
        rate = rospy.Rate(hz)
        count = 0
        times = set()
        while not rospy.is_shutdown():
            count += 1
            if not self.pub_start:
                if count%300 == 0:
                    rospy.loginfo("Waiting for Plot Value...")
            else:
                start = time.time()

                marker_msg = self.CreateCubeList(self.function_values)
                self.marker_pub.publish(marker_msg)

                # Calculate sampling time
                end = time.time()
                times.add(end - start)
                if count%300 == 0:
                    averagetime = sum(times)/300.
                    count = 0
                    times = set()
                    print "Sampling time is ", averagetime

            rate.sleep()

if __name__=="__main__":
    rospy.init_node('surf_visualization')
    max_value = rospy.get_param('~max', default=1)
    min_value = rospy.get_param('~min', default=0)
    x_region = np.zeros(2)
    y_region = np.zeros(2)
    x_region[0] = rospy.get_param("/x_min", default=-1.2)
    x_region[1] = rospy.get_param("/x_max", default=1.2)
    y_region[0] = rospy.get_param("/y_min", default=-1.7)
    y_region[1] = rospy.get_param("/y_max", default=1.5)

    surf = SurfVisualize(x_region, y_region, min_value, max_value)
    surf.start(50)
