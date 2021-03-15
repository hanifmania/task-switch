#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Empty, String
import tf

import numpy as np
import cv2 as cv
import os

class FromJoyToTwistPublisherForBebop(object):

    def __init__(self):
        # ROS Initialize
        rospy.init_node('joy_to_twist', anonymous=True)
        #subscriber
        rospy.Subscriber("/joy", Joy, self.joy_callback, queue_size=1)
        rospy.Subscriber("cmd_input",  Twist, self.twist_callback, queue_size=1)
        rospy.Subscriber("cmd_takeoffland", String , self.string_callback, queue_size=1)
        # publisher
        self.pub_twist = rospy.Publisher('cmd_vel2', Twist, queue_size=1)

        self.pub_takeoff = rospy.Publisher('takeoff', Empty, queue_size=1)
        self.pub_land = rospy.Publisher('land', Empty, queue_size=1)
        self.pub_reset = rospy.Publisher('reset', Empty, queue_size=1)


        #get_ROSparam
        self.clock = rospy.get_param("~clock",100)
        self.axes_linear_x = rospy.get_param("~axes_linear_x",1)
        self.axes_linear_y = rospy.get_param("~axes_linear_y",0)
        self.axes_linear_z = rospy.get_param("~axes_linear_z",4)
        self.axes_angular_z = rospy.get_param("~axes_angular_z",3)
        self.axes_linear_speed = rospy.get_param("~axes_linear_speed",7)
        self.axes_angular_speed = rospy.get_param("~axes_angular_speed",6)

        self.buttons_takeoff = rospy.get_param("~buttons_takeoff",7)
        self.buttons_land = rospy.get_param("~buttons_land",6)
        self.buttons_reset = rospy.get_param("~buttons_reset",8)

        self.buttons_destroy_window = rospy.get_param("~buttons_destroy_window",3)
        self.buttons_change_mode = rospy.get_param("~buttons_change_mode",0)

        # param initialize
        self.rate = rospy.Rate(self.clock)
        self.linear_scale = 0.8
        self.angular_scale = 1.2

        self.twist_from_joy = Twist()
        self.twist_from_twist = Twist()
        self.takeoffland = String()

        self.command_takeoff = 0
        self.command_land = 0
        self.command_reset = 0
        self.vel_command_mode = 0
        self.is_display_window = 1
        self.mode_change_button_old = 0
        self.destroy_window_button_old = 0

        # state windows
        img_path =  os.path.dirname(os.path.abspath(__file__)) +'/bebop2.jpg'
        self.img_raw = cv.imread(img_path,cv.IMREAD_COLOR)

        rospy.loginfo("starting node")
    

    def check_axes_angle(self, angle):
        if abs(angle)<=0.15:
            return 0
        else:
            return angle
            
    def joy_callback(self, data):
        self.linear_scale = self.linear_scale +  0.01 * self.check_axes_angle( data.axes[self.axes_linear_speed] )
        self.angular_scale =  self.angular_scale + 0.01 * self.check_axes_angle( data.axes[self.axes_angular_speed] )
        self.twist_from_joy.linear.x = self.linear_scale * self.check_axes_angle( data.axes[self.axes_linear_x] )
        self.twist_from_joy.linear.y = self.linear_scale * self.check_axes_angle( data.axes[self.axes_linear_y] )
        self.twist_from_joy.linear.z = self.linear_scale * self.check_axes_angle( data.axes[self.axes_linear_z] )
        self.twist_from_joy.angular.x = 0
        self.twist_from_joy.angular.y = 0
        self.twist_from_joy.angular.z = self.angular_scale * self.check_axes_angle( data.axes[self.axes_angular_z] )

        self.command_takeoff = data.buttons[self.buttons_takeoff] 
        self.command_land =  data.buttons[self.buttons_land]
        self.command_reset = data.buttons[self.buttons_reset]

        if data.buttons[self.buttons_destroy_window] == 1 and self.destroy_window_button_old == 0:
            self.is_display_window = not self.is_display_window
        self.destroy_window_button_old = data.buttons[self.buttons_destroy_window]

        if data.buttons[self.buttons_change_mode] == 1 and self.mode_change_button_old == 0:
            self.vel_command_mode =  (self.vel_command_mode + 1) % 2
        self.mode_change_button_old = data.buttons[self.buttons_change_mode]
    
    def twist_callback(self, data):
        self.twist_from_twist = data

    def string_callback(self, data):
        self.takeoffland = data.data
    
    def publish_empty(self):
        empty_msg = Empty()
        if self.vel_command_mode == 1:
            if self.takeoffland == 'takeoff':
                self.pub_takeoff.publish(empty_msg)
            if self.takeoffland == 'land':
                self.pub_land.publish(empty_msg)

        if self.command_takeoff == 1:
            self.pub_takeoff.publish(empty_msg)
            # rospy.logwarn("send takeoff")
        if self.command_land == 1:
            self.pub_land.publish(empty_msg)
            # rospy.logwarn("send land")
        if self.command_reset == 1:
            self.pub_reset.publish(empty_msg)
        
    def draw_state_window(self):
        img = self.img_raw.copy()
        
        cv.putText(img,'Bebop2 Control',(50,50), cv.FONT_HERSHEY_COMPLEX_SMALL, 2,(255,255,255),2,cv.LINE_AA)
        cv.putText(img,"ROS Time : " + str(rospy.Time.now()),(550,50), cv.FONT_HERSHEY_COMPLEX_SMALL, 1,(255,255,255),1,cv.LINE_AA)
        cv.putText(img,"node name : " + rospy.get_name(),(550,75), cv.FONT_HERSHEY_COMPLEX_SMALL, 1,(255,255,255),1,cv.LINE_AA)
        cv.putText(img,"name space : " + rospy.get_namespace(),(550,100), cv.FONT_HERSHEY_COMPLEX_SMALL, 1,(255,255,255),1,cv.LINE_AA)

        if self.vel_command_mode == 0:
            cv.putText(img,"Mode : Joy Control",(50,100), cv.FONT_HERSHEY_COMPLEX_SMALL, 1.5,(255,255,255),2,cv.LINE_AA)

            cv.putText(img,'Linear scale : ' + str(self.linear_scale),(50,130), cv.FONT_HERSHEY_COMPLEX_SMALL, 1,(255,255,255),1,cv.LINE_AA)
            cv.putText(img,'Angular scale : ' + str(self.angular_scale),(50,150), cv.FONT_HERSHEY_COMPLEX_SMALL, 1,(255,255,255),1,cv.LINE_AA)

            cv.putText(img,"linear vel X: " + str(self.twist_from_joy.linear.x) ,(50,190), cv.FONT_HERSHEY_COMPLEX_SMALL, 1,(255,255,255),1,cv.LINE_AA)
            cv.putText(img,"linear vel Y: " + str(self.twist_from_joy.linear.y) ,(50,210), cv.FONT_HERSHEY_COMPLEX_SMALL, 1,(255,255,255),1,cv.LINE_AA)
            cv.putText(img,"linear vel Z: " + str(self.twist_from_joy.linear.z) ,(50,230), cv.FONT_HERSHEY_COMPLEX_SMALL, 1,(255,255,255),1,cv.LINE_AA)
            vel_norm = (abs(self.twist_from_joy.linear.x)  + abs(self.twist_from_joy.linear.y) + abs(self.twist_from_joy.linear.z))  / (3 *self.linear_scale)
            cv.rectangle(img,(50,250),(50+int(vel_norm*1000),270),(50,50,50 + int(vel_norm*200)),-1)
            cv.rectangle(img,(50,250),(990,270),(120,120,120),2)

            cv.putText(img,"angular vel X: " + str(self.twist_from_joy.angular.x) ,(50,330), cv.FONT_HERSHEY_COMPLEX_SMALL, 1,(255,255,255),1,cv.LINE_AA)
            cv.putText(img,"angular vel Y: " + str(self.twist_from_joy.angular.y) ,(50,350), cv.FONT_HERSHEY_COMPLEX_SMALL, 1,(255,255,255),1,cv.LINE_AA)
            cv.putText(img,"angular vel Z: " + str(self.twist_from_joy.angular.z) ,(50,370), cv.FONT_HERSHEY_COMPLEX_SMALL, 1,(255,255,255),1,cv.LINE_AA)
            ang_norm = (abs(self.twist_from_joy.angular.x)  + abs(self.twist_from_joy.angular.y) + abs(self.twist_from_joy.angular.z))  / (3 *self.angular_scale)
            cv.rectangle(img,(50,390),(50+int(ang_norm*2000),410),(50 + int(ang_norm*400),50,50),-1)
            cv.rectangle(img,(50,390),(990,410),(120,120,120),2)
        
        if self.vel_command_mode == 1:
            cv.putText(img,"Mode : Message Control",(50,100), cv.FONT_HERSHEY_COMPLEX_SMALL, 1.5,(0,0,255),2,cv.LINE_AA)

            cv.putText(img,"linear vel X: " + str(self.twist_from_twist.linear.x) ,(50,190), cv.FONT_HERSHEY_COMPLEX_SMALL, 1,(255,255,255),1,cv.LINE_AA)
            cv.putText(img,"linear vel Y: " + str(self.twist_from_twist.linear.y) ,(50,210), cv.FONT_HERSHEY_COMPLEX_SMALL, 1,(255,255,255),1,cv.LINE_AA)
            cv.putText(img,"linear vel Z: " + str(self.twist_from_twist.linear.z) ,(50,230), cv.FONT_HERSHEY_COMPLEX_SMALL, 1,(255,255,255),1,cv.LINE_AA)
            vel_norm = (abs(self.twist_from_twist.linear.x)  + abs(self.twist_from_twist.linear.y) + abs(self.twist_from_twist.linear.z))  / (3 *self.linear_scale)
            cv.rectangle(img,(50,250),(50+int(vel_norm*1000),270),(120,120,100 + int(vel_norm*200)),-1)
            cv.rectangle(img,(50,250),(990,270),(120,120,120),2)

            cv.putText(img,"angular vel X: " + str(self.twist_from_twist.angular.x) ,(50,330), cv.FONT_HERSHEY_COMPLEX_SMALL, 1,(255,255,255),1,cv.LINE_AA)
            cv.putText(img,"angular vel Y: " + str(self.twist_from_twist.angular.y) ,(50,350), cv.FONT_HERSHEY_COMPLEX_SMALL, 1,(255,255,255),1,cv.LINE_AA)
            cv.putText(img,"angular vel Z: " + str(self.twist_from_twist.angular.z) ,(50,370), cv.FONT_HERSHEY_COMPLEX_SMALL, 1,(255,255,255),1,cv.LINE_AA)
            ang_norm = (abs(self.twist_from_twist.angular.x)  + abs(self.twist_from_twist.angular.y) + abs(self.twist_from_twist.angular.z))  / (3 *self.angular_scale)
            cv.rectangle(img,(50,390),(50+int(ang_norm*2000),410),(100 + int(ang_norm*200),120,120),-1)
            cv.rectangle(img,(50,390),(990,410),(120,120,120),2)

        if self.command_takeoff == 1:
            cv.putText(img,"COMMAND TAKE OFF !",(50,470), cv.FONT_HERSHEY_COMPLEX_SMALL, 3,(255,0,0),3,cv.LINE_AA)
        if self.command_land == 1:
            cv.putText(img,"COMMAND LAND !",(50,470), cv.FONT_HERSHEY_COMPLEX_SMALL, 3,(0,0,255),3,cv.LINE_AA)
        if self.command_reset == 1:
            cv.putText(img,"COMMAND RESET !",(50,470), cv.FONT_HERSHEY_COMPLEX_SMALL, 3,(0,255,255),3,cv.LINE_AA)
        window_name = rospy.get_namespace() + rospy.get_name()
        cv.namedWindow(window_name, cv.WINDOW_NORMAL)
        cv.imshow(window_name,img)
        cv.waitKey(1)


    def spin(self):
        while not rospy.is_shutdown():
            if self.command_land == 0:
                if self.vel_command_mode == 0:
                    self.pub_twist.publish(self.twist_from_joy)
                if self.vel_command_mode == 1:
                    self.pub_twist.publish(self.twist_from_twist)

            self.publish_empty()
            if self.is_display_window == 1:
                self.draw_state_window()
            if self.is_display_window == 0:
                cv.destroyAllWindows()
            
            self.rate.sleep()

        cv.destroyAllWindows()
        

if __name__ == '__main__':
    try:
        j2t = FromJoyToTwistPublisherForBebop()
        j2t.spin()

    except rospy.ROSInterruptException: pass
