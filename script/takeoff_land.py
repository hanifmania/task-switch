#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String, Empty

class TakeoffLand:
    def __init__(self):
        self.bebop_number = rospy.get_param("~bebop_number", default="1")
        self.droneid = 'bebop10'+str(self.bebop_number)
        self.pub_takeoff = rospy.Publisher("takeoff", Empty, queue_size=1)
        self.pub_land = rospy.Publisher("land", Empty, queue_size=1)
        self.subtopic_takeoff = "/" + self.droneid + "/cmd_takeoff"
        self.sub_takeoff = rospy.Subscriber(self.subtopic_takeoff, String, self.callbackTakeoff)
        self.subtopic_land  = "/" + self.droneid + "/cmd_land"
        self.sub_land = rospy.Subscriber(self.subtopic_land, String, self.callbackLand)
        self.takeoffflag = False
        self.landflag = False
        self.clock = 100
        self.rate = rospy.Rate(self.clock)

    def callbackTakeoff(self, msg):
        data = msg.data
        if data == "takeoff":
            self.takeoffflag = True
        else:
            self.takeoffflag = False

    def callbackLand(self, msg):
        data = msg.data
        if data == "land":
            self.landflag = True
        else:
            self.landflag = False

    def sendTakeoff(self):
        empty = Empty()
        self.pub_takeoff.publish(empty)
        
    def sendLand(self):
        empty = Empty()
        self.pub_land.publish(empty)


    def main(self):
        while not rospy.is_shutdown():
            if self.landflag:
                self.sendLand()
                rospy.loginfo("land send")
            elif self.takeoffflag:
                self.sendTakeoff()
                rospy.loginfo("takeoff send")
            
            self.rate.sleep()
            # rospy.spin()


if __name__ == '__main__':
    rospy.init_node('takeoffland', anonymous=True)
    rospy.loginfo("starting node")
    takeoffland = TakeoffLand()
    takeoffland.main()
