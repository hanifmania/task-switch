#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
import rospy
from std_msgs.msg import Float32,Bool
# import dynamic_reconfigure.client

class virtualEnergy(object):
    def __init__(self):
        rospy.init_node('virtual_energy', anonymous=True)
        self.node_name = rospy.get_name()

        clock = rospy.get_param("~clock",100)
        self.dt = 1.0/clock
        # drain rate is -Kd * stepsize
        


        #subscriber
        rospy.Subscriber(self.node_name + "/drainRate", Float32, self.float_callback, queue_size=1)
        # publisher
        self.energy_pub = rospy.Publisher(self.node_name + "/energy", Float32, queue_size=1)

        self.drainRate = 0.
        self.energy = rospy.get_param("initialEnergy",2000)
        # Kd will be updated by subscriber

        self.rate = rospy.Rate(clock)

    def float_callback(self,msg_data):
        self.updateDrainRate(msg_data.data)

    def updateEnergy(self):
        self.energy = self.energy - self.drainRate*self.dt

    def updateDrainRate(self,drainRate):
        self.drainRate = drainRate

    def getEnergy(self):
        return self.energy

    def publish_energy(self):
        self.updateEnergy()
        self.energy_pub.publish(self.getEnergy())

    def spin(self):
        rospy.loginfo("Spinning node")
        while not rospy.is_shutdown():
           self.publish_energy()
           self.rate.sleep()
           # print self.g
        return 0

if __name__ == '__main__':
    try:
        ve = virtualEnergy()
        ve.spin()

    except rospy.ROSInterruptException: pass
