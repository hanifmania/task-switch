#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Pose, PoseStamped
from gazebo_msgs.msg import ModelStates

class Remapper:
    def __init__(self):
        self.number = rospy.get_param('~number', default=1)
        self.pubtopic = '/vrpn_client_node/bebop10'+str(self.number)+'/pose'
        self.droneid = 'bebop10'+str(self.number)
        self.pub = rospy.Publisher(self.pubtopic, PoseStamped, queue_size=1)
        self.subtopic = rospy.get_param('~subtopic', default='/gazebo/model_states')
        self.model_sub = rospy.Subscriber(self.subtopic, ModelStates, self.callbackModel)

    def callbackModel(self, msg):
        data = msg
        send = PoseStamped()
        cnt = 0
        for i in data.name:
            if i==self.droneid:
                pubpose = data.pose[cnt]    
            cnt += 1

        now = rospy.get_rostime()
        send.header.stamp = now
        send.header.frame_id = self.droneid
        send.pose = pubpose

        self.pub.publish(send)

    def main(self):
        while not rospy.is_shutdown():
            # self.rate.sleep()
            rospy.spin()


if __name__ == '__main__':
    rospy.init_node('topic_remap', disable_signals=True)
    remapper = Remapper()
    remapper.main()
