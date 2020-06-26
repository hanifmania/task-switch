#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from std_msgs.msg import Float32,Bool
import tf
# import dynamic_reconfigure.client
from task_switch.transformations import *
from task_switch.se3_operations import *
from task_switch.ros_utility import *

class Energy(object):
    def __init__(self,initialEnergy,drainRate):
        # drain rate is -Kd * stepsize
        self.energy = initialEnergy
        self.drainRate = drainRate
        self.chargeState = False

    def updateEnergy(self):
        if self.chargeState:
            self.energy = self.energy + 5*self.drainRate
        else:
            self.energy = self.energy - self.drainRate

    def updateChargeState(self,chargeState):
        self.chargeState = chargeState

    def getEnergy(self):
        return self.energy


class RigidBodyMotion(object):
    def __init__(self): 


        self.g_dot = self.g_dot_old = np.zeros((4,4), dtype=np.float64)
        # ROS Initialize
        rospy.init_node('rigid_body_motion', anonymous=True)
        self.node_name = rospy.get_name()

        #getROSparam

        self.tf_prefix = rospy.get_param("~tf_prefix","")
        clock = rospy.get_param("~clock",100)
        self.dt = 1.0/clock
        # pose_topic = rospy.get_param("~pose_topic","rigid_body_motion/pose")
        # self.my_frame = rospy.get_param("~my_frame","/rigid_body_motion")
        self.g = point_rpy_to_gmatrix(
                rospy.get_param("~initial_pose/x",0),
                rospy.get_param("~initial_pose/y",0),
                rospy.get_param("~initial_pose/z",0),
                rospy.get_param("~initial_pose/R",0) /180.0 * 3.14159265358979,
                rospy.get_param("~initial_pose/P",0) /180.0 * 3.14159265358979,
                rospy.get_param("~initial_pose/Y",0) /180.0 * 3.14159265358979,
                axes="rxyz"
                )

        # create energy
        initialEnergy = 2000
        Kd = 50 # per seconds
        self.energy = Energy(initialEnergy,Kd*self.dt) 

        #subscriber
        rospy.Subscriber(self.node_name + "/command/pose", Pose, self.pose_callback, queue_size=1)
        rospy.Subscriber(self.node_name + "/command/cmd_vel", Twist, self.twist_callback, queue_size=1)
        rospy.Subscriber(self.node_name + "/chargeState", Bool, self.bool_callback, queue_size=1)

        # publisher
        self.pose_pub = rospy.Publisher(self.node_name + "/pose", PoseStamped, queue_size=1)
        self.velocity_pub = rospy.Publisher(self.node_name + "/velocity", Twist, queue_size=1)
        self.energy_pub = rospy.Publisher(self.node_name + "/energy", Float32, queue_size=1)

        #dynamic_reconfigure
        #   self.dycon_client = dynamic_reconfigure.client.Client("vfc_parameter", timeout=2, config_callback=self.config_callback)

        # param initialize
        self.rate = rospy.Rate(clock)
        # self.g = np.eye(4, dtype=np.float64)

        # g_init = [rospy.get_param("~initial_pose/x","0"), rospy.get_param("~initial_pose/y","0"), rospy.get_param("~initial_pose/z","0")]
        # self.g[0:3,3:4] = np.c_[np.array(g_init)]

        self.velocity = Twist()

        print "initial pose :"
        print self.g

        # def config_callback(self,config):
        #   self.rate = rospy.Rate(config.sampling_clock)
        #   self.dt = 1.0/config.sampling_clock
        #   rospy.loginfo("Dynamic Reconfigure Prams Update")
        #   return 0


    def twist_callback(self, msg_data):
        twist_vec = np.c_[np.array([msg_data.linear.x, msg_data.linear.y, msg_data.linear.z, msg_data.angular.x, msg_data.angular.y, msg_data.angular.z])]
        twist_vec = velocity_saturator(twist_vec,linear_max=5.0, angular_max=5.0)
        self.velocity = vec6_to_twist(twist_vec)
        Vb = vec6_to_mat44(twist_vec)
        # self.g_dot = Vb.dot(self.g)
        self.g_dot = self.g.dot(Vb)
        self.integrate_g()
        rospy.loginfo("Velocity is commanded")

        return 0

    def pose_callback(self, msg_data):
        self.g = self.pose_to_g(msg_data)
        rospy.loginfo("Pose is commanded")
        return 0

    def bool_callback(self,msg_data):
        self.energy.updateChargeState(msg_data.data)

    def integrate_g(self):
        # tagged integrator(台形積分)
        self.g =  self.g + (self.g_dot + self.g_dot_old) * self.dt / 2.0
        self.g_dot_old = self.g_dot
        return self.g

    def publish_pose(self):
        pub_msg =  g_to_pose(self.g)
        # print pub_msg
        self.pose_pub.publish(pub_msg)
        return 0

    def publish_energy(self):
        self.energy.updateEnergy()
        self.energy_pub.publish(self.energy.getEnergy())

    def tf_broadcat(self):
        br = tf.TransformBroadcaster()
        pose_point = self.g[0:3,3:4].T.tolist()[0]
        R_mat = np.eye(4)
        R_mat[0:3,0:3] = self.g[0:3,0:3]
        pose_quaternion = quaternion_from_matrix(R_mat)
        br.sendTransform(pose_point,
                pose_quaternion,
                rospy.Time.now(),
                self.tf_prefix + self.node_name,
                "/world")
        return 0

    def spin(self):
        rospy.loginfo("Spinning node")
        while not rospy.is_shutdown():
           self.publish_pose()
           self.velocity_pub.publish(self.velocity)
           self.publish_energy()
           self.tf_broadcat()
           self.g = g_reprojection(self.g)
           self.rate.sleep()
           # print self.g
        return 0

if __name__ == '__main__':
    try:
        rbm = RigidBodyMotion()
        rbm.spin()

    except rospy.ROSInterruptException: pass
