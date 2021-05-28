#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseArray
from task_switch.voronoi_main import Voronoi
from task_switch.voronoi_main import Field
from std_msgs.msg import Empty, Float32
from std_msgs.msg import (
    Float32MultiArray,
    Int8MultiArray,
    MultiArrayLayout,
    MultiArrayDimension,
)
import tf


import dynamic_reconfigure.client

import numpy as np
import cv2 as cv
import os


class Field(Field):
    # override
    def __init__(self, mesh_acc, xlimit, ylimit):
        self.mesh_acc = mesh_acc
        self.xlimit = xlimit
        self.ylimit = ylimit
        # dimension is inverse to X,Y
        self.phi = 0.05 * np.ones((self.mesh_acc[1], self.mesh_acc[0]))
        self.phi1 = np.ones((self.mesh_acc[1], self.mesh_acc[0]))
        self.phi2 = 0.5 * np.ones((self.mesh_acc[1], self.mesh_acc[0]))
        # self.phi = np.ones((self.mesh_acc[1],self.mesh_acc[0]))
        self.pointDense = (
            (self.xlimit[1] - self.xlimit[0])
            * (self.ylimit[1] - self.ylimit[0])
            / (self.mesh_acc[0] * self.mesh_acc[1])
        )

    def setb(self, b):
        self.b = b

    def getJintQ(self):
        # calc J = - \sum \int_{S_i} \|q-p_i\|^2\phi(q,t)dq + b\int_{Q...} \phi(q,t)dq
        #        = - \sum \int_{S_i} (\|q-p_i\|^2 + b) \phi(q,t)dq + b\int_{Q} \phi(q,t)dq
        #                                    here calc this term -> ~~~~~~~~~~~~~~~~~~~~
        pointDense = (
            (self.xlimit[1] - self.xlimit[0])
            * (self.ylimit[1] - self.ylimit[0])
            / (self.mesh_acc[0] * self.mesh_acc[1])
        )
        JintQ = self.b * np.sum(self.phi) * pointDense
        return JintQ


class Collector:
    # collect each agent's sensing region and coverage performance(JintSPlusb)

    def __init__(self, agentName, mesh_acc):
        subTopicRegion = agentName + "/region"
        subTopicJintSPlusb = agentName + "/JintSPlusb"
        # subscriber for each agent's region
        rospy.Subscriber(
            subTopicRegion, Int8MultiArray, self.int8MultiArrayCallback, queue_size=1
        )
        rospy.Subscriber(
            subTopicJintSPlusb, Float32, self.Float32Callback, queue_size=1
        )

        # initialze with zeros
        self.region = np.zeros((mesh_acc[1], mesh_acc[0]), dtype=np.bool)
        self.JintSPlusb = 0
        self.ready = False

        ### add by shimizu
        rospy.Subscriber(
            agentName + "/center_region",
            Int8MultiArray,
            self.int8MultiArrayCallback2,
            queue_size=1,
        )
        self.center_region = np.zeros((mesh_acc[1], mesh_acc[0]), dtype=np.bool)

    def int8MultiArrayCallback2(self, msg_data):
        if self.ready == False:  # wait for message, can be replace by wait_for_message?
            self.ready = True
        else:
            # msg_data is region data, which is vectorized.
            region_ = np.array(msg_data.data).reshape(
                (msg_data.layout.dim[0].size, msg_data.layout.dim[1].size)
            )
            # reset to boolean value
            self.center_region = region_.astype(np.bool)

    def int8MultiArrayCallback(self, msg_data):
        if self.ready == False:  # wait for message, can be replace by wait_for_message?
            self.ready = True
        else:
            # msg_data is region data, which is vectorized.
            region_ = np.array(msg_data.data).reshape(
                (msg_data.layout.dim[0].size, msg_data.layout.dim[1].size)
            )
            # reset to boolean value
            self.region = region_.astype(np.bool)

    def Float32Callback(self, msg_data):
        self.JintSPlusb = msg_data.data

    def getRegion(self):
        return self.region

    def getCenterRegion(self):
        return self.center_region

    def getJintSPlusb(self):
        return self.JintSPlusb

    def getReady(self):
        return self.ready


class central:
    def __init__(self):
        # ROS Initialize
        rospy.init_node("central", anonymous=True)

        # field mesh accuracy
        mesh_acc = [
            rospy.get_param("/mesh_acc/x", 100),
            rospy.get_param("/mesh_acc/y", 150),
        ]
        xlimit = [rospy.get_param("/x_min", -1.0), rospy.get_param("/x_max", 1.0)]
        ylimit = [rospy.get_param("/y_min", -1.0), rospy.get_param("/y_max", 1.0)]
        self.field = Field(mesh_acc, xlimit, ylimit)

        # Number of Agents
        self.agentNum = rospy.get_param("/agentNum", 1)

        self.Collectors = []
        # createsubscriber x[Agent's number]
        for agentID in range(self.agentNum):
            agentName = "bebop10" + str(agentID + 1)
            rospy.loginfo(agentName)
            collector = Collector(agentName, mesh_acc)
            self.Collectors.append(collector)

        self.buttons_enable_info_update = rospy.get_param(
            "~buttons_enable_info_update", 2
        )
        rospy.Subscriber("/joy", Joy, self.joy_callback, queue_size=1)

        # publisher for information density
        self.pub_info = rospy.Publisher("/info", Float32MultiArray, queue_size=1)

        # publisher for coverage performance
        self.pub_J = rospy.Publisher("/J", Float32, queue_size=1)
        # publisher for target coverage performance
        self.pub_targetJ = rospy.Publisher("/targetJ", Float32, queue_size=1)

        # node freq
        self.clock = rospy.get_param("~clock", 100)
        self.rate = rospy.Rate(self.clock)

        # information decay, acquisition parmeters. any numbers are ok because it will be overwritten by dycon
        self.delta_decrease = 1.0
        self.delta_increase = 0.01
        self.b = 1

        # dynamic_reconfigure
        self.pcc_dycon_client = dynamic_reconfigure.client.Client(
            "/pcc_parameter", timeout=2, config_callback=self.pcc_config_callback
        )
        self.cbf_dycon_client = dynamic_reconfigure.client.Client(
            "/cbf_parameter", timeout=2, config_callback=self.cbf_config_callback
        )

        self.previousInfoUpdateTime = rospy.Time.now().to_sec()

        self.info_update = False

        rospy.loginfo("starting central node")

    def update_param(self, R, b_):
        self.b = -(R ** 2) - b_
        self.field.setb(self.b)

    def enable_info_update(self, arg):
        self.info_update = arg

    ###################################################################
    ### subscriber callback
    ###################################################################
    def joy_callback(self, data):
        # button_is_pushed = data.buttons[self.buttons_enable_info_update]
        button_is_pushed = True
        if button_is_pushed:
            self.enable_info_update(True)
            self.previousInfoUpdateTime = rospy.Time.now().to_sec()

    ###################################################################
    ### dycon update functions
    ###################################################################

    ############## PCC dycon ##########################################
    def pcc_update_config_params(self, config):
        self.delta_decrease = config.delta_decrease
        self.delta_increase = config.delta_increase
        self.update_param(config.agent_R, config.agent_b_)

    def pcc_set_config_params(self):
        config = self.pcc_dycon_client.get_configuration()
        self.pcc_update_config_params(config)
        rospy.loginfo("Dynamic Reconfigure Pcc Params SET in central")

    def pcc_config_callback(self, config):
        self.pcc_update_config_params(config)
        rospy.loginfo("Dynamic Reconfigure Pcc Params Update in central")

    ############## cbf dycon ##########################################
    def cbf_update_config_params(self, config):
        self.publishTargetJ(config.gamma)

    def cbf_set_config_params(self):
        config = self.cbf_dycon_client.get_configuration()
        self.cbf_update_config_params(config)
        rospy.loginfo("Dynamic Reconfigure CBF Params SET in central")

    def cbf_config_callback(self, config):
        self.cbf_update_config_params(config)
        rospy.loginfo("Dynamic Reconfigure CBF Params Update in central")

    ###################################################################
    ### publisher functions
    ###################################################################

    def publishTargetJ(self, gamma):
        self.pub_targetJ.publish(Float32(data=gamma))

    def publishJ(self, J):
        self.pub_J.publish(Float32(data=J))

    def publishInfo(self, info):
        # publish information density

        # make multiarraydimension
        dim_ = []
        dim_.append(
            MultiArrayDimension(
                label="y", size=info.shape[0], stride=info.shape[0] * info.shape[1]
            )
        )
        dim_.append(
            MultiArrayDimension(label="x", size=info.shape[1], stride=info.shape[1])
        )
        # make multiarraylayout
        layout_ = MultiArrayLayout(dim=dim_)

        # vectorize info(=phi), convert cast, delete size 1 dimension.
        info_vec = np.reshape(info, (1, -1)).astype(np.float32).squeeze()

        # make Float32multiarray. numpy to list convert
        info_for_pub = Float32MultiArray(data=info_vec.tolist(), layout=layout_)

        # publish
        self.pub_info.publish(info_for_pub)

    def infoUpdate(self, Z, region):
        # information reliability update

        # delta_decrease = 0.01
        # delta_increase = 0.0001
        currentTime = rospy.Time.now().to_sec()
        dt = currentTime - self.previousInfoUpdateTime
        self.previousInfoUpdateTime = currentTime

        Z = Z - self.delta_decrease * dt * region
        Z = np.where(Z < 0.01, 0.01, Z)
        # Z = Z + self.delta_increase * (1 - Z) * dt * ~region
        Z = np.where(Z > 1.0, 1.0, Z)

        return Z

    def spin(self):
        # check sampling time

        self.pcc_set_config_params()
        self.cbf_set_config_params()

        while not rospy.is_shutdown():
            JintSPlusb_all = 0.0
            ready = True

            # initialize region
            region = self.Collectors[0].getRegion()

            center_region = self.Collectors[0].getCenterRegion()

            # collect each agent's region by sum
            for collector in self.Collectors:
                region = region + collector.getRegion()
                center_region = center_region + collector.getCenterRegion()
                JintSPlusb_all = JintSPlusb_all + collector.getJintSPlusb()

                # wait for all collector's subscriber get region message
                ready = ready * collector.getReady()

            # get current field information density
            phi = self.field.getPhi()

            # publish current coverage performance
            # J = - JintSPlusb_all + self.field.getJintQ()
            J = np.sum(phi * ~region * self.field.pointDense)
            self.publishJ(J)

            # if any collector does not get region message from agent,
            # do not update field density
            if ready:

                ### add by shimizu
                self.field.phi1 = self.infoUpdate(self.field.phi1, region)
                # self.field.phi2 = self.field.phi2*center_region*0 + self.field.phi2*~center_region
                phi = self.field.phi1  # +self.field.phi2
                # update information density phi according to region
                # phi = self.infoUpdate(phi,region)
                self.field.updatePhi(phi)

            # publish updated information density
            if self.info_update:
                self.publishInfo(self.field.getPhi())

            self.rate.sleep()


if __name__ == "__main__":
    try:
        central = central()
        central.spin()

    except rospy.ROSInterruptException:
        pass
