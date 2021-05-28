#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import numpy as np
import pandas as pd
import datetime
from task_switch.theta_1d_field import Theta1dField
from task_switch.theta_1d_voronoi import Theta1dVoronoi
from task_switch.voronoi import Voronoi


from task_switch.cbf_optimizer_ros import CBFOptimizerROS
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Empty, String, ColorRGBA
from std_msgs.msg import Int8MultiArray, MultiArrayLayout, MultiArrayDimension
from std_msgs.msg import Bool, Float32, Float32MultiArray
from vision_msgs.msg import Detection2DArray
from jsk_rviz_plugins.msg import *

import dynamic_reconfigure.client

from task_switch.voronoi_main import Field
from task_switch.cbf_qp_optimizer import CBFOptimizer
import tf

from task_switch.transformations import *

import numpy as np
import cv2 as cv
import os


class Theta1dAgentManager:
    def __init__(self):
        # ROS Initialize
        rospy.init_node("AgentManager", anonymous=True)

        # wait for pose array to get neighbor position
        self.checkNeighborStart = False

        # get_ROSparam
        # self.agentID = rospy.get_param("agentID",1)
        # self.agentNum = rospy.get_param("agentNum",1)
        # mesh_acc = [rospy.get_param("mesh_acc/x",100),rospy.get_param("mesh_acc/y",150)]
        # xlimit = [rospy.get_param("x_min",-1.0),rospy.get_param("x_max",1.0)]
        # ylimit = [rospy.get_param("y_min",-1.0),rospy.get_param("y_max",1.0)]
        self.agentNum = rospy.get_param("/agentNum")
        mesh_acc = [rospy.get_param("/mesh_acc/x"), rospy.get_param("/mesh_acc/y")]
        xlimit = [rospy.get_param("/x_min"), rospy.get_param("/x_max")]
        ylimit = [rospy.get_param("/y_min"), rospy.get_param("/y_max")]
        self.agentID = rospy.get_param("agentID")

        # param initialize
        self.clock = rospy.get_param("~clock", 100)
        self.rate = rospy.Rate(self.clock)

        # initialize field class
        self.field = Theta1dField(mesh_acc, xlimit, ylimit)
        # initialize voronoiCalc class
        self.voronoi = Theta1dVoronoi(self.field)
        self._voronoi2 = Voronoi(self.field)
        # initialize neighborpos
        # allPositions includes myself position
        self.allPositions = np.zeros((self.agentNum, 3))

        # initialize CBF Optimizer
        self.optimizer = CBFOptimizerROS()

        # subscriber to get own pose
        rospy.Subscriber(
            "posestamped", PoseStamped, self.poseStampedCallback, queue_size=1
        )
        # subscriber to get own energyel
        rospy.Subscriber("energy", Float32, self.energyCallback, queue_size=1)
        # subscriber to get own energyel
        rospy.Subscriber("objects", Detection2DArray, self.objectCallback, queue_size=1)
        # subscriber to get field information density
        rospy.Subscriber(
            "/info", Float32MultiArray, self.Float32MultiArrayCallback, queue_size=1
        )
        # subscriber to other agent's position
        rospy.Subscriber("/allPose", PoseArray, self.poseArrayCallback, queue_size=1)
        rospy.Subscriber("/J", Float32, self.jCallback, queue_size=1)

        # publisher for agent control
        self.pub_twist = rospy.Publisher("cmd_input", Twist, queue_size=1)
        self.pub_takeoffland = rospy.Publisher("cmd_takeoffland", String, queue_size=1)
        self.pub_reset = rospy.Publisher("reset", Empty, queue_size=1)
        # publisher for own region
        self.pub_region = rospy.Publisher("region", Int8MultiArray, queue_size=1)
        # publisher for own coverage performance value
        self.pub_JintSPlusb = rospy.Publisher("JintSPlusb", Float32, queue_size=1)
        # publisher for energy drain rate
        self.pub_drainRate = rospy.Publisher("drainRate", Float32, queue_size=1)
        # publisher to display current CBF optimization status
        self.pub_optStatus = rospy.Publisher("optStatus", OverlayText, queue_size=1)
        # publisher to display object position
        self.pub_object = rospy.Publisher("object", PoseStamped, queue_size=1)

        ### add by shimizu
        self.publish_center_region = rospy.Publisher(
            "center_region", Int8MultiArray, queue_size=1
        )

        # dynamic_reconfigure
        self.pcc_dycon_client = dynamic_reconfigure.client.Client(
            "/pcc_parameter", timeout=2, config_callback=self.pcc_config_callback
        )
        self.charge_dycon_client = dynamic_reconfigure.client.Client(
            "/charge_parameter", timeout=2, config_callback=self.charge_config_callback
        )
        self.cbf_dycon_client = dynamic_reconfigure.client.Client(
            "/cbf_parameter", timeout=2, config_callback=self.cbf_config_callback
        )

        # controller gain. any number is ok because it will be overwritten by dycon
        self.controllerGain = 0.1
        # pcc CBF parameter settings.any number is ok because it will be overwritten by dycon
        k = 1.0
        gamma = -7.0

        # charging configs. any number is ok because it will be overwitten by dycon
        self.maxEnergy = 4000
        minEnergy = 1500
        Kd = 50  # per seconds
        k_charge = 0.15

        # init enegy
        self.energy = 0
        # current charging or not charged status
        self.charging = False

        # ref for altitude
        self.zRef = 1.2
        # Threshold for altitude control
        self.zThreshold = 0.35

        # current state: ''/'takeoff'/'land'
        # each means normal/ taking off / landing
        self.takeofflandflag = ""

        # collision avoidance radius
        # each agents keeps collisionR * 2 distance from other agents
        self.collisionR = 0.5

        # field setting
        # prohibit from going out of field
        centPos = [sum(xlimit) / len(xlimit), sum(ylimit) / len(ylimit)]
        theta = 0
        norm = 6
        width = [(xlimit[1] - xlimit[0]) / 2, (ylimit[1] - ylimit[0]) / 2]
        keepInside = True
        self.optimizer.setFieldArea(centPos, theta, norm, width, keepInside)

        # charging station position and radius
        chargePos = [
            rospy.get_param("charge_station/x", 0.0),
            rospy.get_param("charge_station/y", 0.0),
        ]
        radiusCharge = rospy.get_param("charge_station/r", 0.2)

        # charging station configs.
        self.optimizer.setChargeStation(chargePos, radiusCharge)

        # charging configs. any number is ok because it will be overwitten by dycon
        self.optimizer.setChargeSettings(minEnergy, Kd, k_charge)

        # for object detection
        self.object = Detection2DArray()
        self.pubObjectMsg = PoseStamped()
        self.pubObjectMsg.header.frame_id = "/world"

        rospy.loginfo("starting node:agent" + str(self.agentID))

        self._log = []

    ###################################################################
    ### dycon update functions
    ###################################################################

    ############## PCC dycon ##########################################
    def pcc_update_config_params(self, config):
        self.controllerGain = config.controller_gain
        self.collisionR = config.collisionR
        agent_R = config.agent_R
        agent_b_ = config.agent_b_
        delta_decrease = config.delta_decrease
        delta_increase = config.delta_increase
        self.voronoi.update_agentParam(agent_R, agent_b_)
        self.voronoi.update_fieldParam(delta_decrease, delta_increase)

    def pcc_set_config_params(self):
        config = self.pcc_dycon_client.get_configuration()
        self.pcc_update_config_params(config)
        rospy.loginfo("Dynamic Reconfigure Pcc Params SET in agent" + str(self.agentID))

    def pcc_config_callback(self, config):
        self.pcc_update_config_params(config)
        rospy.loginfo(
            "Dynamic Reconfigure Pcc Params Update in agent" + str(self.agentID)
        )

    ############## charge dycon ##########################################
    def charge_update_config_params(self, config):
        self.maxEnergy = config.maxEnergy
        self.optimizer.setChargeSettings(config.minEnergy, config.Kd, config.k_charge)

    def charge_set_config_params(self):
        config = self.charge_dycon_client.get_configuration()
        self.charge_update_config_params(config)
        rospy.loginfo(
            "Dynamic Reconfigure Charge Params SET in agent" + str(self.agentID)
        )

    def charge_config_callback(self, config):
        self.charge_update_config_params(config)
        rospy.loginfo(
            "Dynamic Reconfigure Charge Params Update in agent" + str(self.agentID)
        )

    ############## cbf dycon ##########################################
    def cbf_update_config_params(self, config):
        self.optimizer.updateCbfConfig(config)
        self.voronoi.update_PccCBFParam(config.gamma, config.pcc_CBF_h_gain_k)

    def cbf_set_config_params(self):
        config = self.cbf_dycon_client.get_configuration()
        self.cbf_update_config_params(config)
        rospy.loginfo("Dynamic Reconfigure CBF Params SET in agent" + str(self.agentID))

    def cbf_config_callback(self, config):
        self.cbf_update_config_params(config)
        rospy.loginfo(
            "Dynamic Reconfigure CBF Params Update in agent" + str(self.agentID)
        )

    ###################################################################
    ### subscriber callback functions
    ###################################################################

    def poseStampedCallback(self, pose_msg):
        # subscriber to get own pose(position)
        self.position = np.array(
            [
                pose_msg.pose.position.x,
                pose_msg.pose.position.y,
                pose_msg.pose.position.z,
            ]
        )
        self.orientation = np.array(
            [
                pose_msg.pose.orientation.x,
                pose_msg.pose.orientation.y,
                pose_msg.pose.orientation.z,
                pose_msg.pose.orientation.w,
            ]
        )

    def energyCallback(self, msg):
        # subscriber to get own energy
        self.energy = msg.data

    def Float32MultiArrayCallback(self, msg_data):
        # subscriber to get information density
        # msg_data is information density, which is vectorized.
        info = np.array(msg_data.data).reshape(
            (msg_data.layout.dim[0].size, msg_data.layout.dim[1].size)
        )
        self.field.updatePhi(info)

    def poseArrayCallback(self, msg):
        # subscriber to get every agent's position
        arraynum = len(msg.poses)
        if (self.checkNeighborStart == False) and (arraynum == self.agentNum):
            # if the pose array contains every agent's pose, set the checkNeighborstart flag as True
            self.checkNeighborStart = True
            rospy.loginfo("NeighborStart")

        elif self.checkNeighborStart:
            for i in range(arraynum):
                pos = [
                    msg.poses[i].position.x,
                    msg.poses[i].position.y,
                    msg.poses[i].position.z,
                ]
                self.allPositions[i] = pos

    def objectCallback(self, msg):
        # subscriber to get own energy
        self.object = msg

    def jCallback(self, msg):
        self.J = msg.data

    ###################################################################
    ### publisher functions
    ###################################################################

    def publishRegion(self, region, target=None):
        # publish my sensing region
        # make multiarraydimension
        dim_ = []
        dim_.append(
            MultiArrayDimension(
                label="y",
                size=region.shape[0],
                stride=region.shape[0] * region.shape[1],
            )
        )
        dim_.append(
            MultiArrayDimension(label="x", size=region.shape[1], stride=region.shape[1])
        )
        # make multiarraylayout
        layout_ = MultiArrayLayout(dim=dim_)

        # vectorize region, convert cast, delete size 1 dimension.
        region_vec = np.reshape(region, (1, -1)).astype(np.int8).squeeze()

        # make Int8multiarray. numpy to list convert
        region_for_pub = Int8MultiArray(data=region_vec.tolist(), layout=layout_)
        # publish
        if target is None:
            self.pub_region.publish(region_for_pub)
        else:
            target.publish(region_for_pub)

    def publishJintSPlusb(self, JintSPlusb):
        # publish my coverage performance
        self.pub_JintSPlusb.publish(Float32(data=JintSPlusb))

    def publishOptStatus(self, optStatus, task):
        # publish my CBF optimization status
        showText = OverlayText()
        showText.text = (
            "agent"
            + str(self.agentID)
            + "'s optimization: "
            + optStatus
            + "| task: "
            + task
        )
        if optStatus == "optimal":
            showText.fg_color = ColorRGBA(25 / 255.0, 1.0, 240.0 / 255.0, 1.0)
        elif optStatus == "error":
            showText.fg_color = ColorRGBA(240.0 / 255.0, 0.0, 240.0 / 255.0, 1.0)
        else:
            showText.fg_color = ColorRGBA(240.0 / 255.0, 1.0, 25 / 255.0, 1.0)
        showText.bg_color = ColorRGBA(0.0, 0.0, 0.0, 0.2)
        self.pub_optStatus.publish(showText)

    def publishDrainRate(self, drainRate):
        # publish my current energy drain rate
        self.pub_drainRate.publish(Float32(data=drainRate))

    def publishTwist(self, twist):
        # publish my current energy drain rate
        self.pub_twist.publish(twist)

    def publishString(self, string):
        pubString = String(string)
        self.pub_takeoffland.publish(pubString)

    def publishObject(self, posestamped):
        self.pub_object.publish(posestamped)

    ###################################################################
    ### voronoi region calculation function
    ###################################################################

    def calcVoronoiRegion(self):
        # extract x,y position from x,y,z position data
        pos = self.position[0:2]
        # print pos

        # extract x,y position from list of x,y,z position
        allPos2d = np.delete(self.allPositions, 2, axis=1)
        # delete THIS agent position
        neighborPosOnly = np.delete(allPos2d, self.agentID - 1, axis=0)
        # print str(self.agentID)+"'s pos: "+str(pos)+ " neighborpos: "+str(neighborPosOnly)

        # set my x,y position, and neighbor's position
        self.voronoi.setPos(pos)
        self.voronoi.setNeighborPos(neighborPosOnly)

        # set information density
        self.voronoi.setPhi(self.field.getPhi())
        # self.voronoi.setPhi(1)
        self.voronoi.calcVoronoi()

        self._voronoi2.setPos(pos)
        self._voronoi2.setNeighborPos(neighborPosOnly)

        # set information density
        self._voronoi2.setPhi(self.field.getPhi())
        # self.voronoi.setPhi(1)
        self._voronoi2.calcVoronoi()

    ###################################################################
    ### velocity command calculation function
    ###################################################################

    def Vel2dCommandCalc(self):
        # calculate command for agent

        pos = self.position[0:2]
        AgentPos = [pos[0], pos[1], 0.0, 0.0, 0.0, 0.0]
        currentEnergy = self.energy

        # extract x,y position from list of x,y,z position
        allPos2d = np.delete(self.allPositions, 2, axis=1)
        # delete THIS agent position
        neighborPosOnly = np.delete(allPos2d, self.agentID - 1, axis=0)

        #### normal persistent coverage ##################################
        # calculate command for agent
        # different from sugimoto san paper at divided 2mass.(probably dividing 2mass is true)
        # u_nom2d = (
        #     -pos
        #     + self.voronoi.getCent()
        #     - self.voronoi.getExpand() / (2 * self.voronoi.getMass())
        # ) * self.controllerGain
        # u_nom = np.array( [ [u_nom2d[0]], [u_nom2d[1]], [0.], [0.], [0.], [0.] ]  )
        # dJdp = [0., 0., 0., 0., 0., 0.]
        # xi = [0.]

        #### cbf persistent coverage #####################################
        self.voronoi.calcdJdp()
        u_nom_val = -1 * self._voronoi2.getdJdp()

        u_nom = np.array(
            [
                [u_nom_val],
                [0.0],
                [0.0],
                [0.0],
                [0.0],
                [0.0],
            ]
        )
        # dJdp2d = 2*self.voronoi.getMass()*(self.voronoi.getCent()-pos)-self.voronoi.getExpand()
        # dJdp = [dJdp2d[0], dJdp2d[1], 0., 0., 0., 0.]

        dJdp = [self.voronoi.getdJdp(), 0, 0, 0, 0, 0]
        # xi = [self.voronoi.getXi()]
        # xi = [self.voronoi.getXi() / self.agentNum]
        xi = [(self.voronoi.k * (self.voronoi.gamma - self.J))]

        u, opt_status, task = self.optimizer.optimize(
            u_nom, AgentPos, currentEnergy, dJdp, xi, neighborPosOnly, self.collisionR
        )
        rospy.loginfo(
            "ID, dJ1dp, dJ2dp, u : {}, {:.5f}, {:.5f},{:.5f}".format(
                self.agentID, self.voronoi.getdJdp(), self._voronoi2.getdJdp(), u[0][0]
            )
        )
        self._log.append(
            [
                self.voronoi.getdJdp(),
                self._voronoi2.getdJdp(),
                u[0][0],
                self.J - self.voronoi.gamma,
            ]
        )
        # self.J_tilde_log.append(self.J - self.voronoi.gamma)
        return u[0], u[1], opt_status, task
        # return u_nom2d[0], u_nom2d[1], opt_status, task

    ###################################################################
    ### judge charging state and return energy drain rate
    ###################################################################

    def judgeCharge(self):
        pos = self.position[0:2]
        currentEnergy = self.energy
        (
            minEnergy,
            Kd,
            k_charge,
            chargePos,
            radiusCharge,
        ) = self.optimizer.getChargeStation()
        isInStation = (pos[0] - chargePos[0]) ** 2 + (
            pos[1] - chargePos[1]
        ) ** 2 < radiusCharge ** 2

        lastChargeState = self.charging

        if isInStation or self.charging:
            if currentEnergy <= minEnergy + 500:
                self.charging = True

            if currentEnergy >= self.maxEnergy:
                self.charging = False

        self.takeofflandflag = ""
        if self.charging and self.optimizer.activate_chargecbf:
            drainRate = -5 * Kd  # minus drainrate means battery is being charged
            self.takeofflandflag = "land"
            if lastChargeState == False:
                rospy.loginfo("start charge")
        else:
            drainRate = Kd * self.optimizer.activate_chargecbf
            if self.position[2] < self.zThreshold:  # margin for threshold
                self.takeofflandflag = "takeoff"
            if lastChargeState == True:
                rospy.loginfo("end charge")

        self.publishString(self.takeofflandflag)
        return drainRate

    ###################################################################
    ### for the object detection, set detected object position
    ###################################################################
    def updateObject(self):
        objGrobalPos = []
        if len(self.object.detections) > 0:
            for detection in self.object.detections:
                if detection.results[0].id == 3:
                    perception = True
                    objCenter = detection.bbox.center
                    objVector = np.array(
                        [
                            1.0 * (480.0 / 2 - objCenter.y) / 480.0,
                            1.8 * (856.0 / 2 - objCenter.x) / 856.0,
                            -self.position[2],
                        ]
                    )
                    quat = np.array(self.orientation)
                    # transform to rotation matrix
                    rotm_ = quaternion_matrix(quat)
                    rotm = rotm_[0:3, 0:3]
                    rotObjVector = np.dot(rotm, objVector)
                    objGrobalPos = (
                        np.array([self.position[0], self.position[1], 0])
                        + rotObjVector
                        - [0.0, 0.0, 0.0]
                    )
                    self.pubObjectMsg.header.stamp = rospy.Time.now()
                    self.pubObjectMsg.pose.position.x = objGrobalPos[0]
                    self.pubObjectMsg.pose.position.y = objGrobalPos[1]
                    self.publishObject(self.pubObjectMsg)

        self.setObjectPosition(objGrobalPos)

    def setObjectPosition(self, objGrobalPos):
        if objGrobalPos == []:
            self.optimizer.setPerception(False)

        else:
            centPos = objGrobalPos[0:2]
            theta = 0
            norm = 2.0
            width = [0.1, 0.1]
            keepInside = True
            self.optimizer.setPerception(True)
            self.optimizer.setStayArea(centPos, theta, norm, width, keepInside)

    ###################################################################
    ### loop
    ###################################################################

    def spin(self):
        rospy.wait_for_message("/info", Float32MultiArray)
        rospy.wait_for_message("posestamped", PoseStamped)
        rospy.loginfo("agent" + str(self.agentID) + " controller start!")

        self.pcc_set_config_params()
        self.charge_set_config_params()
        self.cbf_set_config_params()

        # initialize message
        twist = Twist()
        opt_status = "init..."
        task = "init..."
        # raw_input('PRESS ENTER')
        # twist.linear.x = 1.0
        # twist.linear.y = 0.0
        # twist.linear.z = 0.0
        # twist.angular.z = 0.0
        # self.publishTwist(twist)
        # rospy.sleep(0.1)
        while not rospy.is_shutdown():
            if self.checkNeighborStart:  # wait for all neighbor start
                twist.linear.x = 0.0
                twist.linear.y = 0.0
                twist.linear.z = 0.0
                twist.angular.z = 0.0

                # calculate voronoi region
                self.calcVoronoiRegion()

                if self.takeofflandflag == "":
                    twist.linear.z = 0.6 * (
                        self.zRef - self.position[2]
                    )  # altitude control

                    if self.position[2] > 0.7:  # enough altitude--> input pcc command
                        ux, uy, opt_status, task = self.Vel2dCommandCalc()
                        # quaternion [x,y,z,w]
                        quat = np.array(self.orientation)
                        # transform to rotation matrix
                        rotm_ = quaternion_matrix(quat)
                        rotm = rotm_[0:3, 0:3]

                        # calc body velocity
                        body_vel = np.dot(rotm.transpose(), np.vstack([ux, uy, 0]))
                        twist.linear.x = body_vel[0]
                        twist.linear.y = body_vel[1]
                        twist.angular.z = -self.orientation[2]

                self.updateObject()

                # publish drain rate
                drainRate = self.judgeCharge()
                self.publishDrainRate(drainRate)

                # publish optimization status
                # self.publishOptStatus(opt_status,task)

                # add by shimizu
                ### sensing region (smaller than voronoi region)
                grid = self.field.getGrid()
                X = grid[0]
                Y = grid[1]
                Pos = self.position[0:2]
                sensor_R = 0.1
                region = (X - Pos[0]) ** 2 + (Y - Pos[1]) ** 2 < sensor_R ** 2
                self.publishRegion(region, self.publish_center_region)

                # publish my region
                self.publishRegion(self.voronoi.getRegion())
                # publish vel command
                self.publishTwist(twist)

                # publish my coverage performance
                # JintSPlusb = self.voronoi.getJintSPlusb()
                # self.publishJintSPlusb(JintSPlusb)

            self.rate.sleep()

        self.savelog("_agent" + str(self.agentID))

    def savelog(self, other_str):
        CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
        data_dir = CURRENT_DIR + "/../data/"
        now = datetime.datetime.now()
        filename = data_dir + now.strftime("%Y%m%d_%H%M%S") + other_str
        # df = pd.DataFrame(
        #     data={
        #         "u": self.u_log,
        #         "J~": self.J_tilde_log,
        #     },
        #     columns=["u", "J~"],
        # )
        df = pd.DataFrame(
            data=self._log,
            columns=["dJ1dp", "dJ2dp", "u", "J~"],
        )
        df.to_csv(filename + ".csv", index=False)
        rospy.loginfo("save " + filename)


if __name__ == "__main__":
    agent = Theta1dAgentManager()
    agent.spin()
