#!/usr/bin/env python
# -*- coding: utf-8 -*-
from abc import abstractmethod
import rospy
import numpy as np
import pandas as pd
import datetime

from task_switch.field import Field

from task_switch.cbf_optimizer_ros import CBFOptimizerROS
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Empty, String, ColorRGBA
from std_msgs.msg import Int8MultiArray, MultiArrayLayout, MultiArrayDimension
from std_msgs.msg import Bool, Float32, Float32MultiArray
from vision_msgs.msg import Detection2DArray
from jsk_rviz_plugins.msg import *

import dynamic_reconfigure.client

from task_switch.transformations import *

import os
from abc import ABCMeta


class AgentManagerBase(object):
    __metaclass__ = ABCMeta

    def __init__(self):
        # ROS Initialize
        rospy.init_node("agent_manager", anonymous=True)
        self.agentNum = rospy.get_param("/agentNum")
        self.agentID = rospy.get_param("agentID")
        self.clock = rospy.get_param("/clock")

        # allPositions includes myself position
        self.allPositions = np.zeros((self.agentNum, 3))
        # initialize CBF Optimizer
        self.optimizer = CBFOptimizerROS()

        # publisher for agent control
        self.pub_cmd_input = rospy.Publisher("cmd_input", Twist, queue_size=1)
        # subscriber to get own pose
        rospy.Subscriber(
            "posestamped", PoseStamped, self.poseStampedCallback, queue_size=1
        )
        # subscriber to other agent's position
        rospy.Subscriber("/allPose", PoseArray, self.poseArrayCallback, queue_size=1)

        # ref for altitude
        self.zRef = 1.0
        # collision avoidance radius
        # each agents keeps collisionR * 2 distance from other agents
        self.collisionR = 0.5
        self._log = []

    def init(self):
        ### init after __init__
        ### after set self.voronoi by overloading __init__
        # dynamic_reconfigure
        self.pcc_dycon_client = dynamic_reconfigure.client.Client(
            "/pcc_parameter", timeout=2, config_callback=self.pccConfigCallback
        )
        self.cbf_dycon_client = dynamic_reconfigure.client.Client(
            "/cbf_parameter", timeout=2, config_callback=self.cbfConfigCallback
        )
        config = self.pcc_dycon_client.get_configuration()
        self.pccConfigCallback(config)
        config = self.cbf_dycon_client.get_configuration()
        self.cbfConfigCallback(config)
        # subscriber to get field information density [TODO] phi_compress
        rospy.Subscriber("/phi", Float32MultiArray, self.mainCallback, queue_size=1)

    ###################################################################
    ### dynamic config callback functions
    ###################################################################

    def pccConfigCallback(self, config):
        delta_decrease = config.delta_decrease
        self.voronoi.setDelta(delta_decrease)

    def cbfConfigCallback(self, config):
        self.optimizer.updateCbfConfig(config)
        self.voronoi.setGamma(config.gamma)

    ###################################################################
    ### subscriber callback functions
    ###################################################################
    def mainCallback(self, msg_data):
        # subscriber to get information density
        self.updatePhi(msg_data)
        self.updatePos()
        self.calcCmdInput()
        self.publishCmdInput()

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

    def poseArrayCallback(self, msg):
        # subscriber to get every agent's position
        arraynum = len(msg.poses)
        if arraynum == self.agentNum:
            for i in range(arraynum):
                pos = [
                    msg.poses[i].position.x,
                    msg.poses[i].position.y,
                    msg.poses[i].position.z,
                ]
                self.allPositions[i] = pos

    ###################################################################
    ### publisher functions
    ###################################################################
    def publishCmdInput(self):
        self.pub_cmd_input.publish(self._cmd_input)

    ###################################################################
    ### calculation function
    ###################################################################
    def calcAltitudeControl(self):
        self._cmd_input.linear.z = 0.6 * (
            self.zRef - self.position[2]
        )  # altitude control

    def resetCmdInput(self):
        self._cmd_input = Twist()

    def updatePos(self):
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
        self.neighborPosOnly = neighborPosOnly

    def calcBodyVelocity(self, ux, uy):
        quat = np.array(self.orientation)
        # transform to rotation matrix
        rotm_ = quaternion_matrix(quat)
        rotm = rotm_[0:3, 0:3]

        # calc body velocity
        body_vel = np.dot(rotm.transpose(), np.vstack([ux, uy, 0]))
        self._cmd_input.linear.x = body_vel[0]
        self._cmd_input.linear.y = body_vel[1]
        self._cmd_input.angular.z = -self.orientation[2]

    def calcCmdInput(self):
        self.resetCmdInput()
        self.calcAltitudeControl()
        if self.position[2] < self.zRef * 0.7:  # enough altitude--> input pcc command
            return
        self.voronoi.calcRegion()
        ux, uy, opt_status, task = self.Vel2dCommandCalc()
        self.calcBodyVelocity(ux, uy)

    def updatePhi(self, msg_data):
        phi = self.msg2phi(msg_data)
        self.voronoi.setPhi(phi)

    def __del__(self):
        self.savelog()

    ###################################################################
    ### abstruct method
    ###################################################################
    @abstractmethod
    def Vel2dCommandCalc(self):
        # calculate command for agent
        pass

    def savelog(self, other_str):
        CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
        data_dir = CURRENT_DIR + "/../../data/"
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
            columns=["H", "u", "dJdp", "delta"],
        )
        df.to_csv(filename + ".csv", index=False)
        rospy.loginfo("save " + filename)

    @abstractmethod
    def msg2phi(self, msg_data):
        pass


from task_switch.voronoiXZTheta import VoronoiXZTheta


class AgentManagerXTheta(AgentManagerBase):
    def __init__(self):
        super(AgentManagerXTheta, self).__init__()
        field_param = rospy.get_param("/compressed_field")
        self.voronoi = VoronoiXZTheta(field_param)
        sigma = rospy.get_param("/sigma")
        self.voronoi.setSigma(sigma)
        self._u_old = [[0], [0]]
        self._H_old = 0
        self._p_old = [0, 0]
        self._dHdt_old = 0

    def msg2phi(self, msg_data):
        # msg_data is information density, which is vectorized.
        info = np.array(msg_data.data).reshape(
            (msg_data.layout.dim[0].size, msg_data.layout.dim[1].size)
        )
        return info

    def Vel2dCommandCalc(self):
        # calculate command for agent
        pos = self.position[0:2]
        AgentPos = [pos[0], pos[1], 0.0, 0.0, 0.0, 0.0]
        neighborPosOnly = self.neighborPosOnly
        currentEnergy = 1000  # [TODO]

        #### cbf persistent coverage #####################################
        self.voronoi.calcdJdp()
        dJdp_x, dJdp_y = self.voronoi.getdJdp()
        dJdp = [dJdp_x, dJdp_y, 0, 0, 0, 0]
        xi = [self.voronoi.getXi()]

        u_nom_x = 0.0  # np.sign(dJdp_x) * self._u_base
        u_nom = np.array(
            [
                [u_nom_x],
                [0.0],
                [0.0],
                [0.0],
                [0.0],
                [0.0],
            ]
        )
        u, opt_status, task = self.optimizer.optimize(
            u_nom, AgentPos, currentEnergy, dJdp, xi, neighborPosOnly, self.collisionR
        )
        rospy.loginfo(
            "ID,p, dJdp, u : {}, {:.4f},{:.4f},{:.4f}, {:.4f}".format(
                self.agentID, pos[0], dJdp_x, u[0][0], self.clock
            )
        )
        H = self.voronoi.getH()  # calc H by drone

        dHdt = (H - self._H_old) * self.clock
        d2Hdt2 = (dHdt - self._dHdt_old) * self.clock
        self._log.append(
            [
                H,
                u[0][0],
                dJdp_x,
                self.optimizer.delta[0][0],
                dHdt,
                -self.voronoi.getGamma(),
                xi[0],
                u_nom_x,
                self.voronoi.getdHdt()[0],
                d2Hdt2,
                -dJdp_x * u[0][0] + self.voronoi._temp2,
                (pos[0] - self._p_old[0]) * self.clock,
            ]
        )
        # self.J_tilde_log.append(self.J - self.voronoi.gamma)
        self._p_old = pos
        self._u_old = u
        self._H_old = H
        self._dHdt_old = dHdt
        self.voronoi.setU(u[0])
        # self.voronoi.myUpdatePhi()
        return u[0], u[1], opt_status, task


if __name__ == "__main__":
    agent = AgentManagerXTheta()
    agent.init()
    rospy.spin()
