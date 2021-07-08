#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import numpy as np
import os
import pandas as pd
import datetime
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, MultiArrayLayout

from task_switch.agent_manager_base import AgentManagerBase
from task_switch.voronoi_xztheta import VoronoiXThetaCompressed
from task_switch.field import Field

import rospy
import numpy as np
import pandas as pd
import datetime
import os


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
from task_switch.field import Field
from task_switch.cbf_optimizer_ros import CBFOptimizerROS


class AgentManagerXTheta(AgentManagerBase):
    def __init__(self, VoronoiClass):
        rospy.init_node("AgentManager", anonymous=True)
        field_param = rospy.get_param("/drone_field")
        self.field = Field(field_param)
        self.field._point_dense = 1  # A = 1
        self.voronoi = VoronoiClass(self.field)
        sigma = rospy.get_param("/sigma")
        self.voronoi.setSigma(sigma)
        self._u_old = [[0], [0]]
        self._H_old = 0
        self._p_old = [0, 0]
        self._dHdt_old = 0
        rospy.Subscriber(
            "/compressed_phi",
            Float32MultiArray,
            self.Float32MultiArrayCallback,
            queue_size=1,
        )

    def init(self):
        # wait for pose array to get neighbor position
        self.checkNeighborStart = False
        self.agentNum = rospy.get_param("/agentNum")
        self.agentID = rospy.get_param("agentID")

        # param initialize
        self.clock = rospy.get_param("clock")
        self.rate = rospy.Rate(self.clock)

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

        # charging configs. any number is ok because it will be overwitten by dycon
        self.maxEnergy = 4000
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
        xlimit = self.field.getLimit(0)
        ylimit = [-0.1, 0.1]
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
        minEnergy = 1500
        Kd = 50  # per seconds
        k_charge = 0.15
        self.optimizer.setChargeSettings(minEnergy, Kd, k_charge)

        # for object detection
        self.object = Detection2DArray()
        self.pubObjectMsg = PoseStamped()
        self.pubObjectMsg.header.frame_id = "/world"

        rospy.loginfo("starting node:agent" + str(self.agentID))

        self._log = []
        self._start = True

    def Float32MultiArrayCallback(self, msg_data):
        # subscriber to get information density
        # msg_data is information density, which is vectorized.
        info = np.array(msg_data.data).reshape((msg_data.layout.dim[0].size))
        # rospy.loginfo("agent")
        # rospy.loginfo(info)
        self.field.setPhi(info)
        self._start = True

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
        self._p_old = pos
        self._u_old = u
        self._H_old = H
        self._dHdt_old = dHdt
        self.voronoi.setU(u[0])
        return u[0], u[1], opt_status, task

    def savelog(self, other_str):
        CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
        data_dir = CURRENT_DIR + "/../../data/"
        now = datetime.datetime.now()
        filename = data_dir + now.strftime("%Y%m%d_%H%M%S") + other_str
        df = pd.DataFrame(
            data=self._log,
            columns=[
                "H",
                "u",
                "dJdp",
                "slack",
                "dHdt",
                "gamma",
                "xi",
                "u_nom_x",
                "dHdt_predict",
                "d2Hdt2",
                "d2Hdt2_predict",
                "dpdt",
            ],
        )
        df.to_csv(filename + ".csv", index=True)
        rospy.loginfo("save " + filename)

    def publishRegion(self, region):
        pass


if __name__ == "__main__":
    agent = AgentManagerXTheta(VoronoiXThetaCompressed)
    agent.init()
    agent.spin()
