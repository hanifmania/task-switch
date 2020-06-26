#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from visualization_msgs.msg import Marker
from std_msgs.msg import Float32MultiArray, ColorRGBA
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from task_switch.voronoi_main import Voronoi
from task_switch.voronoi_main import Field

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

import tf
import time
import numpy as np
import cv2 as cv
import warnings

from cv_bridge import CvBridge

class Field(Field):
    # override
    def __init__(self,mesh_acc,xlimit,ylimit):
        self.mesh_acc = mesh_acc
        self.xlimit = xlimit
        self.ylimit = ylimit
        # dimension is inverse to X,Y
        self.phi = np.zeros((self.mesh_acc[1],self.mesh_acc[0]))
    # override
    def updatePhi(self,msg):
        function_values = np.array(msg.data).reshape((self.mesh_acc[1], self.mesh_acc[0]))
        self.phi = function_values

    def updateAgentPos(self,pos):
        self.allPositions = pos

    def plotInfo(self): 
        neighborPos2d = np.delete(self.allPositions,2,axis=1)

        xlimit = self.xlimit
        ylimit = self.ylimit
        xbox = [xlimit[0],xlimit[1],xlimit[1],xlimit[0],xlimit[0]]
        ybox = [ylimit[0],ylimit[0],ylimit[1],ylimit[1],ylimit[0]]
        
        fig = plt.figure()
        ax = fig.add_subplot(111)
        ax.plot(xbox,ybox,color="k")
        X,Y = self.getGrid()
        Z = self.getPhi()
        ax.contourf(X,Y,Z)
        ax.scatter(neighborPos2d[:,0],neighborPos2d[:,1])

        fig.canvas.draw()

        data = np.frombuffer(fig.canvas.tostring_rgb(), dtype=np.uint8)
        data = data.reshape(fig.canvas.get_width_height()[::-1] + (3,))
        img = cv.cvtColor(data, cv.COLOR_RGB2BGR)

        plt.close()
        return img

class plotter():
    def __init__(self):
        # ROS Initialize
        rospy.init_node('plotter', anonymous=True)

        # field mesh accuracy
        mesh_acc = [rospy.get_param("/mesh_acc/x",100),rospy.get_param("/mesh_acc/y",150)]
        xlimit = [rospy.get_param("/x_min",-1.0),rospy.get_param("/x_max",1.0)]
        ylimit = [rospy.get_param("/y_min",-1.0),rospy.get_param("/y_max",1.0)]

        self.field = Field(mesh_acc,xlimit,ylimit)
        self.bridge = CvBridge()

        # Number of Agents
        self.agentNum = rospy.get_param("/agentNum",1)
        self.allPositions = np.zeros((self.agentNum,3)) 

        # ROS set up
        self.listner = tf.TransformListener()
        self.plotimage_pub = rospy.Publisher("plotimage", Image)
        self.surf_value_sub = rospy.Subscriber("surf_value", Float32MultiArray, self.callbackFunctionValue)

        # node freq
        self.clock = rospy.get_param("~clock",1)
        self.rate = rospy.Rate(self.clock)

    def allPositionGet(self):
        # get every agent's position

        for i in range(self.agentNum):
            try:
                # agent i's tf prefix
                agenttf = "/bebop10" + str(i+1) + "/virtualdrone"

                # get agent i's tf from world
                (position, orientation) = self.listner.lookupTransform(
                    "/world",
                     agenttf,
                      rospy.Time(0))
                # Save agent position
                self.allPositions[i] = position
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
        self.field.updateAgentPos(self.allPositions)

    def check_start(self):
        for i in range(self.agentNum):
            while not rospy.is_shutdown():
                try:
                    # agent i's tf prefix
                    agenttf = "/bebop10" + str(i+1) + "/virtualdrone"

                    # get agent i's tf from world
                    (_position, _orientation) = self.listner.lookupTransform(
                        "/world",
                         agenttf,
                          rospy.Time(0))
                    break

                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    continue
        rospy.loginfo("All tf is ready")

    def callbackFunctionValue(self, msg):
        self.field.updatePhi(msg)


    def spin(self):
        self.check_start()
        # # sampling time check 
        # count = 1
        # sample = 0
        # samplenum = 10
        # dispspan = 2
        # samplespan = int((dispspan*self.clock)/samplenum)
        # processTime = [0]*samplenum # save last 10 sample
        previousTime = 0

        while not rospy.is_shutdown():
            self.allPositionGet()

            # publish updated information density
            img = self.field.plotInfo()
            self.plotimage_pub.publish(self.bridge.cv2_to_imgmsg(img,"bgr8"))


            # sampling time check 
            currentTime = rospy.Time.now().to_sec()
            processTime_ = currentTime - previousTime
            previousTime = currentTime
            rospy.loginfo( "Plot process time is "+ str(processTime_))
            
            # if count%samplespan == 0:
            #     processTime[sample] = processTime_
            #     sample += 1

            # count += 1

            # if sample == samplenum:
            #     averagetime = sum(processTime)/len(processTime)
            #     count = 1
            #     sample = 0
            #     # print "Sampling time is ", averagetime
            #     rospy.loginfo( "Plot process time is "+ str(averagetime))


            self.rate.sleep()

        cv.destroyAllWindows()
    

if __name__=="__main__":
    warnings.simplefilter('ignore', UserWarning)
    try:
        plotter = plotter()
        plotter.spin()

    except rospy.ROSInterruptException: pass
