#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import ConvexHull
from scipy.stats import multivariate_normal


class Voronoi:
    def __init__(self,field):

        self.phi = field.getPhi()
        self.Grid = field.getGrid()
        self.X = self.Grid[0]
        self.Y = self.Grid[1]

        self.R = 1
        self.b = self.R**2-1
        

        self.Pos = [0,0]
        self.listNeighborPos = np.array([[2,2],[-2,2]])

    def calcVoronoi(self):
        self.Region = (self.X-self.Pos[0])**2 + (self.Y-self.Pos[1])**2 <= self.R**2
        # create R+margin circle
        outside = (self.X-self.Pos[0])**2 + (self.Y-self.Pos[1])**2 <= (1.1*self.R)**2

        # calculate distance to each point in R+margin circle
        distance = (self.X*outside-self.Pos[0])**2 + (self.Y*outside-self.Pos[1])**2
        
        # eliminate R circle from outside (make donut)
        arc = outside*~self.Region

        for neighborPos in self.listNeighborPos:
            # distance to each point from neighbor
            neighborDistance = (self.X-neighborPos[0])**2 + (self.Y-neighborPos[1])**2
            # nearer points in R+margin circle
            nearRegion = neighborDistance > distance
            # delete points near to neighbor from my Region
            self.Region = self.Region*nearRegion
            # delete points near to neighbor from arc
            arc = arc*nearRegion

        weightedX = self.X*self.phi*self.Region
        weightedY = self.Y*self.phi*self.Region

        self.mass = np.sum(self.phi*self.Region)
        self.cent = np.array([weightedX.sum(), weightedY.sum()])/self.mass

        onEdgeX = self.X*arc
        onEdgeY = self.Y*arc
        onEdgePhi = self.phi*arc

        vectorX = onEdgeX-self.Pos[0] 
        vectorY = onEdgeY-self.Pos[1]
        distance = vectorX**2 + vectorY**2 

        expandX = vectorX[distance>0]/distance[distance>0]*onEdgePhi[distance>0]
        expandY = vectorY[distance>0]/distance[distance>0]*onEdgePhi[distance>0]
        self.expand = (self.R**2+self.b)*[expandX.sum(), expandY.sum()]
    
    def setPos(self,pos):
        self.Pos = pos

    def setNeighborPos(self,NeighborPos):
        self.listNeighborPos = NeighborPos

    def setPhi(self,phi):
        self.phi = phi
    
    def getCent(self):
        return self.cent

    def getRegion(self):
        return self.Region

    def getConv(self):
        onlymyX = self.X[self.Region == True]
        onlymyY = self.Y[self.Region == True]
        XY = np.concatenate([onlymyX.reshape(-1,1), onlymyY.reshape(-1,1)],1)
        hull = ConvexHull(XY)
        points = hull.points
        hull_points = points[hull.vertices]
        hp = np.vstack((hull_points, hull_points[0]))
        return hp


            
class Plotter:
    def __init__(self,span):
        self.span = span

    def voronoiPlot(self,field,Agents,pos):
        xlimit, ylimit = field.getXYlimit()
        xbox = [xlimit[0],xlimit[1],xlimit[1],xlimit[0],xlimit[0]]
        ybox = [ylimit[0],ylimit[0],ylimit[1],ylimit[1],ylimit[0]]
        plt.plot(xbox,ybox,color="k")
        X,Y = field.getGrid()
        Z = field.getPhi()
        plt.contourf(X,Y,Z)

        for i in range(len(pos)):
            hp = Agents[i].getConv()
            plt.plot(hp[:,0],hp[:,1])

        plt.scatter(pos[:,0],pos[:,1])
        plt.pause(self.span)
        plt.clf()



class Field:
    def __init__(self):
        self.mesh_acc = [200,200]
        self.xlimit = [-2.0,2.0]
        self.ylimit = [-2.0,2.0]
        # dimension is inverse to X,Y
        self.phi = np.ones((self.mesh_acc[1],self.mesh_acc[0]))

    def updatePhi(self,phi):
        self.phi = phi

    def getXYlimit(self):
        return self.xlimit, self.ylimit

    def getGrid(self):
        xgrid = np.linspace(self.xlimit[0], self.xlimit[1], self.mesh_acc[0])
        ygrid = np.linspace(self.ylimit[0], self.ylimit[1], self.mesh_acc[1])
        # X and Y are mesh_acc[1] columns, mesh_acc[0] rows matrix
        X, Y = np.meshgrid(xgrid, ygrid)
        return X,Y

    def getPhi(self):
        return self.phi


def infoUpdate(Z,Agents):
    delta_decrease = 1
    delta_increase = 0.01
    Region = Agents[1].getRegion()
    for i in range(len(Agents)):
        Region = Region + Agents[i].getRegion()
    Z = Z-delta_decrease*Region
    Z = np.where(Z<0.01,0.01,Z) 
    Z = Z + delta_increase*~Region
    Z = np.where(Z>1,1,Z) 
    return Z


def setGaussian(field):
    X, Y = field.getGrid()
    XY = np.dstack((X, Y))
    mu = [0.5, -0.2]
    sigma = [[2.0, 0.3],[0.3,0.5]]
    rv = multivariate_normal(mu, sigma)
    Z = rv.pdf(XY)
    return Z

def main():
    AgentNum = 3
    pos = -2+4*np.random.rand(AgentNum,2)
    field = Field()
    # field.updatePhi(setGaussian(field))
    Agents = []
    for i in range(AgentNum):
        agent = Voronoi(field)
        Agents.append(agent)


    plotter = Plotter(0.01)
    for t in range(100):
        try:
            for i in range(AgentNum):
                Agents[i].setPos(pos[i])
                Agents[i].setNeighborPos(np.delete(pos,i,axis=0))
                Agents[i].calcVoronoi()
                pos[i] = pos[i] + (Agents[i].getCent()-pos[i])*0.3
            Z = infoUpdate(field.getPhi(),Agents)
            field.updatePhi(Z)

            plotter.voronoiPlot(field,Agents,pos)

        except:
            break




if __name__ == '__main__':
    main()

