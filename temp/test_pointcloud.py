#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import open3d as o3d
import numpy as np

class ShowPointCloud:
    def xztheta_xmove(self):
        num = 100
        r_max=3
        phi_max = np.pi/4
        x = np.linspace(-3, 3, 401)
        mesh_x, mesh_y = np.meshgrid( np.linspace(-3, 3, 401),  np.linspace(0, 3, 401))
        mesh_z = np.arctan2( mesh_x,r_max - mesh_y)
        region =np.abs(mesh_z) <= phi_max
        circle_x = mesh_x[region]
        circle_y = mesh_y[region]
        z = mesh_z[region]
        self.show_pointcloud(circle_x,circle_y,z)

    def show_pointcloud(self, x,y,z): #x,y,z should the same shape
        xyz = np.zeros((np.size(x), 3))
        xyz[:, 0] = np.reshape(x, -1)
        xyz[:, 1] = np.reshape(y, -1)
        xyz[:, 2] = np.reshape(z, -1)
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(xyz)
        o3d.visualization.draw_geometries([pcd])

    def xytheta_xymove(self):
        r_max=3
        x = np.linspace(-3, 3, 401)
        mesh_x, mesh_y = np.meshgrid(x, x)
        region = mesh_x**2 + mesh_y**2 <= r_max**2
        circle_x = mesh_x[region]
        circle_y = mesh_y[region]
        z = np.arctan2(circle_y,circle_x)
        self.show_pointcloud(circle_x,circle_y,z)
    
    def xytheta2_xymove(self):
        r_max=3
        x = np.linspace(-3, 3, 401)
        mesh_x, mesh_y = np.meshgrid(x, x)
        mesh_r = np.sqrt( mesh_x**2 + mesh_y**2)
        region = mesh_r <= r_max
        circle_x = mesh_x[region]
        circle_y = mesh_y[region]
        z = np.arctan2(mesh_r[region],r_max)
        self.show_pointcloud(circle_x,circle_y,z)
    

        

if __name__ == "__main__":
    test = ShowPointCloud()
    test.xztheta_xmove()
    test.xytheta_xymove()
    test.xytheta2_xymove()

