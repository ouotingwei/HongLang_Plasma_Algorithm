# Version : V2
# Deadline : 2023 / 02 / 10
# Author : TingWei Ou, PoLin Jiang
# Discription : HongLang Project
#!/usr/bin/env python3
# -*- coding: utf-8 -*-


import cv2
import numpy as np 
import glob
from tqdm import tqdm
import PIL.ExifTags
import PIL.Image
from matplotlib import pyplot as plt 
import open3d as o3d
import csv
import math
import time


class WayPoints:
    def __init__(self, x=0.000, y=0.000, z=0.000, W=0.000, P=0.000, R=0.000, V=100, C="CNT100"):
        self.x = x  # xs
        self.y = y  # y
        self.z = z  # z
        self.W = W  # phi
        self.P = P  # theta
        self.R = R  # psi
        self.V = V  # velocity
        self.C = C  # continuity


def pointCloudProcess_v1():
    # find the distance between two working path
    #sample_dis = diameter * (1 - (overlap*0.01))

    # read .xyz file
    global pcd 
    pcd = o3d.io.read_point_cloud(FileName)

    o3d.visualization.draw_geometries([pcd], window_name="test", point_show_normal=True)  
    print("origin : ",pcd)

    downpcd = pcd.voxel_down_sample(voxel_size=1)
    #o3d.visualization.draw_geometries([downpcd])
    print('downsample pointcloud',downpcd)
    o3d.io.write_point_cloud('result_down.ply', downpcd)
    pcd = o3d.io.read_point_cloud("result_down.ply")

    #modular design not yet
    radius = 50
    max_nn = 50

    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius, max_nn))
    totol_point = np.asarray(pcd.normals)[:,:].size/3
    pcd.paint_uniform_color([1, 0.706, 0])

    for i in range(0,int(totol_point)):
        if abs(np.asarray(pcd.normals)[i][1])  > 0.3:
            np.asarray(pcd.colors)[i, :] = [0, 0, 1]

    #modular design not yet
    o3d.geometry.PointCloud.orient_normals_towards_camera_location(pcd, camera_location=np.array([0.0, 0.0, 1000.])) 
    o3d.visualization.draw_geometries([pcd], window_name="result", point_show_normal=True)  
    
    return 0

def fingMaximumBondary(pcdSample_pre):
    global max_x
    global min_x 
    global max_y
    global min_y
    global max_z

    max_x = 0
    min_x = 0
    max_y = 0
    min_y = 0
    max_z = 0
    
    i = 0
    while i < len(pcdSample_pre) - 1:
        if pcdSample_pre[i][0] > max_x :
            max_x = pcdSample_pre[i][0]
        
        if pcdSample_pre[i][0] < min_x :
            min_x = pcdSample_pre[i][0]
            
        if pcdSample_pre[i][1] > max_y :
            max_y = pcdSample_pre[i][1]
            
        if pcdSample_pre[i][1] < min_y:
            min_y = pcdSample_pre[i][1]
        
        if pcdSample_pre[i][2] > max_z:
            max_z = pcdSample_pre[i][2]
        
        i = i + 1

    return 0 

def pointCloudSample(times):
    sampleRange  = 5
    pointCloud = np.asarray(pcd.points)
    fingMaximumBondary(pointCloud)

    pcdSample = np.zeros((10000, 4), float) # [x][y][z][write flag]

    x = max_x - min_x
    sample_x = x / times

    print(pointCloud)

    cnt = 0
    for i in range(len(pointCloud)):
        for j in range(times):
            temp = pointCloud[0][0] - (sample_x * j)
            temp = temp % sample_x

            if temp < sampleRange and temp > (-1 * sampleRange):
                pcdSample[i][0] = pointCloud[i][0]
                pcdSample[i][1] = pointCloud[i][1]
                pcdSample[i][2] = pointCloud[i][2]
                pcdSample[i][3] = 1

                cnt = cnt + 1
    
    i = 0
    while i < 10000:
        if pcdSample[i][3] == 1:
            print(pcdSample[i][0], pcdSample[i][1], pcdSample[i][2])

        i = i + 1


def main():
    global OutputFile
    global FileName
    OutputFile = "P004.LS"
    FileName = "004_rand.xyz"
    times = int(input("[Q]等分數 : "))

    pointCloudProcess_v1()
    pointCloudSample(times)


if __name__ == '__main__':
    main()