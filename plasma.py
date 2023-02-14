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


def pointCloudProcess_v1(diameter, overlap):
    # find the distance between two working path
    sample_dis = diameter * (1 - (overlap*0.01))

    # read .xyz file
    global pcd 
    pcd = o3d.io.read_point_cloud(FileName)

    o3d.visualization.draw_geometries([pcd], window_name="test", point_show_normal=True)  
    print("origin : ",pcd)

    downpcd = pcd.voxel_down_sample(voxel_size=sample_dis)
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
    o3d.geometry.PointCloud.orient_normals_towards_camera_location(pcd, camera_location=np.array([0.0, 0.0, 10.])) 
    o3d.visualization.draw_geometries([pcd], window_name="result", point_show_normal=True)  
    
    return 0

def pointCloudSampleWall(diameter):
    # read .xyz file
    global pcd
    global pcdSample
    pcd = o3d.io.read_point_cloud(FileName)

    #o3d.visualization.draw_geometries([pcd], window_name="test", point_show_normal=True)  
    #print("origin : ",pcd)

    downpcd = pcd.voxel_down_sample(voxel_size=1) 
    #o3d.visualization.draw_geometries([downpcd])
    print('downsample pointcloud',downpcd)
    o3d.io.write_point_cloud('result_down.ply', downpcd)
    pcd = o3d.io.read_point_cloud("result_down.ply")

    #### scall
    global pcd_small_size
    max_x, max_y = findMaxXY()
    w = max_x*2
    h = max_y*2
    scale_size = ((w-7)*(h-10))/(w*h)
    pcd_small_size = pcd.scale(scale_size, (0, 0, 0))
    ####

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
    #o3d.geometry.PointCloud.orient_normals_towards_camera_location(pcd, camera_location=np.array([0.0, 0.0, 10.])) 
    #o3d.visualization.draw_geometries([pcd], window_name="result", point_show_normal=True) 

    #sample
    points = np.asarray(pcd.points)
    normals = np.asarray(pcd.normals)
    pcdSample_pre = np.zeros((len(points), 7), float) # [x][y][z][a][b][c][ = 1 -> base , = 0 -> wall ]

    #sample
    i = 0
    while i < len(pcdSample_pre):
        pcdSample_pre[i][0] = points[i][0]
        pcdSample_pre[i][1] = points[i][1]
        pcdSample_pre[i][2] = points[i][2]
        pcdSample_pre[i][3] = normals[i][0]
        pcdSample_pre[i][4] = normals[i][1]
        pcdSample_pre[i][5] = normals[i][2]
        
        i = i + 1

    fingMaximumBondary(pcdSample_pre, diameter)
    
    global R
    R = 3
    
    x = max_x - min_x
    y = max_y - min_y
    z = max_z
    
    #times_x = 10
    #times_y = 14
    #times_z = 4
    
    sample_x = x / times_x
    sample_y = y / (times_y - 1)
    sample_z = z / times_z

    filter = 0.5
    
    #flag pcdSsample[i][6] -> 0 & sample x & y
    i = 0
    filterCNT = 0

    #wall
    while i < len(pcdSample_pre):
        pcdSample_pre[i][6] = 0

        '''
        if pcdSample_pre[i][0] % sample_x < filter and pcdSample_pre[i][0] % sample_x > -1 * filter and pcdSample_pre[i][1] % sample_y < filter and pcdSample_pre[i][1] % sample_y > -1 * filter and pcdSample_pre[i][2] < R:
            pcdSample_pre[i][6] = 1
            filterCNT = filterCNT + 1
        '''
        
        if pcdSample_pre[i][2] % sample_z < filter and pcdSample_pre[i][2] % sample_z > -1 * filter and pcdSample_pre[i][2] > R:
            pcdSample_pre[i][6] = 1
            filterCNT = filterCNT + 1

        i = i + 1

    #bottom 
    max_x
    bottomDot = np.zeros(( (times_x + 1) * (times_y), 2), float)

    i = 0
    timesX = 0
    timesY = 0
    while i < len(bottomDot):
        if  (i + 1) % times_y != 0:
            bottomDot[i][0] = max_x - timesX * sample_x
            bottomDot[i][1] = max_y - timesY * sample_y
            timesY = timesY + 1

        if  (i + 1) % times_y == 0:
            bottomDot[i][0] = max_x - timesX * sample_x
            bottomDot[i][1] = max_y - timesY * sample_y
            timesX = timesX + 1
            timesY = 0
    
        i = i + 1

    pcdSample = np.zeros((filterCNT + ((times_x + 1) * times_y), 6), float)

    #print(bottomDot)

    i = 0
    pcdCNT = 0
    while i < len(pcdSample_pre):
        if pcdSample_pre[i][6] == 1:
            pcdSample[pcdCNT][0] = pcdSample_pre[i][0]
            pcdSample[pcdCNT][1] = pcdSample_pre[i][1]
            pcdSample[pcdCNT][2] = pcdSample_pre[i][2]
            pcdSample[pcdCNT][3] = pcdSample_pre[i][3]
            pcdSample[pcdCNT][4] = pcdSample_pre[i][4]
            pcdSample[pcdCNT][5] = pcdSample_pre[i][5]
            
            pcdCNT = pcdCNT + 1
        
        i = i + 1

    i = 0
    while i < (len(bottomDot)):
        pcdSample[pcdCNT][0] = bottomDot[i][0]
        pcdSample[pcdCNT][1] = bottomDot[i][1]
        pcdSample[pcdCNT][2] = 0

        i = i + 1
        pcdCNT = pcdCNT + 1
        
    return 0


def pointCloudSampleBot(diameter):
    # read .xyz file
    global pcd
    global pcdSample
    pcd = o3d.io.read_point_cloud(FileName)

    #o3d.visualization.draw_geometries([pcd], window_name="test", point_show_normal=True)  
    #print("origin : ",pcd)

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
    #o3d.geometry.PointCloud.orient_normals_towards_camera_location(pcd, camera_location=np.array([0.0, 0.0, 10.])) 
    #o3d.visualization.draw_geometries([pcd], window_name="result", point_show_normal=True) 

    #sample
    points = np.asarray(pcd.points)
    normals = np.asarray(pcd.normals)
    pcdSample_pre = np.zeros((len(points), 7), float) # [x][y][z][a][b][c][ = 1 -> base , = 0 -> wall ]

    #sample
    i = 0
    while i < len(pcdSample_pre):
        pcdSample_pre[i][0] = points[i][0]
        pcdSample_pre[i][1] = points[i][1]
        pcdSample_pre[i][2] = points[i][2]
        pcdSample_pre[i][3] = normals[i][0]
        pcdSample_pre[i][4] = normals[i][1]
        pcdSample_pre[i][5] = normals[i][2]
        
        i = i + 1

    fingMaximumBondary(pcdSample_pre, diameter)
    
    global R
    R = 5
    
    x = max_x - min_x
    y = max_y - min_y
    z = max_z
    
    #times_x = 10
    #times_y = 14
    #times_z = 4
    
    sample_x = x / times_x
    sample_y = y / (times_y - 1)
    sample_z = z / times_z

    filter = 0.5
    
    #flag pcdSsample[i][6] -> 0 & sample x & y
    i = 0
    filterCNT = 0

    #wall
    while i < len(pcdSample_pre):
        pcdSample_pre[i][6] = 0
        
        if pcdSample_pre[i][2] % sample_z < filter and pcdSample_pre[i][2] % sample_z > -1 * filter and pcdSample_pre[i][2] > R:
            pcdSample_pre[i][6] = 1
            filterCNT = filterCNT + 1

        i = i + 1

    #bottom 
    max_x
    bottomDot = np.zeros(( (times_x + 1) * (times_y), 2), float)

    i = 0
    timesX = 0
    timesY = 0
    while i < len(bottomDot):
        if  (i + 1) % times_y != 0:
            bottomDot[i][0] = max_x - timesX * sample_x
            bottomDot[i][1] = max_y - timesY * sample_y
            timesY = timesY + 1

        if  (i + 1) % times_y == 0:
            bottomDot[i][0] = max_x - timesX * sample_x
            bottomDot[i][1] = max_y - timesY * sample_y
            timesX = timesX + 1
            timesY = 0
    
        i = i + 1

    pcdSample = np.zeros((filterCNT + ((times_x + 1) * times_y), 6), float)

    i = 0
    pcdCNT = 0
    while i < len(pcdSample_pre):
        if pcdSample_pre[i][6] == 1:
            pcdSample[pcdCNT][0] = pcdSample_pre[i][0]
            pcdSample[pcdCNT][1] = pcdSample_pre[i][1]
            pcdSample[pcdCNT][2] = pcdSample_pre[i][2]
            pcdSample[pcdCNT][3] = pcdSample_pre[i][3]
            pcdSample[pcdCNT][4] = pcdSample_pre[i][4]
            pcdSample[pcdCNT][5] = pcdSample_pre[i][5]
            
            pcdCNT = pcdCNT + 1
        
        i = i + 1

    i = 0
    while i < (len(bottomDot)):
        pcdSample[pcdCNT][0] = bottomDot[i][0]
        pcdSample[pcdCNT][1] = bottomDot[i][1]
        pcdSample[pcdCNT][2] = 0

        i = i + 1
        pcdCNT = pcdCNT + 1

    return 0

def fingMaximumBondary(pcdSample_pre, diameter):
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
    
    max_x = max_x - (diameter / 2)
    min_x = min_x + (diameter / 2)
    max_y = max_y - (diameter / 2)
    min_y = min_y + (diameter / 2)

    return 0 
    

#add two point
def backAndForth(Point):
   #ARRANGE IN ORDER X
    n = len(Point)
    for i in range(n-1):
        for j in range(n-i-1):
            if Point[j][0] > Point[j+1][0]:
                Point[j][0], Point[j+1][0] = Point[j+1][0], Point[j][0]
                Point[j][1], Point[j+1][1] = Point[j+1][1], Point[j][1]

    #INITALIZATION
    CountingArray = np.zeros(((len(Point) + 1, 5)), float)
    i = 0
    flag = 0
    time = 1
    CountingArray[0][3] = 1
    CountingArray[len(Point)-1][0] = Point[len(Point)-1][0]
    CountingArray[len(Point)-1][1] = Point[len(Point)-1][1]
    CountingArray[len(Point)-1][2] = Point[len(Point)-1][2]

    CountingArray[len(Point)][3] = 1
    CountingArray[len(Point)][4] = 4

    #SORT
    while i < len(Point) - 1:
        CountingArray[i][0] = Point[i][0]
        CountingArray[i][1] = Point[i][1]
        CountingArray[i][2] = Point[i][2]
        
        if round(Point[i+1][0]) != round(Point[i][0]) :
            flag = flag + 1
            CountingArray[i+1][4] = flag
            CountingArray[i+1][3] = time = 1
        else:
            CountingArray[i+1][4] = flag
            time = time + 1
            CountingArray[i+1][3] = time 

        i = i + 1

    #ARRANGE IN ORDER Y
    i = 0
    while i < len(Point) :

        if CountingArray[i+1][3] == 1 :
            n = int(CountingArray[i][3])

            if (int(CountingArray[i][4]) % 2) == 0 or int(CountingArray[i][4]) == 0:
                cnt = 0
                TempArray = np.zeros(((n, 5)), float)

                while cnt < n:
                    TempArray[cnt][0] = CountingArray[i-n+cnt+1][0]
                    TempArray[cnt][1] = CountingArray[i-n+cnt+1][1]
                    TempArray[cnt][2] = CountingArray[i-n+cnt+1][2]
                    cnt = cnt + 1           

                for temp in range(n-1):
                    for j in range(n-temp-1):
                        if TempArray[j][1] > TempArray[j+1][1]:
                            TempArray[j][0], TempArray[j+1][0] = TempArray[j+1][0], TempArray[j][0]
                            TempArray[j][1], TempArray[j+1][1] = TempArray[j+1][1], TempArray[j][1]
                            TempArray[j][2], TempArray[j+1][2] = TempArray[j+1][2], TempArray[j][2]     
                        
                cnt = 0
                while cnt < n:
                    CountingArray[i-n+cnt+1][0] = TempArray[cnt][0]
                    CountingArray[i-n+cnt+1][1] = TempArray[cnt][1]
                    CountingArray[i-n+cnt+1][2] = TempArray[cnt][2]
                    cnt = cnt + 1

            elif (int(CountingArray[i][4]) % 2) == 1:
                cnt = 0
                TempArray = np.zeros(((n, 5)), float)

                while cnt < n:
                    TempArray[cnt][0] = CountingArray[i-n+cnt+1][0]
                    TempArray[cnt][1] = CountingArray[i-n+cnt+1][1]
                    TempArray[cnt][2] = CountingArray[i-n+cnt+1][2]
                    cnt = cnt + 1  

                for temp in range(n-1):
                    for j in range(n-temp-1):
                        if TempArray[j][1] < TempArray[j+1][1]:
                            TempArray[j][0], TempArray[j+1][0] = TempArray[j+1][0], TempArray[j][0]
                            TempArray[j][1], TempArray[j+1][1] = TempArray[j+1][1], TempArray[j][1]
                            TempArray[j][2], TempArray[j+1][2] = TempArray[j+1][2], TempArray[j][2]
                        
                cnt = 0
                while cnt < n:
                    CountingArray[i-n+cnt+1][0] = TempArray[cnt][0]
                    CountingArray[i-n+cnt+1][1] = TempArray[cnt][1]
                    CountingArray[i-n+cnt+1][2] = TempArray[cnt][2]
                    cnt = cnt + 1
 
        i = i + 1
    
    PArray = np.zeros(((len(Point) + 2  , 3)), float)

    i = 0
    while i < len(Point):
        PArray[i][0] = CountingArray[i][0]
        PArray[i][1] = CountingArray[i][1] 
        PArray[i][2] = CountingArray[i][2] + 20

        i = i + 1

    # output ordered waypoints
    workingSpaceTF(PArray, np.zeros(((len(Point) + 2, 3)), float))


def circularArrangement(Point):
    filter = 2

    n = len(Point)
    for i in range(n-1):
        for j in range(n-i-2):
            if Point[j][2] > Point[j+1][2]:
                Point[j][0], Point[j+1][0] = Point[j+1][0], Point[j][0]
                Point[j][1], Point[j+1][1] = Point[j+1][1], Point[j][1]
                Point[j][2], Point[j+1][2] = Point[j+1][2], Point[j][2]

    CountingArray = np.zeros(((len(Point)+2 , 5)), float)

    i = 0
    while i < len(Point) - 1:
        CountingArray[i][0] = Point[i][0]
        CountingArray[i][1] = Point[i][1]
        CountingArray[i][2] = Point[i][2]

        x = Point[i][0]
        y = Point[i][1]

        CountingArray[i][3] = math.atan2(y, x)*180/3.1415
        if CountingArray[i][3] > 360:
            CountingArray[i][3] = CountingArray[i][3] - 360

        i = i + 1

    i = 0
    while i < len(Point) - 1:
        high = CountingArray[i][2] + filter
        low = CountingArray[i][2] - filter

        if CountingArray[i+1][2] > low and CountingArray[i+1][2] < high:
            CountingArray[i+1][4] = 0
        else:
            CountingArray[i+1][4] = 1

        i = i + 1
    
    i = 0
    cnt = 1
    while i < len(Point):
        n = len(Point)

        if CountingArray[i+1][4] != 1:
            cnt = cnt + 1

        else:
            TempArray = np.zeros(((n, 5)), float)

            n = 0
            while n < cnt:
                TempArray[n][0] = CountingArray[i-cnt+n+1][0]    
                TempArray[n][1] = CountingArray[i-cnt+n+1][1]
                TempArray[n][2] = CountingArray[i-cnt+n+1][2]
                TempArray[n][3] = CountingArray[i-cnt+n+1][3]
                n = n + 1           

            for temp in range(n-1):
                for j in range(n-temp-1):
                    if TempArray[j][3] > TempArray[j+1][3]:
                        TempArray[j][0], TempArray[j+1][0] = TempArray[j+1][0], TempArray[j][0]
                        TempArray[j][1], TempArray[j+1][1] = TempArray[j+1][1], TempArray[j][1]
                        TempArray[j][2], TempArray[j+1][2] = TempArray[j+1][2], TempArray[j][2]
                        TempArray[j][3], TempArray[j+1][3] = TempArray[j+1][3], TempArray[j][3]
                          
            n = 0
            while n < cnt:
                CountingArray[i-cnt+n+1][0] = TempArray[n][0]
                CountingArray[i-cnt+n+1][1] = TempArray[n][1]
                CountingArray[i-cnt+n+1][2] = TempArray[n][2]
                CountingArray[i-cnt+n+1][3] = TempArray[n][3]
                n = n + 1 

            cnt = 1

        i = i + 1

    mark = 0
    while CountingArray[mark][0] != 0 and CountingArray[mark][1] != 0:
        mark = mark + 1

    Array = np.zeros(((len(Point)  , 3)), float)

    i = 0
    cnt = 0
    while i < len(Array):
        if i != mark:
            Array[i][0] = CountingArray[cnt][0]
            Array[i][1] = CountingArray[cnt][1]
            Array[i][2] = CountingArray[cnt][2]
            
            i = i + 1
            cnt = cnt + 1
        
        else:
            cnt = cnt + 1
            Array[i][0] = CountingArray[cnt][0]
            Array[i][1] = CountingArray[cnt][1]
            Array[i][2] = CountingArray[cnt][2]

            i = i + 1
    
    i = 0
    while i < len(Point):
        Point[i][0] = Array[i][0]
        Point[i][1] = Array[i][1] 
        Point[i][2] = Array[i][2] + test_z

        i = i + 1
    
    workingSpaceTF(Point, np.zeros(((len(Point) , 3)), float))
    
    
# !
def Wall(gate):
    #PointArray = np.asarray(pcd.points)
    PointArray = pcdSample

    i = 0
    size = 0
    while i < len(PointArray):
        if int(PointArray[i][2]) > R: #gate = 10
            size = size + 1
        
        i = i + 1
    
    Point_filter = np.zeros(((size , 3)), float)

    i = 0
    cnt = 0
    while i < len(PointArray):
        if int(PointArray[i][2]) > R:
            Point_filter[cnt][0] = PointArray[i][0]
            Point_filter[cnt][1] = PointArray[i][1]
            Point_filter[cnt][2] = PointArray[i][2]

            cnt = cnt + 1
            
        i = i + 1
    
    circularArrangement(Point_filter)
    return 0


# test not yet
def BottomFlat(gate):
    #PointArray = np.asarray(pcd.points)
    #NormalArray = np.asarray(pcd.normals)
    PointArray = np.zeros(((len(pcdSample), 3)), float)

    i = 0
    while i < len(pcdSample):
        PointArray[i][0] = pcdSample[i][0]
        PointArray[i][1] = pcdSample[i][1]
        PointArray[i][2] = pcdSample[i][2]
        
        i = i + 1
    
    i = 0
    size = 0
    while i < len(PointArray):
        if int(PointArray[i][2]) < R:
            size  = size + 1
        
        i = i + 1
    
    Point_filter = np.zeros(((size, 3)), float)

    i = 0
    cnt = 0
    while i < len(PointArray):
        if int(PointArray[i][2]) < R:
            Point_filter[cnt][0] = PointArray[i][0]
            Point_filter[cnt][1] = PointArray[i][1]
            Point_filter[cnt][2] = PointArray[i][2]

            cnt = cnt + 1

        i = i + 1

    backAndForth(Point_filter)

    return 0


def wallNormalProccessing(d, CountingArray):
    Point = np.zeros(((len(CountingArray)-3, 3)), float)
    Parall = np.zeros(((len(CountingArray)-3, 3)), float)

    # initialize Point
    i = 0
    while i < len(Point):
        Point[i][0] = CountingArray[i][0]
        Point[i][1] = CountingArray[i][1] 
        Point[i][2] = CountingArray[i][2] 

        i = i + 1

    # find the slope of wall
    slope = 0
    cnt = 1
    for i in range(len(Point)):
        Closest_Point = np.array([100, 100, 100], float)
        for j in range(len(Point)):
            if((Point[j][2] > Point[i][2] + 5 or Point[j][2] < Point[i][2] - 5) and (getDistance(Point[j], Point[i]) < getDistance(Closest_Point, Point[i]))):
                    Closest_Point = Point[j]
        
        delta_x = abs(Closest_Point[0] - Point[i][0])
        delta_y = abs(Closest_Point[1] - Point[i][1])
        delta_z = abs(Closest_Point[2] - Point[i][2])
        new_slope = math.atan2(delta_z**2, (delta_x**2 + delta_y**2)**0.5) ##\\*180/3.1415

        if(i==0): slope += new_slope
        if(i>0 and new_slope>(slope/cnt-5) and new_slope<(slope/cnt+5)):
            slope += new_slope
            cnt  += 1
    
    slope /= cnt

    # let end_effector keep a distance d and be parallel to the wall
    max_x, max_y = findMaxXY()
    corner = math.atan2(max_y, max_x)*180/3.1415

    for i in range(len(Point)-1):
        if(CountingArray[i][3] <= corner and CountingArray[i][3] > -corner ):
            Point[i][0] = Point[i][0] - d*math.sin(slope)
            Point[i][1] = Point[i][1] - d*math.sin(slope)*math.tan(CountingArray[i][3]*3.1415/180) 
            Point[i][2] = Point[i][2] + d*math.cos(slope)
            Parall[i][0] = 0.000
            Parall[i][1] = 90 - slope*180/3.1415
            Parall[i][2] = 0.000

        if(CountingArray[i][3] <= (180 - corner) and CountingArray[i][3] > corner ):
            Point[i][0] = Point[i][0] - d*math.sin(slope)/math.tan(CountingArray[i][3]*3.1415/180)
            Point[i][1] = Point[i][1] - d*math.sin(slope)
            Point[i][2] = Point[i][2] + d*math.cos(slope)
            Parall[i][0] = - (90 - slope*180/3.1415)
            Parall[i][1] = 0.000
            Parall[i][2] = 0.000

        if(CountingArray[i][3] <= 180 and CountingArray[i][3] > (180 - corner) or  CountingArray[i][3] < (-180 + corner)  and CountingArray[i][3] >= -180):
            Point[i][0] = Point[i][0] + d*math.sin(slope)
            Point[i][1] = Point[i][1] + d*math.sin(slope)*math.tan(CountingArray[i][3]*3.1415/180)
            Point[i][2] = Point[i][2] + d*math.cos(slope)
            Parall[i][0] = 0.000
            Parall[i][1] = - (90 - slope*180/3.1415)
            Parall[i][2] = 0.000

        if(CountingArray[i][3] <= -corner and CountingArray[i][3] >= (-180 + corner)):
            Point[i][0] = Point[i][0] + d*math.sin(slope)/math.tan(CountingArray[i][3]*3.1415/180)
            Point[i][1] = Point[i][1] + d*math.sin(slope)
            Point[i][2] = Point[i][2] + d*math.cos(slope)
            Parall[i][0] = 90 - slope*180/3.1415
            Parall[i][1] = 0.000
            Parall[i][2] = 0.000

    return Point, Parall
    

def workingSpaceTF(Position,Vector):

    n = len(Position)
    if(n!=len(Vector)):
        print("length error")
        exit() 

    global waypoints
    waypoints = []

    theta = 60*(3.1415/180)
    rotation_matrix = np.array([[math.cos(theta), -math.sin(theta), 0], [math.sin(theta), math.cos(theta), 0], [0,0,1]], float) ## rotate about z axis

    transition_p = [210.000 , -375.000, -350.000]
    # transition_v = [90, 0, 90-theta*(180/3.1415)]
    transition_v = [90, 0, 90]

    # initial position
    waypoints.append(WayPoints(transition_p[0], transition_p[1], transition_p[2] + 100, transition_v[0], transition_v[1], transition_v[2]))

    for i in range(0, n-2): 
        # transform points to workspace     
        Position_tf = np.matmul(Position[i], rotation_matrix) ## rotation
        Position_tf = Position_tf + transition_p ## transition

        # transform vector to workspace 
        Vector_tf = np.array([Vector[i][0], Vector[i][1], Vector[i][2]]) 
        Vector_tf = Vector_tf + transition_v ## transition
        
        # output watpoints
        waypoints.append(WayPoints(Position_tf[0], Position_tf[1], Position_tf[2], Vector_tf[0], Vector_tf[1], Vector_tf[2]))
    
    # end position
    waypoints.append(WayPoints(transition_p[0], transition_p[1], transition_p[2] + 100, transition_v[0], transition_v[1], transition_v[2]))

    return 0


def writeLsFile(file, waypoints):
    f = open(file, 'w')
    f.write("/PROG  "+OutputFile+"\n")
    f.write("/ATTR\n")
    f.write("OWNER       = MNEDITOR;\n")
    f.write("COMMENT     = \"\";\n")
    f.write("PROG_SIZE   = 636;\n")
    f.write("CREATE      = DATE 23-01-07  TIME 11:59:14;\n")
    f.write("MODIFIED    = DATE 23-01-07  TIME 12:02:18;\n")
    f.write("FILE_NAME   = ;\n")
    f.write("VERSION     = 0;\n")
    f.write("LINE_COUNT  = 4;\n")
    f.write("MEMORY_SIZE = 992;\n")
    f.write("PROTECT     = READ_WRITE;\n")
    f.write("TCD:  STACK_SIZE	= 0,\n")
    f.write("      TASK_PRIORITY	= 50,\n")
    f.write("      TIME_SLICE	= 0,\n")
    f.write("      BUSY_LAMP_OFF	= 0,\n")
    f.write("      ABORT_REQUEST	= 0,\n")
    f.write("      PAUSE_REQUEST	= 0;\n")
    f.write("DEFAULT_GROUP	= 1,*,*,*,*;\n")
    f.write("CONTROL_CODE	= 00000000 00000000;\n")

    f.write("/MN\n")
    f.write("   1:J P[1] 100% FINE    ;\n")
    for i in range(2,len(waypoints)+1):
        f.write("   " + str(i) + ":L P[" + str(i) + "] " + str(waypoints[i-1].V) + "mm/sec " + waypoints[i-1].C + "    ;" + "\n")

    f.write("/POS\n")
    for i in range(1,len(waypoints)+1):
        f.write("P[" + str(i) + "]{\n")
        f.write("   GP1:\n")
        f.write("	UF : 0, UT : 6,		CONFIG : 'N U T, 0, 0, 0',\n")
        f.write("	X =  " + "{:.3f}".format(waypoints[i-1].x) + "  mm,	Y =   "+ "{:.3f}".format(waypoints[i-1].y) + "  mm,	Z =   "+ "{:.3f}".format(waypoints[i-1].z) + "  mm,\n")
        f.write("	W =  " + "{:.3f}".format(waypoints[i-1].W) + " deg,	P =   " + "{:.3f}".format(waypoints[i-1].P) + " deg,	R =   " + "{:.3f}".format(waypoints[i-1].R) + " deg\n")
        f.write("};\n")

    f.write("/END\n")
    f.close()
    return 0


def getDistance(vector_1,vector_2):
    return ((vector_1[0] - vector_2[0])**2 + (vector_1[1] - vector_2[1])**2 + (vector_1[2] - vector_2[2])**2)**0.5


def findMaxXY():
    PointArray = np.asarray(pcd.points)
    maxX = 0
    maxY = 0
    
    for i in range(len(PointArray) - 1):
        if PointArray[i][0] > maxX:
            maxX = PointArray[i][0]
        if PointArray[i][1] > maxY:
            maxY = PointArray[i][1]
    
    return maxX, maxY

    
def main():
    global FileName
    global OutputFile
    global times_x 
    global times_y
    global times_z
    global test_z 

    test_z = 75
    gate = 5
    times_y = 2
    diameter = 50

    model = str(input("[Q]model : "))
    times_x = int(input("[Q]times_x : "))
    times_z = int(input("[Q]times_z : "))
    
    FileName = model + "_rand.xyz"
    print("FileName = ", FileName)
    
    start = time.time()
    
    pointCloudSampleWall(diameter)
    OutputFile = "W" + model + ".LS"
    Wall(gate)
    writeLsFile(OutputFile, waypoints)

    pointCloudSampleBot(diameter)

    OutputFile = "B" + model + ".LS"
    BottomFlat(gate)
    writeLsFile(OutputFile, waypoints)

    end = time.time()
    print("time used :", end - start, "sec")


if __name__ == '__main__':
    main()