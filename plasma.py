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
import  time


global FileName
global OutputFile
global waypoints
global PointArray
global NormalArray
waypoints = []
breakPoint = 30 # > breakPoint high wall ; < breakPoint low wall


class WayPoints:
    def __init__(self, x=0.000, y=0.000, z=0.000, W=0.000, P=0.000, R=0.000, V=100, C="CNT0"):
        self.x = x  # x
        self.y = y  # y
        self.z = z  # z
        self.W = W  # phi
        self.P = P  # theta
        self.R = R  # psi
        self.V = V  # velocity
        self.C = C  # continuity


def pointCloudProcess(diameter, overlap):
    # find the distance between two working path
    sample_dis = diameter * (1 - (overlap*0.01))

    # read .xyz file
    global pcd 
    pcd = o3d.io.read_point_cloud(FileName)

    #o3d.visualization.draw_geometries([pcd], window_name="test", point_show_normal=True)  
    #print("origin : ",pcd)

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


def sampleFromOriginPCD(diameter, overlap):
    # find the distance between two working path
    sample_dis = diameter * (1 - (overlap*0.01))

    # read .xyz file
    global pcd 
    pcd = o3d.io.read_point_cloud(FileName)

    #o3d.visualization.draw_geometries([pcd], window_name="test", point_show_normal=True)  
    #print("origin : ",pcd)

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


def lowWallPlanning():
    PointArray = np.asarray(pcd.points)
    NormalArray = np.asarray(pcd.normals)
    
    #ARRANGE IN ORDER X
    n = len(PointArray)
    for i in range(n-1):
        for j in range(n-i-1):
            if PointArray[j][0] > PointArray[j+1][0]:
                PointArray[j][0], PointArray[j+1][0] = PointArray[j+1][0], PointArray[j][0]
                PointArray[j][1], PointArray[j+1][1] = PointArray[j+1][1], PointArray[j][1]
                PointArray[j][2], PointArray[j+1][2] = PointArray[j+1][2], PointArray[j][2]
                NormalArray[j][0], NormalArray[j+1][0] = NormalArray[j+1][0], NormalArray[j][0]
                NormalArray[j][1], NormalArray[j+1][1] = NormalArray[j+1][1], NormalArray[j][1]
                NormalArray[j][2], NormalArray[j+1][2] = NormalArray[j+1][2], NormalArray[j][2]
    
    #print(point_arr)
    #print("")

    #INITALIZATION
    CountingArray = np.zeros(((len(PointArray) + 1, 8)), float)
    i = 0
    flag = 0
    time = 1
    CountingArray[0][3] = 1
    CountingArray[len(PointArray)-1][0] = PointArray[len(PointArray)-1][0]
    CountingArray[len(PointArray)-1][1] = PointArray[len(PointArray)-1][1]
    CountingArray[len(PointArray)-1][2] = PointArray[len(PointArray)-1][2]

    CountingArray[len(PointArray)-1][5] = NormalArray[len(PointArray)-1][0]
    CountingArray[len(PointArray)-1][6] = NormalArray[len(PointArray)-1][1]
    CountingArray[len(PointArray)-1][7] = NormalArray[len(PointArray)-1][2]

    CountingArray[len(PointArray)][3] = 1
    CountingArray[len(PointArray)][4] = 4

    #SORT
    while i < len(PointArray) - 1:
        CountingArray[i][0] = PointArray[i][0]
        CountingArray[i][1] = PointArray[i][1]
        CountingArray[i][2] = PointArray[i][2]
        CountingArray[i][5] = NormalArray[i][0]
        CountingArray[i][6] = NormalArray[i][1]
        CountingArray[i][7] = NormalArray[i][2]
        
        if round(PointArray[i+1][0]) != round(PointArray[i][0]) :
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
    while i < len(PointArray) :

        if CountingArray[i+1][3] == 1 :
            n = int(CountingArray[i][3])

            if (int(CountingArray[i][4]) % 2) == 0:
                cnt = 0
                TempArray = np.zeros(((n, 8)), float)

                while cnt < n:
                    TempArray[cnt][0] = CountingArray[i-n+cnt+1][0]
                    TempArray[cnt][1] = CountingArray[i-n+cnt+1][1]
                    TempArray[cnt][2] = CountingArray[i-n+cnt+1][2]
                    TempArray[cnt][5] = CountingArray[i-n+cnt+1][5]
                    TempArray[cnt][6] = CountingArray[i-n+cnt+1][6]
                    TempArray[cnt][7] = CountingArray[i-n+cnt+1][7]
                    cnt = cnt + 1           

                for temp in range(n-1):
                    for j in range(n-temp-1):
                        if TempArray[j][1] > TempArray[j+1][1]:
                            TempArray[j][0], TempArray[j+1][0] = TempArray[j+1][0], TempArray[j][0]
                            TempArray[j][1], TempArray[j+1][1] = TempArray[j+1][1], TempArray[j][1]
                            TempArray[j][2], TempArray[j+1][2] = TempArray[j+1][2], TempArray[j][2]
                            TempArray[j][5], TempArray[j+1][5] = TempArray[j+1][5], TempArray[j][5]
                            TempArray[j][6], TempArray[j+1][6] = TempArray[j+1][6], TempArray[j][6]
                            TempArray[j][7], TempArray[j+1][7] = TempArray[j+1][7], TempArray[j][7]     
                        
                cnt = 0
                while cnt < n:
                    CountingArray[i-n+cnt+1][0] = TempArray[cnt][0]
                    CountingArray[i-n+cnt+1][1] = TempArray[cnt][1]
                    CountingArray[i-n+cnt+1][2] = TempArray[cnt][2]
                    CountingArray[i-n+cnt+1][5] = TempArray[cnt][5]
                    CountingArray[i-n+cnt+1][6] = TempArray[cnt][6]
                    CountingArray[i-n+cnt+1][7] = TempArray[cnt][7]
                    cnt = cnt + 1

            elif (int(CountingArray[i][4]) % 2) == 1:
                cnt = 0
                TempArray = np.zeros(((n, 8)), float)

                while cnt < n:
                    TempArray[cnt][0] = CountingArray[i-n+cnt+1][0]
                    TempArray[cnt][1] = CountingArray[i-n+cnt+1][1]
                    TempArray[cnt][2] = CountingArray[i-n+cnt+1][2]
                    TempArray[cnt][5] = CountingArray[i-n+cnt+1][5]
                    TempArray[cnt][6] = CountingArray[i-n+cnt+1][6]
                    TempArray[cnt][7] = CountingArray[i-n+cnt+1][7]
                    cnt = cnt + 1  

                for temp in range(n-1):
                    for j in range(n-temp-1):
                        if TempArray[j][1] < TempArray[j+1][1]:
                            TempArray[j][0], TempArray[j+1][0] = TempArray[j+1][0], TempArray[j][0]
                            TempArray[j][1], TempArray[j+1][1] = TempArray[j+1][1], TempArray[j][1]
                            TempArray[j][2], TempArray[j+1][2] = TempArray[j+1][2], TempArray[j][2]
                            TempArray[j][5], TempArray[j+1][5] = TempArray[j+1][5], TempArray[j][5]
                            TempArray[j][6], TempArray[j+1][6] = TempArray[j+1][6], TempArray[j][6]
                            TempArray[j][7], TempArray[j+1][7] = TempArray[j+1][7], TempArray[j][7]
                        
                cnt = 0
                while cnt < n:
                    CountingArray[i-n+cnt+1][0] = TempArray[cnt][0]
                    CountingArray[i-n+cnt+1][1] = TempArray[cnt][1]
                    CountingArray[i-n+cnt+1][2] = TempArray[cnt][2]
                    CountingArray[i-n+cnt+1][5] = TempArray[cnt][5]
                    CountingArray[i-n+cnt+1][6] = TempArray[cnt][6]
                    CountingArray[i-n+cnt+1][7] = TempArray[cnt][7]
                    cnt = cnt + 1
 
        i = i + 1
    
    i = 0
    while i < len(PointArray):
        PointArray[i][0] = CountingArray[i][0]
        PointArray[i][1] = CountingArray[i][1] 
        PointArray[i][2] = CountingArray[i][2]
        NormalArray[i][0] = CountingArray[i][5] = 90
        NormalArray[i][1] = CountingArray[i][6] = 0
        NormalArray[i][2] = CountingArray[i][7] = 90

        i = i + 1
    
    # new 
    for i in range(0,len(PointArray)):
        theta = 0
        transition = [500 , -500, -270] 
        rotation = np.array([[math.cos(theta), -math.sin(theta), 0], [math.sin(theta), math.cos(theta), 0], [0,0,1]], float)
        position = np.matmul(PointArray[i], rotation) 
        position = position + transition
        waypoints.append(WayPoints(position[0], position[1], position[2], NormalArray[i][0], NormalArray[i][1], NormalArray[i][2]))
 

# !
def highWallPlanning_Wall():

    return 0

# !
def highWallPlanning_Bottom_Flat():
    PointArray = np.asarray(pcd.points)
    NormalArray = np.asarray(pcd.normals)
    
    
    
    i = 0
    size = 0
    while i < len(PointArray):
        if int(PointArray[i][2]) == 0 :
            size  = size + 1
        
        i = i + 1
    
    CountingArray = np.zeros(((len(size) + 1, 8)), float)
    i = 0
    cnt = 0
    while i < len(PointArray):
        if int(PointArray[i][2]) == 0:
            

        i = i + 1




    return 0


#new
def movePose():
    for i in range(0,len(PointArray)):
        theta = 60*3.1415/180
        transition = [280 , -400, -270] 
        rotation = np.array([[math.cos(theta), -math.sin(theta), 0], [math.sin(theta), math.cos(theta), 0], [0,0,1]], float)
        position = np.matmul(PointArray[i], rotation) 
        position = position + transition
        waypoints.append(WayPoints(position[0], position[1], position[2], NormalArray[i][0], NormalArray[i][1], NormalArray[i][2]))


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


def findMaxZ():
    PointArray = np.asarray(pcd.points)

    i = 0
    maxZ = 0
    while i < (len(PointArray) - 1):
        if PointArray[i][2] > PointArray[i+1][2]:
            maxZ = PointArray[i][2]

        i = i + 1
    
    return maxZ

    
def main():
    diameter = float(input("[Q]diameter (mm) : "))
    overlap = int(input("[Q]overlap (0~90%) : "))
    FileName = str(input("[Q]file name(.xyz) : "))
    
    pointCloudProcess(diameter, overlap)

    start = time.time()
    
    if findMaxZ() > breakPoint:
        # !
        OutputFile = "WALL.LS"
        highWallPlanning_Wall()
        writeLsFile(OutputFile, waypoints)

        OutputFile = "BOTTOM.LS"
        highWallPlanning_Bottom_Flat()
        writeLsFile(OutputFile, waypoints)
        

    else:
        OutputFile = "NORMAL.LS"
        lowWallPlanning()
        movePose()
        writeLsFile(OutputFile, waypoints)

    end = time.time()
    print("time used :", end - start, "sec")


if __name__ == '__main__':
    main()






