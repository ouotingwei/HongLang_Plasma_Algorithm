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
    o3d.geometry.PointCloud.orient_normals_towards_camera_location(pcd, camera_location=np.array([0.0, 0.0, 1000000.])) 
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
    sampleRange  = 0.5
    pointCloud = np.asarray(pcd.points)
    NormalCloud = np.asarray(pcd.normals)
    fingMaximumBondary(pointCloud)

    pcdSample = np.zeros((10000, 7), float) # [x][y][z][a][b][c][write flag]

    x = max_x - min_x
    sample_x = x / times

    cnt = 0
    for i in range(len(pointCloud)):
        for j in range(times):
            temp = max_x - (j * sample_x)

            up = temp + sampleRange
            down = temp - sampleRange

            if pointCloud[i][0] < up and pointCloud[i][0] > down:
                pcdSample[cnt][0] = pointCloud[i][0]
                pcdSample[cnt][1] = pointCloud[i][1]
                pcdSample[cnt][2] = pointCloud[i][2]
                pcdSample[cnt][3] = NormalCloud[i][0]
                pcdSample[cnt][4] = NormalCloud[i][1]
                pcdSample[cnt][5] = NormalCloud[i][2]
                pcdSample[cnt][6] = 1

                cnt = cnt + 1

    i = 0
    size  = 0
    while i < 10000:
        if pcdSample[i][6] == 1:
            size = size + 1
        
        i = i + 1
    
    #print(size)
    pcdDownSample = np.zeros((size, 6), float) # [point][normal]

    i = 0 
    while i < size:
        pcdDownSample[i][0] = pcdSample[i][0]
        pcdDownSample[i][1] = pcdSample[i][1]
        pcdDownSample[i][2] = pcdSample[i][2]
        pcdDownSample[i][3] = pcdSample[i][3]
        pcdDownSample[i][4] = pcdSample[i][4]
        pcdDownSample[i][5] = pcdSample[i][5]

        i = i + 1

    #print(pcdDownSample)
    backAndForth(pcdDownSample)

    return 0
    

def backAndForth(Point):
   #ARRANGE IN ORDER X
    n = len(Point)
    for i in range(n-1):
        for j in range(n-i-1):
            if Point[j][0] > Point[j+1][0]:
                Point[j][0], Point[j+1][0] = Point[j+1][0], Point[j][0]
                Point[j][1], Point[j+1][1] = Point[j+1][1], Point[j][1]
                Point[j][2], Point[j+1][2] = Point[j+1][2], Point[j][2]
                Point[j][3], Point[j+1][3] = Point[j+1][3], Point[j][3]
                Point[j][4], Point[j+1][4] = Point[j+1][4], Point[j][4]
                Point[j][5], Point[j+1][5] = Point[j+1][5], Point[j][5]

    #INITALIZATION
    CountingArray = np.zeros(((len(Point) + 1, 8)), float)
    i = 0
    flag = 0
    time = 1
    CountingArray[0][3] = 1
    CountingArray[len(Point)-1][0] = Point[len(Point)-1][0]
    CountingArray[len(Point)-1][1] = Point[len(Point)-1][1]
    CountingArray[len(Point)-1][2] = Point[len(Point)-1][2]
    CountingArray[len(Point)-1][3] = Point[len(Point)-1][3]
    CountingArray[len(Point)-1][4] = Point[len(Point)-1][4]
    CountingArray[len(Point)-1][5] = Point[len(Point)-1][5]

    CountingArray[len(Point)][6] = 1
    CountingArray[len(Point)][7] = 4

    #SORT
    while i < len(Point) - 1:
        CountingArray[i][0] = Point[i][0]
        CountingArray[i][1] = Point[i][1]
        CountingArray[i][2] = Point[i][2]
        CountingArray[i][3] = Point[i][3]
        CountingArray[i][4] = Point[i][4]
        CountingArray[i][5] = Point[i][5]
        
        if round(Point[i+1][0]) != round(Point[i][0]) :
            flag = flag + 1
            CountingArray[i+1][7] = flag
            CountingArray[i+1][6] = time = 1
        else:
            CountingArray[i+1][7] = flag
            time = time + 1
            CountingArray[i+1][6] = time 

        i = i + 1

    #ARRANGE IN ORDER Y
    i = 0
    while i < len(Point) :

        if CountingArray[i+1][6] == 1 :
            n = int(CountingArray[i][6])

            if (int(CountingArray[i][7]) % 2) == 0 or int(CountingArray[i][7]) == 0:
                cnt = 0
                TempArray = np.zeros(((n, 8)), float)

                while cnt < n:
                    TempArray[cnt][0] = CountingArray[i-n+cnt+1][0]
                    TempArray[cnt][1] = CountingArray[i-n+cnt+1][1]
                    TempArray[cnt][2] = CountingArray[i-n+cnt+1][2]
                    TempArray[cnt][3] = CountingArray[i-n+cnt+1][3]
                    TempArray[cnt][4] = CountingArray[i-n+cnt+1][4]
                    TempArray[cnt][5] = CountingArray[i-n+cnt+1][5]
                    cnt = cnt + 1           

                for temp in range(n-1):
                    for j in range(n-temp-1):
                        if TempArray[j][1] > TempArray[j+1][1]:
                            TempArray[j][0], TempArray[j+1][0] = TempArray[j+1][0], TempArray[j][0]
                            TempArray[j][1], TempArray[j+1][1] = TempArray[j+1][1], TempArray[j][1]
                            TempArray[j][2], TempArray[j+1][2] = TempArray[j+1][2], TempArray[j][2]
                            TempArray[j][3], TempArray[j+1][3] = TempArray[j+1][3], TempArray[j][3]
                            TempArray[j][4], TempArray[j+1][4] = TempArray[j+1][4], TempArray[j][4]
                            TempArray[j][5], TempArray[j+1][5] = TempArray[j+1][5], TempArray[j][5]     
                        
                cnt = 0
                while cnt < n:
                    CountingArray[i-n+cnt+1][0] = TempArray[cnt][0]
                    CountingArray[i-n+cnt+1][1] = TempArray[cnt][1]
                    CountingArray[i-n+cnt+1][2] = TempArray[cnt][2]
                    CountingArray[i-n+cnt+1][3] = TempArray[cnt][3]
                    CountingArray[i-n+cnt+1][4] = TempArray[cnt][4]
                    CountingArray[i-n+cnt+1][5] = TempArray[cnt][5]
                    cnt = cnt + 1

            elif (int(CountingArray[i][7]) % 2) == 1:
                cnt = 0
                TempArray = np.zeros(((n, 8)), float)

                while cnt < n:
                    TempArray[cnt][0] = CountingArray[i-n+cnt+1][0]
                    TempArray[cnt][1] = CountingArray[i-n+cnt+1][1]
                    TempArray[cnt][2] = CountingArray[i-n+cnt+1][2]
                    TempArray[cnt][3] = CountingArray[i-n+cnt+1][3]
                    TempArray[cnt][4] = CountingArray[i-n+cnt+1][4]
                    TempArray[cnt][5] = CountingArray[i-n+cnt+1][5]
                    cnt = cnt + 1  

                for temp in range(n-1):
                    for j in range(n-temp-1):
                        if TempArray[j][1] < TempArray[j+1][1]:
                            TempArray[j][0], TempArray[j+1][0] = TempArray[j+1][0], TempArray[j][0]
                            TempArray[j][1], TempArray[j+1][1] = TempArray[j+1][1], TempArray[j][1]
                            TempArray[j][2], TempArray[j+1][2] = TempArray[j+1][2], TempArray[j][2]
                            TempArray[j][3], TempArray[j+1][3] = TempArray[j+1][3], TempArray[j][3]
                            TempArray[j][4], TempArray[j+1][4] = TempArray[j+1][4], TempArray[j][4]
                            TempArray[j][5], TempArray[j+1][5] = TempArray[j+1][5], TempArray[j][5]
                        
                cnt = 0
                while cnt < n:
                    CountingArray[i-n+cnt+1][0] = TempArray[cnt][0]
                    CountingArray[i-n+cnt+1][1] = TempArray[cnt][1]
                    CountingArray[i-n+cnt+1][2] = TempArray[cnt][2]
                    CountingArray[i-n+cnt+1][3] = TempArray[cnt][3]
                    CountingArray[i-n+cnt+1][4] = TempArray[cnt][4]
                    CountingArray[i-n+cnt+1][5] = TempArray[cnt][5]
                    cnt = cnt + 1
 
        i = i + 1
    
    PArray = np.zeros(((len(Point) + 2  , 3)), float)
    NArray = np.zeros(((len(Point) + 2  , 3)), float)

    i = 0
    while i < len(Point):
        PArray[i][0] = CountingArray[i][0] # - 150
        PArray[i][1] = CountingArray[i][1] 
        PArray[i][2] = CountingArray[i][2] + 20

        i = i + 1
    
    i = 0
    while i < len(Point):
        NArray[i][0] = CountingArray[i][3]
        NArray[i][1] = CountingArray[i][4] 
        NArray[i][2] = CountingArray[i][5]

        i = i + 1

    # output ordered waypoints
    workingSpaceTF(PArray, NArray)
    return 0


def workingSpaceTF(Position,Vector):

    n = len(Position)
    if(n!=len(Vector)):
        print("length error")
        exit() 

    global waypoints
    waypoints = []

    theta = 0*(3.1415/180)
    rotation_matrix = np.array([[math.cos(theta), -math.sin(theta), 0], [math.sin(theta), math.cos(theta), 0], [0,0,1]], float) ## rotate about z axis

    # transition_p = [350.000 , 0.000, -350.000]
    transition_p = [350.000 , 0.000, -325.827 + 16.000]
    # transition_v = [90, 0, 90-theta*(180/3.1415)]
    # transition_v = [90, 0, 90]
    transition_v = [-180, 0, 0]

    # initial position
    waypoints.append(WayPoints(transition_p[0], transition_p[1], transition_p[2] + 100, transition_v[0], transition_v[1], transition_v[2]))

    tmp_phi = 0
    for i in range(0, n-2): 
        # transform points to workspace     
        Position_tf = np.matmul(Position[i], rotation_matrix) ## rotation
        Position_tf = Position_tf + transition_p ## transition

        # print(Vector[i][0], Vector[i][1], Vector[i][2])
        # transform vector to workspace 
        phi  = math.atan2(Vector[i][1], Vector[i][2])*180/3.1415
        
        limit = 10
        if(phi > limit): phi = limit
        if(phi < -limit): phi = -limit
        # print(phi)
        '''
        if(i>0 and abs(phi - tmp_phi > 150)): 
            # print(phi, tmp_phi)
            # phi = tmp_phi
        tmp_phi = phi
        '''
        Vector_tf = np.array([-phi, 0, 0]) 
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


def main():
    global OutputFile
    global FileName
    OutputFile = "PNS004.LS"
    FileName = "input/004_rand.xyz"
    times = int(input("[Q]等分數 : "))

    start = time.time()

    pointCloudProcess_v1()
    pointCloudSample(times)
    writeLsFile("output/" + OutputFile, waypoints)

    end = time.time()
    print("time used :", end - start, "sec")


if __name__ == '__main__':
    main()