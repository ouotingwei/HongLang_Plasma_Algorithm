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
    
    return 0


def pointCloudSample():
    # read .xyz file
    global pcd
    global pcdSample
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
    o3d.geometry.PointCloud.orient_normals_towards_camera_location(pcd, camera_location=np.array([0.0, 0.0, 10.])) 
    o3d.visualization.draw_geometries([pcd], window_name="result", point_show_normal=True) 

    #sample
    points = np.asarray(pcd.points)
    normals = np.asarray(pcd.normals)
    pcdSample_bottom = np.zeros((len(points), 6), float) # [x][y][z][a][b][c]

    #bottom
    i = 0
    while i < len(pcdSample_bottom):
        pcdSample_bottom[i][0] = points[i][0]
        pcdSample_bottom[i][1] = points[i][1]
        pcdSample_bottom[i][2] = points[i][2]
        pcdSample_bottom[i][3] = normals[i][0]
        pcdSample_bottom[i][4] = normals[i][1]
        pcdSample_bottom[i][5] = normals[i][2]
        i = i + 1

    return 0


def backAndForth(Point, Normal):
   #ARRANGE IN ORDER X
    n = len(Point)
    for i in range(n-1):
        for j in range(n-i-1):
            if Point[j][0] > Point[j+1][0]:
                Point[j][0], Point[j+1][0] = Point[j+1][0], Point[j][0]
                Point[j][1], Point[j+1][1] = Point[j+1][1], Point[j][1]
                Point[j][2], Point[j+1][2] = Point[j+1][2], Point[j][2]
    
    #print(point_arr)
    #print("")

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

            if (int(CountingArray[i][4]) % 2) == 0:
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
    
    i = 0
    while i < len(Point):
        Point[i][0] = CountingArray[i][0]
        Point[i][1] = CountingArray[i][1] 
        Point[i][2] = CountingArray[i][2]

        i = i + 1

    # output ordered waypoints
    workingSpaceTF(Point, np.zeros(((len(Point), 3)), float))


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
                TempArray[n][0] = CountingArray[i-cnt+n+1][0]    #
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

    # normal proccessing
    # let end-effector keep 10mm from the wall
    Point, Parall = wallNormalProccessing(10, CountingArray)

    # output ordered waypoints
    workingSpaceTF(Point, Parall)
    
    
# !
def Wall(gate):
    PointArray = np.asarray(pcd.points)

    i = 0
    size = 0
    while i < len(PointArray):
        if int(PointArray[i][2]) > gate: #gate = 10
            size = size + 1
        
        i = i + 1
    
    Point_filter = np.zeros(((size + 1, 3)), float)

    i = 0
    cnt = 0
    while i < len(PointArray):
        if int(PointArray[i][2]) > gate:
            Point_filter[cnt][0] = PointArray[i][0]
            Point_filter[cnt][1] = PointArray[i][1]
            Point_filter[cnt][2] = PointArray[i][2]


            cnt = cnt + 1
            
        i = i + 1
    
    circularArrangement(Point_filter)
    return 0


# test not yet
def BottomFlat(gate):
    PointArray = np.asarray(pcd.points)
    NormalArray = np.asarray(pcd.normals)
    
    
    i = 0
    size = 0
    while i < len(PointArray):
        if int(PointArray[i][2]) < gate : # gate = 5
            size  = size + 1
        
        i = i + 1
    
    Point_filter = np.zeros(((size + 1, 3)), float)
    Normal_filter = np.zeros(((size + 1, 3)), float)

    i = 0
    cnt = 0
    while i < len(PointArray):
        if int(PointArray[i][2]) < gate:
            Point_filter[cnt][0] = PointArray[i][0]
            Point_filter[cnt][1] = PointArray[i][1]
            Point_filter[cnt][2] = PointArray[i][2]
            Normal_filter[cnt][0] = NormalArray[i][0]
            Normal_filter[cnt][1] = NormalArray[i][1]
            Normal_filter[cnt][2] = NormalArray[i][2]

            cnt = cnt + 1

        i = i + 1

    backAndForth(Point_filter, Normal_filter)
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
            Parall[i][0] = math.cos(slope)
            Parall[i][1] = 0.000
            Parall[i][2] = math.sin(slope)

        if(CountingArray[i][3] <= (180 - corner) and CountingArray[i][3] > corner ):
            Point[i][0] = Point[i][0] - d*math.sin(slope)/math.tan(CountingArray[i][3]*3.1415/180)
            Point[i][1] = Point[i][1] - d*math.sin(slope)
            Point[i][2] = Point[i][2] + d*math.cos(slope)
            Parall[i][0] = 0.000
            Parall[i][1] = math.cos(slope)
            Parall[i][2] = math.sin(slope)

        if(CountingArray[i][3] <= 180 and CountingArray[i][3] > (180 - corner) or  CountingArray[i][3] < (-180 + corner)  and CountingArray[i][3] >= -180):
            Point[i][0] = Point[i][0] + d*math.sin(slope)
            Point[i][1] = Point[i][1] + d*math.sin(slope)*math.tan(CountingArray[i][3]*3.1415/180)
            Point[i][2] = Point[i][2] + d*math.cos(slope)
            Parall[i][0] = -math.cos(slope)
            Parall[i][1] = 0.000
            Parall[i][2] = math.sin(slope)

        if(CountingArray[i][3] <= -corner and CountingArray[i][3] >= (-180 + corner)):
            Point[i][0] = Point[i][0] + d*math.sin(slope)/math.tan(CountingArray[i][3]*3.1415/180)
            Point[i][1] = Point[i][1] + d*math.sin(slope)
            Point[i][2] = Point[i][2] + d*math.cos(slope)
            Parall[i][0] = 0.000
            Parall[i][1] = -math.cos(slope)
            Parall[i][2] = math.sin(slope) 

    return Point, Parall
    

def workingSpaceTF(Position,Vector):

    n = len(Position)
    if(n!=len(Vector)):
        print("length error")
        exit() 

    global waypoints
    waypoints = []

    for i in range(0, n): 
        # transform points to workspace
        transition_p = [240 , -370, -270]
        theta = 60*(3.1415/180)
        rotation_matrix = np.array([[math.cos(theta), -math.sin(theta), 0], [math.sin(theta), math.cos(theta), 0], [0,0,1]], float) ## rotate about z axis
        Position_tf = np.matmul(Position[i], rotation_matrix) ## rotation
        Position_tf = Position_tf + transition_p ## transition

        # transform vector to workspace 
        transition_v = [90, 0, 90]
        Vector_tf = np.matmul(Vector[i], rotation_matrix) ## rotation
        phi = math.degrees(math.atan2(Vector_tf[1],Vector_tf[2]))  
        theta = math.degrees(math.atan2(Vector_tf[0],Vector_tf[2]))
        psi = 0.000

        if phi < -0.000 and phi > -0.0001: phi = 0.000 ## prevent the value of -0.000...
        if theta < -0.000 and theta > -0.0001: theta = 0.000
        
        Vector_tf = np.array([phi, theta, psi]) # transform euler vector to engle representation
        Vector_tf = Vector_tf + transition_v ## transition
        
        # output watpoints
        waypoints.append(WayPoints(Position_tf[0], Position_tf[1], Position_tf[2], Vector_tf[0], Vector_tf[1], Vector_tf[2]))
    
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
    gate = 5

    #diameter = float(input("[Q]diameter (mm) : "))
    diameter = 50
    #overlap = int(input("[Q]overlap (0~90%) : "))
    overlap = 50
    #FileName = str(input("[Q]file name(.xyz) : "))
    FileName = "002_rand.xyz"

    print("diameter = ", diameter)
    print("overlap = ", overlap)
    print("FileName = ", FileName)
    
    pointCloudProcess(diameter, overlap)
    #pointCloudSample(diameter, overlap)

    start = time.time()
    
    # !
    OutputFile = "WALL01.LS"
    Wall(gate)
    writeLsFile(OutputFile, waypoints)

    OutputFile = "BOTTOM01.LS"
    BottomFlat(gate)
    writeLsFile(OutputFile, waypoints)

    end = time.time()
    print("time used :", end - start, "sec")


def test():
    global FileName
    global OutputFile
    gate = 5

    #diameter = float(input("[Q]diameter (mm) : "))
    diameter = 40
    #overlap = int(input("[Q]overlap (0~90%) : "))
    overlap = 40
    #FileName = str(input("[Q]file name(.xyz) : "))
    FileName = "002_rand.xyz"
    pointCloudSample()


if __name__ == '__main__':
    #main()
    test()