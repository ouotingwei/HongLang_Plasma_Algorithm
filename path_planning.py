# Version : V1
# Deadline : 2023 / 01 / 16
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

def xyz_2_point(diameter, overlap):
    # find the distance between two working path
    sample_dis = diameter * (1 - (overlap*0.01))

    # read .xyz file
    global pcd 
    pcd = o3d.io.read_point_cloud(file_name)

    #o3d.visualization.draw_geometries([pcd], window_name="test", point_show_normal=True)  
    #print("origin : ",pcd)

    downpcd = pcd.voxel_down_sample(voxel_size=sample_dis)
    #o3d.visualization.draw_geometries([downpcd])
    print('downsample pointcloud',downpcd)
    o3d.io.write_point_cloud('result_down.ply', downpcd)
    pcd = o3d.io.read_point_cloud("result_down.ply")

    radius = 50
    max_nn = 50

    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius, max_nn))
    totol_point = np.asarray(pcd.normals)[:,:].size/3
    pcd.paint_uniform_color([1, 0.706, 0])

    for i in range(0,int(totol_point)):
        if abs(np.asarray(pcd.normals)[i][1])  > 0.3:
            np.asarray(pcd.colors)[i, :] = [0, 0, 1]

    o3d.geometry.PointCloud.orient_normals_towards_camera_location(pcd, camera_location=np.array([0.0, 0.0, 100.0])) 
    o3d.visualization.draw_geometries([pcd], window_name="result", point_show_normal=True)  

def point_cloud_planning():
    point_arr = np.asarray(pcd.points)
    normal_arr = np.asarray(pcd.normals)
    
    n = len(point_arr)
    for i in range(n-2):
        for j in range(n-i-1):
            if point_arr[j][0] > point_arr[j+1][0]:
                point_arr[j][0], point_arr[j+1][0] = point_arr[j+1][0], point_arr[j][0]
                point_arr[j][1], point_arr[j+1][1] = point_arr[j+1][1], point_arr[j][1]
                point_arr[j][2], point_arr[j+1][2] = point_arr[j+1][2], point_arr[j][2]
                normal_arr[j][0], normal_arr[j+1][0] = normal_arr[j+1][0], normal_arr[j][0]
                normal_arr[j][1], normal_arr[j+1][1] = normal_arr[j+1][1], normal_arr[j][1]
                normal_arr[j][2], normal_arr[j+1][2] = normal_arr[j+1][2], normal_arr[j][2]
                
    # angle transform
    i = 0
    while i < len(point_arr):
        normal_x = math.acos(math.sqrt(math.pow(normal_arr[i][0], 2) + math.pow(normal_arr[i][2], 2)) / (pow(normal_arr[i][0], 2) + pow(normal_arr[i][1], 2) + pow(normal_arr[i][2], 2)))
        normal_x = rad2deg(normal_x)

        if normal_arr[i][1] > 0:
            normal_x = 360 - normal_x

        normal_y = math.atan(normal_arr[i][0] / normal_arr[i][2])
        normal_y = rad2deg(normal_y)

        if normal_y <0:
            normal_y = normal_y + 180

        if normal_arr[i][0] < 0:
            normal_y = normal_y + 180

        normal_z = 0

        normal_arr[i][0] = normal_x
        normal_arr[i][1] = normal_y
        normal_arr[i][2] = normal_z

        i = i + 1

    print(normal_arr)

    global waypoints 
    waypoints = []
    for i in range(0,len(point_arr)):
        tf = [500, 0, -270]
        waypoints.append(WayPoints(point_arr[i][0] + tf[0], point_arr[i][1] + tf[1], point_arr[i][2] + tf[2], normal_arr[i][0], normal_arr[i][1], normal_arr[i][2]))
    
    point_2_ls(output_file, waypoints)

def point_2_ls(file, waypoints):
    f = open(file, 'w')
    f.write("/PROG  PNS888\n")
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
        f.write("	UF : 0, UT : 1,		CONFIG : 'N U T, 0, 0, 0',\n")
        f.write("	X =  " + "{:.3f}".format(waypoints[i-1].x) + "  mm,	Y =   "+ "{:.3f}".format(waypoints[i-1].y) + "  mm,	Z =   "+ "{:.3f}".format(waypoints[i-1].z) + "  mm,\n")
        f.write("	W =  " + "{:.3f}".format(waypoints[i-1].W) + " deg,	P =   " + "{:.3f}".format(waypoints[i-1].P) + " deg,	R =   " + "{:.3f}".format(waypoints[i-1].R) + " deg\n")
        f.write("};\n")

    f.write("/END\n")
    f.close()

def rad2deg(rad):
    return rad*180/3.1415
    


def main():
    global file_name
    global output_file

    diameter = float(input("[Q]diameter (mm) : "))
    overlap = int(input("[Q]overlap (0~90%) : "))
    file_name = str(input("[Q]file name(.xyz) : "))
    output_file = str(input("[Q]output file name(.LS) : "))

    xyz_2_point(diameter, overlap)
    point_cloud_planning()



if __name__ == '__main__':
    main()






