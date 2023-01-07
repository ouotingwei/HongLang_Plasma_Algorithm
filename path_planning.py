# Version : V1
# Deadline : 2023 / 01 / 16
# Author : TingWei Ou
# Discription : HongLang Project

import cv2
import numpy as np 
import glob
from tqdm import tqdm
import PIL.ExifTags
import PIL.Image
from matplotlib import pyplot as plt 
import open3d as o3d
import csv

# read .xyz file
pcd = o3d.io.read_point_cloud("test.xyz")

o3d.visualization.draw_geometries([pcd], window_name="test",
                                  point_show_normal=True,
                                  width=800, 
                                  height=600)  
print("origin : ",pcd)

downpcd = pcd.voxel_down_sample(voxel_size=10)
o3d.visualization.draw_geometries([downpcd])
print('downsample pointcloud',downpcd)
o3d.io.write_point_cloud('result_down.ply', downpcd)
pcd = o3d.io.read_point_cloud("result_down.ply")

radius = 8
max_nn = 5

pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius, max_nn))

totol_point = np.asarray(pcd.normals)[:,:].size/3

pcd.paint_uniform_color([1, 0.706, 0])

for i in range(0,int(totol_point)):
    if abs(np.asarray(pcd.normals)[i][1])  > 0.3:
        np.asarray(pcd.colors)[i, :] = [0, 0, 1]

o3d.geometry.PointCloud.orient_normals_towards_camera_location(pcd, camera_location=np.array([0.0, 0.0, 1000.0])) 
o3d.visualization.draw_geometries([pcd], window_name="result",
                                  point_show_normal=True,
                                  width=800,  
                                  height=600)  

