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

pcd = o3d.io.read_point_cloud("001_rand.xyz")

o3d.visualization.draw_geometries([pcd])  
print("原始點雲:",pcd)

# downpcd = pcd.voxel_down_sample(voxel_size=4.5)
downpcd = o3d.geometry.voxel_down_sample(pcd ,voxel_size=45)
o3d.visualization.draw_geometries([downpcd])
print('downsample pointcloud',downpcd)
o3d.io.write_point_cloud('result_down.ply', downpcd)
pcd = o3d.io.read_point_cloud("result_down.ply")
#print(pcd)

#print("->正在估计法线并可视化...")
radius = 8   # 搜索半径
max_nn = 5
o3d.geometry.estimate_normals(pcd,search_param=o3d.geometry.KDTreeSearchParamKNN(20))
# pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius, max_nn))     # 执行法线估计
# pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamKNN(max_nn) )   # 执行法线估计


### #有三種方式
# o3d.geometry.KDTreeSearchParamKNN(knn=20)                        # 计算近邻的20个点
# o3d.geometry.KDTreeSearchParamRadius(radius=0.01)                # 计算指定半径内的点
# o3d.geometry.KDTreeSearchParamHybrid(radius=0.01, max_nn=20)
pcd.paint_uniform_color([1, 0.706, 0])#source 为黄色
# pcd[100].paint_uniform_color([0, 0.651, 0.929])#target 为蓝色

# np.asarray(pcd.colors)[3, :] = [0, 0, 1]   #single point  painting
totol_point = np.asarray(pcd.normals)[:,:].size/3

for i in range(0,int(totol_point)):
    if abs(np.asarray(pcd.normals)[i][1])  > 0.3:
        np.asarray(pcd.colors)[i, :] = [0, 0, 1]


o3d.geometry.orient_normals_towards_camera_location(pcd, camera_location=np.array([20., 20., 100.]))  #朝向某個方向

o3d.visualization.draw_geometries([pcd], window_name="result")


print("列印出 法向量")
print(np.asarray(pcd.normals)[:,:])

# print(type(np.asarray(pcd.normals)))   ##<class 'numpy.ndarray'>
print(np.asarray(pcd.normals)[1000][0])
print(np.asarray(pcd.normals)[1000])



print(type(pcd))
print(np.asarray(pcd.normals)[:,:].size)


print(type(pcd.normals))





