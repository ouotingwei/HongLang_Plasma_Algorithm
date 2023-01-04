/*

Version : V1
Deadline : 2023 / 01 / 16
Author : TingWei Ou
Discription : HongLang Project
*/
#include<iostream>
#include<pcl/io/pcd_io.h>
#include<pcl/io/pcd_io.h>
#include<pcl/io/ply_io.h>
#include<pcl/io/vtk_lib_io.h>
#include<pcl/point_cloud.h>
#include<pcl/console/parse.h>
#include<pcl/common/transforms.h>
#include<pcl/visualization/cloud_viewer.h>
#include<vtkVersion.h>
#include<vtkSTLReader.h>
#include<vtkTriangle.h>
#include<vtkTriangleFilter.h>
#include<vtkPolyDataMapper.h>
#include<Eigen/Dense>
#include<cmath>

using namespace std;
using namespace pcl;
using namespace Eigen;

class transfer_file{
	public:
		void stl_2_pcd(){

			pcl::PolygonMesh mesh;
			pcl::io::loadPolygonFile("001.stl", mesh);
			pcl::PointCloud<pcl::PointXYZ>::Ptr stl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
			pcl::fromPCLPointCloud2(mesh.cloud, *stl_cloud);
			pcl::io::savePCDFileASCII("001.pcd", *stl_cloud);

		}

		void pcd_2_WorkingPlane(){
			string file_name = "001.pcd";
			


		}

		void PathPlanning_2_ls(){

		}
	
	private:
};

class Planning{
	public:
		
};

int main(){
	float dim_of_tool;
	float overlap;

	transfer_file T;
	Planning P;

	cout<<"[!]diameter of tool (mm) : ";
	cin>>dim_of_tool;
	cout<<"[!]overlap (0~99%) : ";
	cin>>overlap;

	T.stl_2_pcd();
	T.pcd_2_WorkingPlane();

	return 0;
	
}