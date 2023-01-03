#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>
#include <vtkVersion.h>
#include <vtkOBJReader.h>
#include <vtkTriangle.h>
#include <vtkTriangleFilter.h>
#include <vtkPolyDataMapper.h>

using namespace std;
using namespace pcl;
int main()
{

	vtkSmartPointer<vtkOBJReader> reader = vtkSmartPointer<vtkOBJReader>::New();
	reader-> SetFileName("plate.obj");
	reader-> Update();
	vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
	polydata = reader->GetOutput();
	polydata->GetNumberOfPoints();
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());

	pcl::io::vtkPolyDataToPointCloud(polydata, *cloud);

	pcl::visualization::PCLVisualizer vis;
	vis.addModelFromPolyData(polydata, "mesh", 0);

	pcl::io::savePCDFileASCII("data...\\data.pcd", *cloud);
	while (!vis.wasStopped())
	{
		vis.spinOnce();
	}
}