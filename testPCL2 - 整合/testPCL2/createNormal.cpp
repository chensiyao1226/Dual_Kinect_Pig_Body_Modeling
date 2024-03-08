
#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include<pcl/visualization//pcl_visualizer.h>
#include<boost/thread/thread.hpp>
#include <math.h>
#include "MatTrans.h"
#include "printMatrix.h"

using namespace std;

void createNormal(char pcdname[30], char savename[30])
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile(pcdname, *cloud);

	pcl::PointCloud<pcl::Normal>::Ptr cloud_normal(new pcl::PointCloud<pcl::Normal>);//创建存储点云法线对象
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());



	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal; //法线估计对象
	normal.setSearchMethod(tree);
	normal.setInputCloud(cloud);
	normal.setKSearch(30);
	normal.compute(*cloud_normal);//将估计后的法线存入cloud_normals

	double nx = 0, ny = 0, nz = 0;
	for (int ix = 0; ix<cloud_normal->points.size(); ix = ix + 10)
	{
		nx += cloud_normal->points[ix].normal_x;
		ny += cloud_normal->points[ix].normal_y;
		nz += cloud_normal->points[ix].normal_z;
	}
	nx = (nx / (cloud_normal->points.size() / 10));///abs
	ny = ny / (cloud_normal->points.size() / 10);
	nz = nz / (cloud_normal->points.size() / 10);

	if (nx < 0)//法线反向
	{
		nx = -nx;
		ny = -ny;
		nz = -nz;
	}
		

	cout << "normal_x:" << nx << endl;
	cout << "normal_y:" << ny << endl;
	cout << "normal_z:" << nz << endl;

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_normals(new pcl::PointCloud<pcl::PointXYZ>);//创建存储点云法线对象
	pcl::PointXYZ p1(0, 0, 0), p2(nx, ny, nz);

	cloud_normals->push_back(p1);
	cloud_normals->push_back(p2);

	cloud_normals->width = 2;
	cloud_normals->height = 1;
	cloud_normals->points.resize(2);

	pcl::PCDWriter writer;
	writer.write<pcl::PointXYZ>(savename, *cloud_normals, false);

}
