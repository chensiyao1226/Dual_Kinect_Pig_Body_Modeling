#ifndef REVERSEX_H
#define REVERSEX_H

#include <string>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
using namespace std;

typedef pcl::PointXYZ PointT;

void reverseX(char pcdname[30], char savename[30])
{
	pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr transformed_cloud(new pcl::PointCloud<PointT>);
	pcl::io::loadPCDFile(pcdname, *cloud);

	cout << "reversing X..." << endl;

	Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();

	float theta = M_PI; // The angle of rotation in radians
	
	transform(0, 0) = cos(theta);
	transform(0, 1) = sin(theta);
	transform(1, 0) = -sin(theta);
	transform(1, 1) = cos(theta);
	transform(2, 2) = 1;
	transform(3, 3) = 1;

	pcl::transformPointCloud(*cloud, *transformed_cloud, transform);

	pcl::PCDWriter writer;
	writer.write<PointT>(savename, *transformed_cloud, false);

}
#endif



