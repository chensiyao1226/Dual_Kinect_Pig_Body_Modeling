#ifndef REVERSEZ_H
#define REVERSEZ_H

#include <string>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>

typedef pcl::PointXYZL PointT;

void reverseZ(char pcdname[30])
{
	pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr transformed_cloud(new pcl::PointCloud<PointT>);
	pcl::io::loadPCDFile(pcdname, *cloud);


	Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();

	float theta = M_PI; // The angle of rotation in radians
	transform(0, 0) = cos(theta);
	transform(0, 2) = sin(theta);
	transform(1, 1) = 1;
	transform(2, 0) = -sin(theta);
	transform(2, 2) = cos(theta);
	transform(3, 3) = 1;

	pcl::transformPointCloud(*cloud, *transformed_cloud, transform);

	pcl::PCDWriter writer;
	writer.write<PointT>(pcdname, *transformed_cloud, false);

}
#endif



