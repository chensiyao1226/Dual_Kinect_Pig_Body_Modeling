#ifndef SIGNDIR_H
#define SIGNDIR_H

#include "peakExtra.h"
#include <string>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
using namespace std;


void signDir(char topname[30], char basisname[30], char sidename[30], int axis, char savetopname[30], char savesidename[30])
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_top(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZL>::Ptr signed_cloud_top(new pcl::PointCloud<pcl::PointXYZL>);
	pcl::io::loadPCDFile(topname, *cloud_top);

	cout << "signing direction..." << endl;
	signed_cloud_top->points.resize(cloud_top->points.size());
	signed_cloud_top->width = cloud_top->points.size();
	signed_cloud_top->height = 1;


	float min_top = 10, max_top = -10;
	peakExtra(topname, axis, min_top, max_top);

	float min_b = 10, max_b = -10;
	peakExtra(basisname, axis, min_b, max_b);

	int dir = 0;
	if (abs(max_top - max_b) > abs(min_top - min_b))
		dir = 1;
	else
		dir = -1;
	
	cout << "head direction:" << dir << endl;


	for (size_t i = 0; i < cloud_top->points.size(); ++i)
	{
		signed_cloud_top->points[i].x = cloud_top->points[i].x;
		signed_cloud_top->points[i].y = cloud_top->points[i].y;
		signed_cloud_top->points[i].z = cloud_top->points[i].z;
		if (dir * (cloud_top->points[i].x) >min_top + (max_top - min_top)*3 / 4)//.axis
			signed_cloud_top->points[i].label = 1;
		else
			signed_cloud_top->points[i].label = 0;
	}

	pcl::PCDWriter writer;
	writer.write<pcl::PointXYZL>(savetopname, *signed_cloud_top, false);




	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_side(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZL>::Ptr signed_cloud_side(new pcl::PointCloud<pcl::PointXYZL>);
	pcl::io::loadPCDFile(sidename, *cloud_side);
	signed_cloud_side->points.resize(cloud_side->points.size());
	signed_cloud_side->width = cloud_side->points.size();
	signed_cloud_side->height = 1;

	float min_side = 10, max_side = -10;
	peakExtra(sidename, axis, min_side, max_side);
	
	for (size_t i = 0; i < cloud_side->points.size(); ++i)
	{
		signed_cloud_side->points[i].x = cloud_side->points[i].x;
		signed_cloud_side->points[i].y = cloud_side->points[i].y;
		signed_cloud_side->points[i].z = cloud_side->points[i].z;
		if (dir * (cloud_side->points[i].x) < min_side + (max_side - min_side) / 4)//因为两相机不同侧
			signed_cloud_side->points[i].label = 1;
		else
			signed_cloud_side->points[i].label = 0;
	}

	writer.write<pcl::PointXYZL>(savesidename, *signed_cloud_side, false);

}
#endif



