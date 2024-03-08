#pragma once
#ifndef SEARCHPOINT_H
#define SEARCHPOINT_H

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
using namespace std;

void searchPoint(float z, char pcdname[30], char savename[30])
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile(pcdname, *cloud);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_extr(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointXYZ p;
	while (cloud_extr->points.size() == 0)
	{
		for (size_t i = 0; i < cloud->points.size(); ++i)
		{
			if (cloud->points[i].z > z - 0.006 && cloud->points[i].z < z + 0.006)
			{
				p = cloud->points[i];
				cloud_extr->push_back(p);
				break;
			}
		}
		z += 0.01;//尾部z极值点在一侧 另一侧需要范围检测
	}
		

	pcl::PCDWriter writer;
	writer.write<pcl::PointXYZ>(savename, *cloud_extr, false);
}

#endif