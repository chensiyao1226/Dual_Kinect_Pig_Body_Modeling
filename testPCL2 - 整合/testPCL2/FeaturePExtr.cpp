#pragma once
#ifndef FEATUREPEXTR_H
#define FEATUREPEXTR_H

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
using namespace std;

void FeaturePExtr(char pcdname[30], char savename[30])
{
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointNormal>);
	pcl::io::loadPCDFile(pcdname, *cloud);
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_extr(new pcl::PointCloud<pcl::PointNormal>);

	for (size_t i = 0; i < cloud->points.size(); ++i)
	{
		if (cloud->points[i].curvature < 0.002)
			cloud_extr->points.push_back(cloud->points[i]);

	}
	cloud_extr->width = cloud_extr->points.size();
	cloud_extr->height = 1;
	cloud_extr->is_dense = true;

	pcl::PCDWriter writer;
	writer.write<pcl::PointNormal>(savename, *cloud_extr, false);
	cout << "feature point extracted" << endl;
}

#endif