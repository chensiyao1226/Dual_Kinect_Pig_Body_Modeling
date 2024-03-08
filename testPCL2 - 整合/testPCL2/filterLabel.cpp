#ifndef FILTERLABEL_H
#define FILTERLABEL_H

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/console/parse.h>
#include <pcl/console/print.h>
#include<pcl/visualization//pcl_visualizer.h>
#include<boost/thread/thread.hpp>

typedef unsigned int Label;
typedef unsigned long long size_t;

pcl::PointCloud<pcl::PointXYZL>::Ptr filterLabel(Label label, char pcdname[30], char savename[30])
{
	pcl::PointCloud<pcl::PointXYZL>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZL>);
	pcl::io::loadPCDFile(pcdname, *cloud);

	pcl::PointCloud<pcl::PointXYZL>::Ptr cloud_fil(new pcl::PointCloud<pcl::PointXYZL>);//Ö±Í¨ÂË²¨label

	std::cerr << "PointCloud before label filtering: " << cloud->width * cloud->height
		<< " data points (" << pcl::getFieldsList(*cloud) << ")."<<endl;
	
	size_t size_fil = 0;
	for (size_t i = 0; i < cloud->size(); ++i)
	{
		if (cloud->points[i].label == label)
		{
			cloud_fil->push_back(cloud->points[i]);
			size_fil++;
		}
	}
	cloud_fil->points.resize(size_fil);
	cloud_fil->width = size_fil;
	cloud_fil->height = 1;
	// ±£´æ
	pcl::PCDWriter writer;
	writer.write<pcl::PointXYZL>(savename, *cloud_fil, false);
	return (cloud_fil);

}

#endif