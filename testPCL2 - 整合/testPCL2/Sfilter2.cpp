#ifndef SFILTER2_H
#define SFITLER2_H

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include<pcl/visualization//pcl_visualizer.h>
#include<boost/thread/thread.hpp>

typedef pcl::PointXYZL PointT;

void Sfilter2(char pcdname[30], char field[10], float min, float max, char name[30])
{
	pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
	pcl::io::loadPCDFile(pcdname, *cloud);

	pcl::PointCloud<PointT>::Ptr cloud_extracted(new pcl::PointCloud<PointT>);


	std::cerr << "PointCloud before Sfiltering2: " << cloud->width * cloud->height
		<< " data points (" << pcl::getFieldsList(*cloud) << ").";

	//对点云进行直通滤波
	pcl::PassThrough<PointT> pass_fir;
	pass_fir.setInputCloud(cloud);
	pass_fir.setFilterFieldName(field);
	pass_fir.setFilterLimits(min, max);
	//pass_fir.setKeepOrganized(true);
	pass_fir.filter(*cloud_extracted);

	std::cerr << "PointCloud after Sfiltering2: " << cloud_extracted->width * cloud_extracted->height
		<< " data points (" << pcl::getFieldsList(*cloud_extracted) << ")."<<endl;


	pcl::PCDWriter writer;
	writer.write<PointT>(name, *cloud_extracted, false);

}

#endif
