#ifndef PCD_SFILTER_H
#define PCD_SFITLER_H

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include<pcl/visualization//pcl_visualizer.h>
#include<boost/thread/thread.hpp>

void pcd_Sfilter(char pcdname[30], char savename[30])
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile(pcdname, *cloud);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_extracted_fir(new pcl::PointCloud<pcl::PointXYZ>);//直通滤波z轴
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_extracted_sec(new pcl::PointCloud<pcl::PointXYZ>);//直通滤波y轴


	std::cerr << "PointCloud before Sfiltering: " << cloud->width * cloud->height
		<< " data points (" << pcl::getFieldsList(*cloud) << ").";

	//对点云进行直通滤波
	pcl::PassThrough<pcl::PointXYZ> pass_fir;
	pass_fir.setInputCloud(cloud);
	pass_fir.setFilterFieldName("z");
	pass_fir.setFilterLimits(0.2, 2.2);
	//pass_fir.setKeepOrganized(true);
	pass_fir.filter(*cloud_extracted_fir);

	//对点云进行直通滤波
	pcl::PassThrough<pcl::PointXYZ> pass_sec;
	pass_sec.setInputCloud(cloud_extracted_fir);
	pass_sec.setFilterFieldName("y");
	pass_sec.setFilterLimits(-1.3, 1.3);
	pass_sec.setKeepOrganized(true);
	pass_sec.filter(*cloud_extracted_sec);
	cout << "PointCloud after Sfiltering has: " << cloud_extracted_sec->points.size() << " data points." << endl;


	// 保存下采样后的点云
	pcl::PCDWriter writer;
	writer.write<pcl::PointXYZ>(savename, *cloud_extracted_sec, false);

}

#endif