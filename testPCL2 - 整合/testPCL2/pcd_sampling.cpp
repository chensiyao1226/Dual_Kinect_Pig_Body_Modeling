#ifndef PCD_SAMPLING_H
#define PCD_SAMPLING_H

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <boost/thread/thread.hpp>
#include <pcl/visualization/pcl_visualizer.h>

void pcd_sampling(char pcdname[30], char savename[30])
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile(pcdname, *cloud);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sampled(new pcl::PointCloud<pcl::PointXYZ>);

	std::cerr << "PointCloud before sampling: " << cloud->width * cloud->height
		<< " data points (" << pcl::getFieldsList(*cloud) << ").";

	//创建一个叶大小为1cm的pcl::VoxelGrid滤波器
	pcl::VoxelGrid<pcl::PointXYZ> sor;  //创建滤波对象
	sor.setInputCloud(cloud);            //设置需要过滤的点云给滤波对象
	sor.setLeafSize(0.01f, 0.01f, 0.01f);  //设置滤波时创建的体素体积为1cm的立方体
	sor.filter(*cloud_sampled);           //执行滤波处理，存储输出

	std::cerr << "PointCloud after sampling: " << cloud_sampled->width * cloud_sampled->height
		<< " data points (" << pcl::getFieldsList(*cloud_sampled) << ")."<<endl;


	// 保存下采样后的点云
	pcl::PCDWriter writer;
	writer.write<pcl::PointXYZ>(savename, *cloud_sampled, false);

}
#endif