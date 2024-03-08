#ifndef PCD_BIFILTER_H
#define PCD_BIFILTER_H

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/fast_bilateral.h>


void pcd_BiFilter()
{
	// 读入点云文件
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::PCDReader reader;
	reader.read("pig_side.pcd", *cloud);    //读取点云到cloud中

	pcl::PointCloud<pcl::PointXYZ>::Ptr result(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::FastBilateralFilter<pcl::PointXYZ> filter;
	filter.setInputCloud(cloud);
	filter.setSigmaS(0.5);
	filter.setSigmaR(0.004);
	filter.filter(*result);


	pcl::PCDWriter writer;
	writer.write<pcl::PointXYZ>("pig_side(双边滤波).pcd", *result, false);

}
#endif
