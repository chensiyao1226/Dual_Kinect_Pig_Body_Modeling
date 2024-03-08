#ifndef PCD_DENOISING_H
#define PCD_DENOISING_H

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>

void pcd_denoising(char pcdname[30], char savename[30], double threshhold)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile(pcdname, *cloud);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_remain(new pcl::PointCloud<pcl::PointXYZ>);

	std::cerr << "cloud before denoising: " << cloud->width * cloud->height << endl;

	// 创建滤波器，对每个点分析的临近点的个数设置为50 ，并将标准差的倍数设置为1  这意味着如果一
	//个点的距离超出了平均距离一个标准差以上，则该点被标记为离群点，并将它移除，存储起来
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;   //创建滤波器对象
	sor.setInputCloud(cloud);                           //设置待滤波的点云
	sor.setMeanK(30);									 //设置在进行统计时考虑查询点临近点数
	sor.setStddevMulThresh(threshhold);						//设置判断是否为离群点的阀值
	sor.filter(*cloud_remain);							 //存储
	//显示去噪完点云数量
	std::cerr << "cloud after denoising: " << cloud_remain->width * cloud_remain->height << endl;


	pcl::PCDWriter writer;
	writer.write<pcl::PointXYZ>(savename, *cloud_remain, false);

}
#endif