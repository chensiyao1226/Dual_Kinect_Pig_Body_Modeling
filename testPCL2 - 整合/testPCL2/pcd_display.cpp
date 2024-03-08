#ifndef PCD_DISPALY_H
#define PCD_DISPLAY_H

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include<boost/thread/thread.hpp>


void pcd_display(char file[30],char table[20], int i)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer(table));
	switch (i)
	{
		case 0:
		{
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
			pcl::io::loadPCDFile(file, *cloud1);
			viewer->addCoordinateSystem();
			viewer->addPointCloud<pcl::PointXYZ>(cloud1, table);
			break;
		}
		case 1:
		{
			pcl::PointCloud<pcl::PointNormal>::Ptr cloud2(new pcl::PointCloud<pcl::PointNormal>);
			pcl::io::loadPCDFile(file, *cloud2);
			viewer->addPointCloud<pcl::PointNormal>(cloud2, table);
			break;
		}
		case 2:
		{
			pcl::PointCloud<pcl::PointXYZL>::Ptr cloud3(new pcl::PointCloud<pcl::PointXYZL>);
			pcl::io::loadPCDFile(file, *cloud3);
			viewer->addPointCloud(cloud3, table);
		}
	}
	
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(10);
		boost::this_thread::sleep(boost::posix_time::microseconds(1000000));
	}
}
#endif