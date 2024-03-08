#ifndef PCD_DISPALY2_H
#define PCD_DISPLAY2_H

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include<boost/thread/thread.hpp>


void pcd_display2(char file1[30], char file2[30])
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer1(new pcl::visualization::PCLVisualizer("viewer1"));
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer2(new pcl::visualization::PCLVisualizer("viewer2"));

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile(file1, *cloud1);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile(file2, *cloud2);

	viewer1->addCoordinateSystem();
	viewer2->addCoordinateSystem();
	viewer1->addPointCloud<pcl::PointXYZ>(cloud1, "table1");
	viewer2->addPointCloud<pcl::PointXYZ>(cloud2, "table2");

	while (!viewer1->wasStopped())
	{
		viewer1->spinOnce(10);
		boost::this_thread::sleep(boost::posix_time::microseconds(1000000));
	}
	while (!viewer2->wasStopped())
	{
		viewer2->spinOnce(10);
		boost::this_thread::sleep(boost::posix_time::microseconds(1000000));
	}
}
#endif