#ifndef GETX_H
#define GETX_H

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>


using namespace std;

float getX(char pcdname[30], float y_min, float y_max, float z_min, float z_max)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile(pcdname, *cloud);

	for (size_t i = 0; i < cloud->points.size(); ++i)
	{
		if (cloud->points[i].y >= y_min && cloud->points[i].y <= y_max)
			if (cloud->points[i].z >= z_min && cloud->points[i].z <= z_max)
			{
				cout << "getX:" << cloud->points[i].x << endl;
				return (cloud->points[i].x);
			}
				
	}
}
#endif