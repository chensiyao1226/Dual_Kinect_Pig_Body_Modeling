#ifndef SEARCHINDEX_H
#define SEARCHINDEX_H

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>


using namespace std;

int searchIndex(char pcdname[30], float x_min, float x_max, float y_min, float y_max, float z_min, float z_max, pcl::PointXYZ &p)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile(pcdname, *cloud);
	pcl::PCDWriter writer;

	for (size_t i = 0; i < cloud->points.size(); ++i)
	{
		if (cloud->points[i].x >= x_min && cloud->points[i].x <= x_max)
			if (cloud->points[i].y >= y_min && cloud->points[i].y <= y_max)
				if (cloud->points[i].z >= z_min && cloud->points[i].z <= z_max)
				{
					cout << "index:" << i << endl;
					p = cloud->points[i];
					return (i);
				}
					
	}
}
#endif