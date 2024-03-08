#ifndef CALDISTANCE_H
#define CALDISTANCE_H

#include <iostream>
#include "math.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>


using namespace std;

float calDistance(char pcdname1[30], char pcdname2[30])
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile(pcdname1, *cloud1);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile(pcdname2, *cloud2);

	float distance = pow((cloud1->points[0].x - cloud2->points[0].x), 2) + pow((cloud1->points[0].y - cloud2->points[0].y), 2)
		+ pow((cloud1->points[0].z - cloud2->points[0].z), 2);
	return(sqrt(distance));
}
#endif