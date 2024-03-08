#ifndef PEAKEXTRA_H
#define PEAKEXTRA_H

#include <string>
#include <iostream>
#include <pcl/console/parse.h>
#include <pcl/console/print.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>

typedef unsigned int Label;
typedef unsigned long long size_t;

using namespace std;
typedef pcl::PointXYZL PointT;

float peakExtra(char pcdname[30], int feild, float &min, float &max)
{
	pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
	pcl::io::loadPCDFile(pcdname, *cloud);

	float median;

	PointT minPt, maxPt;
	pcl::getMinMax3D(*cloud, minPt, maxPt);

	switch (feild)
	{
		case 1:
		{
			max = maxPt.y;
			min = minPt.y;
		}
		break;
		case 2:
		{
			max = maxPt.z;
			min = minPt.z;
		}
		break;
		case 3:
		{
			max = maxPt.x;
			min = minPt.x;
		}
	}
	
	cout << "min:" << min << "  " << "max:" << max << endl;
	return(median = (max + min) / 2);//ÇÐ¸îÏßyÖµ
}
#endif



