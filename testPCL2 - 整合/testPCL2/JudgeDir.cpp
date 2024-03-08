#ifndef JUDGEDIR_H
#define JUDGEDIR_H

#include <string>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include "reverseZ.h"
using namespace std;

typedef pcl::PointXYZL PointT;

bool JudgeDir(char pcdname[30], int axis)
{
	pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
	pcl::io::loadPCDFile(pcdname, *cloud);

	switch (axis)
	{
	case 1:
		break;
	case 2:
	{
		for (size_t i = 0; i < cloud->points.size(); ++i)
		{
			if (cloud->points[i].label == 1)
			{
				if (cloud->points[i].z > 0.05)
				{
					cout << "direction = 1" << endl;
					return true;
				}
				else if (cloud->points[i].z < -0.05)
				{
					cout << "direction = -1" << endl;
					return false;
				}
				return false;
			}
		}
	}
	break;
	case 3:
	{
		for (size_t i = 0; i < cloud->points.size(); ++i)
		{
			if (cloud->points[i].label == 1)
			{
				if (cloud->points[i].x > 0.05)
				{
					cout << "direction = 1" << endl;
					return true;
				}
				else if (cloud->points[i].x < -0.05)
				{
					cout << "direction = -1" << endl;
					return false;
				}
				return false;
			}
		}
	}
	}
	

}
#endif



