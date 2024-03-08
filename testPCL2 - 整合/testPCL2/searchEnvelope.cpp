#pragma once
#ifndef SEARCHENVELOPE_H
#define SEARCHENVELOPE_H

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
using namespace std;

float searchEnvelope(pcl::PointXYZ point, char cloudPCD[30], int sign, char savename[30])
{

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile(cloudPCD, *cloud);
	//从水平开始 遍历所有点 如果带入有大于0的则斜率减小  如果全小于0则斜率增加     直到变成另一状态  记录斜率
	float k = 0, variety_k = 0.002;
	int orientation, side=1;
	for (size_t i = 0; i < cloud->points.size(); ++i)
	{
		if (cloud->points[i].y*sign > point.y*sign)
		{
			if (cloud->points[i].z*sign < point.z*sign)
				orientation = -1;
			else if (cloud->points[i].z*sign > point.z*sign)
				orientation = 1;
			break;
		}
		else
			orientation = sign;
	}
	cout << "orientation=" << orientation << endl;
	if (point.z < 0)
		side = -1;
	cout << "side=" << side << endl;

	int found = 0, flag = 0;//flag记录found从1到0变化
	if (orientation*sign*side == 1)//
		for (k = 0; k*orientation < 10; k += variety_k*orientation)
		{
			found = 0;
			for (size_t i = 0; i < cloud->points.size(); ++i)
			{
				if (cloud->points[i].y*sign >(k*(cloud->points[i].z - point.z) + point.y)*sign)//y-y1=k(x-x1)
				{
					found = 1;
					break;
				}
			}
			if (found == 1)
				break;
		}
	else
		for (k = 0; k*orientation < 10; k += variety_k*orientation)
		{
			found = 0;
			for (size_t i = 0; i < cloud->points.size(); ++i)
			{
				if (cloud->points[i].y*sign >(k*(cloud->points[i].z - point.z) + point.y)*sign)//y-y1=k(x-x1)
				{
					found = 1;
					flag = 1;
					break;
				}
			}
			if (found == 0 && flag == 1)
				break;
		}
	

	cout << "k=" << k;


	pcl::PointXYZ p;
	float b, variety_b = 0.002;//平行移动变量
	for (b = 0; b*(-sign) < 1; b += -variety_b*sign)
	{
		found = 0;
		for (size_t i = 0; i < cloud->points.size(); ++i)
		{
			if (cloud->points[i].y*sign <(k*(cloud->points[i].z - point.z) + point.y + b)*sign)//y-y1=k(x-x1)
			{
				found = 1;
				p = cloud->points[i];
				break;
			}
		}
		if (found == 1)
			continue;
		else
			break;			
	}
	cout << ", b=" << b << endl;

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_extr(new pcl::PointCloud<pcl::PointXYZ>);
	cloud_extr->push_back(p);
	pcl::PCDWriter writer;
	writer.write<pcl::PointXYZ>(savename, *cloud_extr, false);

	return(k);
}

#endif