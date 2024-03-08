#pragma once
#ifndef GETMIDPOINT_H
#define GETMIDPOINT_H

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
using namespace std;

void getMidpoint(char pcdname1[30], char pcdname2[30], char savename[30])
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile(pcdname1, *cloud1);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile(pcdname2, *cloud2);

	pcl::PointXYZ p((cloud1->points[0].x + cloud2->points[0].x)/2, (cloud1->points[0].y + cloud2->points[0].y)/2, 
		(cloud1->points[0].z + cloud2->points[0].z)/2);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_save(new pcl::PointCloud<pcl::PointXYZ>);
	cloud_save->push_back(p);

	pcl::PCDWriter writer;
	writer.write<pcl::PointXYZ>(savename, *cloud_save, false);
}

#endif