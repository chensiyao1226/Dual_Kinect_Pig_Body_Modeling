#pragma once
#ifndef CURVATURE_H
#define CURVATURE_H

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
using namespace std;

void curvature(char pcdname[30], char savename[30])
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile(pcdname, *cloud);

	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);//创建存储点云法线对象
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());



	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal; //法线估计对象
	normal.setSearchMethod(tree);
	normal.setInputCloud(cloud);
	normal.setKSearch(30);
	normal.compute(*cloud_normals);//将估计后的法线存入cloud_normals

	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields(*cloud, *cloud_normals, *cloud_with_normals);
	cout << "curvature " << endl;
	pcl::io::savePCDFileASCII(savename, *cloud_with_normals);
}

#endif