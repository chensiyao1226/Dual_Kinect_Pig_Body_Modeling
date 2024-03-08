#pragma once
#ifndef GETNORMAL_H
#define GETNORMAL_H

#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include<pcl/visualization//pcl_visualizer.h>
#include<boost/thread/thread.hpp>
#include <math.h>
#include "MatTrans.h"
#include "printMatrix.h"

using namespace std;

void getNormal(char pcdname[30])
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

	float nx = 0, ny = 0, nz = 0;
	for (int ix = 0; ix<cloud_normals->points.size(); ix = ix + 10)
	{
		nx += cloud_normals->points[ix].normal_x;
		ny += cloud_normals->points[ix].normal_y;
		nz += cloud_normals->points[ix].normal_z;
	}
	nx = nx / (cloud_normals->points.size() / 10);
	ny = ny / (cloud_normals->points.size() / 10);
	nz = nz / (cloud_normals->points.size() / 10);

	cout << "normal_x:" << nx << endl;
	cout << "normal_y:" << ny << endl;
	cout << "normal_z:" << nz << endl;

}

#endif