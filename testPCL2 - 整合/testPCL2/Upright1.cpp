#pragma once
#ifndef UPRIGHT1_H
#define UPRIGHT1_H

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

Eigen::Matrix4f Upright1(char pcdname[30], char savename[30])
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile(pcdname, *cloud);

	pcl::PointXYZ p1 = cloud->points[0];
	pcl::PointXYZ p2 = cloud->points[1];

	double nx=0, ny=0, nz=0;
	nx = p2.x - p1.x;
	ny = p2.y - p1.y;
	nz = p2.z - p1.z;

	Eigen::Matrix4f tm1 = Eigen::Matrix4f::Identity();//单位矩阵

	
	double cosRz = nx / sqrt(ny*ny + nx*nx), sinRz = sqrt(1 - cosRz*cosRz) * (ny/abs(ny));//sin有正负 从旋转轴由正向负方向看 逆时针为正 顺时针为负


	tm1(0, 0) = cosRz; tm1(0, 1) = sinRz;
	tm1(1, 0) = -sinRz; tm1(1, 1) = cosRz;



	printMatrix(tm1);
	MatTrans(tm1, pcdname, savename);
	
	return(tm1);
}

#endif