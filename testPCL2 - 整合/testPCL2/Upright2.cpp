#pragma once
#ifndef UPRIGHT2_H
#define UPRIGHT2_H

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

Eigen::Matrix4f Upright2(char pcdname[30], char savename[30])
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile(pcdname, *cloud);

	pcl::PointXYZ p1 = cloud->points[0];
	pcl::PointXYZ p2 = cloud->points[1];

	double nx = 0, ny = 0, nz = 0;
	nx = p2.x - p1.x;
	ny = p2.y - p1.y;
	nz = p2.z - p1.z;

	cout << "normal_x:" << nx << endl;
	cout << "normal_y:" << ny << endl;
	cout << "normal_z:" << nz << endl;

	Eigen::Matrix4f tm2 = Eigen::Matrix4f::Identity();//单位矩阵


	double cosRy = nx / sqrt(nz*nz + nx*nx), sinRy = sqrt(1 - cosRy*cosRy) * (-nz / abs(nz));//sin有正负 从旋转轴由正向负方向看 逆时针为正 顺时针为负  右手坐标系


	tm2(0, 0) = cosRy; tm2(0, 2) = -sinRy;
	tm2(2, 0) = sinRy; tm2(2, 2) = cosRy;

	printMatrix(tm2);
	MatTrans(tm2, pcdname, savename);

	return(tm2);
}

#endif