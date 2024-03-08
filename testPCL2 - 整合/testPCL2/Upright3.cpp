#pragma once
#ifndef UPRIGHT3_H
#define UPRIGHT3_H

#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include<pcl/visualization//pcl_visualizer.h>
#include<boost/thread/thread.hpp>
#include <math.h>
#include "MatTrans.h"
#include "printMatrix.h"

using namespace std;

Eigen::Matrix4f Upright3(char refername[30], char pcdname[30], char savename[30])
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile(pcdname, *cloud);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_normals(new pcl::PointCloud<pcl::PointXYZ>);//�����洢���Ʒ��߶���
	pcl::io::loadPCDFile(refername, *cloud_normals);



	double nx = cloud_normals->points[1].x - cloud_normals->points[0].x, 
		ny = cloud_normals->points[1].y - cloud_normals->points[0].y, 
		nz = cloud_normals->points[1].z - cloud_normals->points[0].z;
	

	cout << "reference normal_x:" << nx << endl;
	cout << "reference normal_y:" << ny << endl;
	cout << "reference normal_z:" << nz << endl;

	Eigen::Matrix4f tm3 = Eigen::Matrix4f::Identity();//��λ����


	double cosRx = nz / sqrt(nz*nz + ny*ny), sinRx = sqrt(1 - cosRx*cosRx) * (ny / abs(ny));//sin������ ����ת�������򸺷��� ��ʱ��Ϊ�� ˳ʱ��Ϊ��  ��������ϵ


	tm3(1, 1) = cosRx; tm3(1, 2) = sinRx;
	tm3(2, 1) = -sinRx; tm3(2, 2) = cosRx;

	printMatrix(tm3);
	MatTrans(tm3, pcdname, savename);

	return(tm3);
}

#endif