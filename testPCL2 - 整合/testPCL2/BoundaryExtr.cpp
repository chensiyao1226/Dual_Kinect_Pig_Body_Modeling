#ifndef BOUNDARYEXTR_H
#define BOUNDARYEXTR_H

#include <iostream>
#include <vector>
#include <ctime>
#include <boost/thread/thread.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/features/eigen.h>
#include <pcl/features/feature.h>
#include <pcl/features/normal_3d.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/features/boundary.h>
#include <pcl/visualization/cloud_viewer.h>
using namespace std;

void BoundaryExtr(char pcdname[30], char savename[30], char savePlanename[30])
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile(pcdname, *cloud);

	std::cout << "points size is:" << cloud->size() << std::endl;
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<pcl::Boundary> boundaries;
	pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> est;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());


	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normEst;  //����pcl::PointXYZ��ʾ�����������ݣ�pcl::Normal��ʾ�������,��pcl::Normalǰ�����Ƿ������һ��������
	normEst.setInputCloud(cloud);
	normEst.setSearchMethod(tree);
	// normEst.setRadiusSearch(2);  //������Ƶİ뾶
	normEst.setKSearch(9);  //������Ƶĵ���
	normEst.compute(*normals);
	cout << "normal size is " << normals->size() << endl;

	//normal_est.setViewPoint(0,0,0); //���Ӧ�û�ʹ����һ��
	est.setInputCloud(cloud);
	est.setInputNormals(normals);
	//  est.setAngleThreshold(90);
	//   est.setSearchMethod (pcl::search::KdTree<pcl::PointXYZ>::Ptr (new pcl::search::KdTree<pcl::PointXYZ>));
	est.setSearchMethod(tree);
	est.setKSearch(20);  //һ���������ֵԽ�ߣ����ձ߽�ʶ��ľ���Խ��
	//  est.setRadiusSearch(everagedistance);  //�����뾶
	est.compute(boundaries);

	//  pcl::PointCloud<pcl::PointXYZ> boundPoints;
	pcl::PointCloud<pcl::PointXYZ>::Ptr boundPoints(new pcl::PointCloud<pcl::PointXYZ>);
	int countBoundaries = 0;
	for (int i = 0; i<cloud->size(); i++)
	{
		uint8_t x = (boundaries.points[i].boundary_point);
		int a = static_cast<int>(x); //�ú����Ĺ�����ǿ������ת��
		if (a == 1)
		{
			//  boundPoints.push_back(cloud->points[i]);
			(*boundPoints).push_back(cloud->points[i]);
			countBoundaries++;
		}
	}
	std::cout << "boudary size is��" << countBoundaries << std::endl;
	//  pcl::io::savePCDFileASCII("boudary.pcd",boundPoints);

	pcl::PointXYZ P;
	pcl::PointCloud<pcl::PointXYZ>::Ptr boundPointsPlane(new pcl::PointCloud<pcl::PointXYZ>);
	for (int i = 0; i<boundPoints->size(); i++)
	{
		P = boundPoints->points[i];
		P.x = 0;
		boundPointsPlane->push_back(P);
	}

	pcl::io::savePCDFileASCII(savename, *boundPoints);
	pcl::io::savePCDFileASCII(savePlanename, *boundPointsPlane);

}
#endif