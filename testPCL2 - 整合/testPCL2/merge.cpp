#ifndef MERGE_H
#define MERGE_H

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <Eigen/Core>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include "MatRead.h"


typedef pcl::PointXYZ PointT;

void merge(char cloud1[30], char cloud2[30], char Transname[30], char savetransname[30], char saveintename[30])
{
	pcl::PointCloud<PointT>::Ptr cloud_1(new pcl::PointCloud<PointT>);//
	pcl::PointCloud<PointT>::Ptr cloud_2(new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr transformed_cloud(new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr cloud_fusion(new pcl::PointCloud<PointT>);


	pcl::io::loadPCDFile(cloud1, *cloud_1);
	pcl::io::loadPCDFile(cloud2, *cloud_2);

	Eigen::Matrix4f Mat;
	Mat = MatRead(Transname);

	pcl::transformPointCloud(*cloud_1, *transformed_cloud, Mat);

	*cloud_fusion = *transformed_cloud;
	*cloud_fusion += *cloud_2;

	pcl::PCDWriter writer;
	writer.write<PointT>(savetransname, *transformed_cloud, false);//
	writer.write<PointT>(saveintename, *cloud_fusion, false);//
}
#endif