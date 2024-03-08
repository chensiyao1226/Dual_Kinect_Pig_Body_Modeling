#ifndef MATTRANS_H
#define MATTRANS_H

#include <Eigen/Core>
#include <pcl/point_types.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include<boost/thread/thread.hpp>
#include "Sfilter2.h"
typedef pcl::PointXYZL PointT;

void MatTrans(Eigen::Matrix4f transMatrix, char pcdfile[30], char savename[30])
{

	pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr transformed_cloud(new pcl::PointCloud<PointT>());

	pcl::io::loadPCDFile(pcdfile, *cloud);

	cout << "pointcloud transforming..." << endl;
	//点云根据矩阵转换
	pcl::transformPointCloud(*cloud, *transformed_cloud, transMatrix);

	//cloud_half = Sfilter2(transformed_cloud, "y", -2, 0.05, "pig_top");
	//将新生成的点云写入文件
	pcl::PCDWriter writer;
	writer.write<PointT>(savename, *transformed_cloud, false);
	
}
#endif
