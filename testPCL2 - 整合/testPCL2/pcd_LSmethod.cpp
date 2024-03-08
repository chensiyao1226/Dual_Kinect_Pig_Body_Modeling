#ifndef PCD_LSMETHOD_H
#define PCD_LSMETHOD_H

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>  //kd-tree搜索对象的类定义的头文件
#include <pcl/surface/mls.h>        //最小二乘法平滑处理类定义头文件

void pcd_LSmethod(char pcdname[30], char savename[30])
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile(pcdname, *cloud);

	// 创建 KD-Tree
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

	// Output has the PointNormal type in order to store the normals calculated by MLS
	pcl::PointCloud<pcl::PointNormal> mls_points;

	// 定义最小二乘实现的对象mls
	pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;

	mls.setComputeNormals(true);  //设置在最小二乘计算中需要进行法线估计

	// Set parameters
	mls.setInputCloud(cloud);
	
	mls.setPolynomialFit(true);
	mls.setSearchMethod(tree);
	mls.setSearchRadius(0.05);
	// MovingLeastSquares<PointXYZ, PointXYZ> mls;
	// mls.setInputCloud(filtered);
	// mls.setSearchRadius(0.01);
	// mls.setPolynomialFit(true);
	// mls.setPolynomialOrder(2);
	// mls.setUpsamplingMethod(MovingLeastSquares<PointXYZ, PointXYZ>::SAMPLE_LOCAL_PLANE);
	// mls.setUpsamplingRadius(0.005);
	// mls.setUpsamplingStepSize(0.003);

	// PointCloud<PointXYZ>::Ptr cloud_smoothed (new PointCloud<PointXYZ>());
	// mls.process(*cloud_smoothed);
	// cout << "移动最小二乘平面滤波完成" << endl;

	// Reconstruct
	mls.process(mls_points);

	pcl::io::savePCDFile(savename, mls_points);

}
#endif