#ifndef PCD_GAUSFILTER_H
#define PCD_GAUSFILTER_H

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/convolution_3d.h>
#include <pcl/filters/convolution.h>
#include <pcl/search/kdtree.h>

void pcd_GausFilter()//需要结构点云？
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	if (pcl::io::loadPCDFile("pig_side(去噪).pcd", *inputCloud) == -1)
	{
		PCL_ERROR("Couldn't read file test_pcd.pcd\n");
	}
	pcl::filters::Convolution<pcl::PointXYZ, pcl::PointXYZ>  convolution;
	Eigen::ArrayXf gaussian_kernel(5);
	gaussian_kernel << 1.f / 16, 1.f / 4, 3.f / 8, 1.f / 4, 1.f / 16;
	convolution.setBordersPolicy(
		pcl::filters::Convolution<pcl::PointXYZ, pcl::PointXYZ>::BORDERS_POLICY_IGNORE);
	convolution.setDistanceThreshold(static_cast<float> (0.1));
	convolution.setInputCloud(inputCloud);
	convolution.setKernel(gaussian_kernel);
	convolution.convolve(*cloud);
}
#endif