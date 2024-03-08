#ifndef PCD_NORFILTER_H
#define PCD_NORFILTER_H

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

void pcd_NorFilter(char pcdname[30], char savename[30], char floorname[30])
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile(pcdname, *cloud);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p(new pcl::PointCloud<pcl::PointXYZ>), cloud_f(new pcl::PointCloud<pcl::PointXYZ>);//提取的猪点云、地面点云
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);//创建存储点云法线对象
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());

	//统计滤波前的点云个数
	std::cerr << "PointCloud before Norfiltering: " << cloud->width * cloud->height << " data points." <<endl;


	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal; //法线估计对象
	normal.setSearchMethod(tree);
	normal.setInputCloud(cloud);
	normal.setKSearch(100);
	normal.compute(*cloud_normals);//将估计后的法线存入cloud_normals

	//创建模型系数
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());   //创建模型系数对象
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices());                  //创建存储平面上点的索引的对象

	//分割点云分为两部分：1、选出点云 2、提取点云

	//这一步是选出将要分割的那部分点云
	pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;//分割对象 
	seg.setOptimizeCoefficients(true);                    //设置对估计模型参数进行优化处理
	seg.setModelType(pcl::SACMODEL_PLANE);                //设置分割模型类别
	seg.setMethodType(pcl::SAC_RANSAC);                   //设置用哪个随机参数估计方法
	seg.setMaxIterations(200);                            //设置最大迭代次数
	seg.setDistanceThreshold(0.02);                       //判断是否为模型内点的距离阀值 0.02
	seg.setInputCloud(cloud);
	seg.setInputNormals(cloud_normals);
	seg.segment(*inliers, *coefficients);


	pcl::PCDWriter writer;

	// 设置ExtractIndices的实际参数（这一步是将选出的点云真正的提取出来）
	pcl::ExtractIndices<pcl::PointXYZ> extract;			   //创建点云提取对象
	extract.setInputCloud(cloud);
	extract.setIndices(inliers);

	//提取地面
	extract.setNegative(false);
	extract.filter(*cloud_f);
	std::cerr << "PointCloud representing the planar component: " << cloud_f->width * cloud_f->height << " data points." << std::endl;


	//提取猪体
	extract.setNegative(true);
	extract.filter(*cloud_p);
	std::cerr << "PointCloud representing the remain component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;
	

	writer.write<pcl::PointXYZ>(savename, *cloud_p, false);
	writer.write<pcl::PointXYZ>(floorname, *cloud_f, false);
}

#endif