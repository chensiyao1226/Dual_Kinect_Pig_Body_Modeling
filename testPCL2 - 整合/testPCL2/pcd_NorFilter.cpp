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

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p(new pcl::PointCloud<pcl::PointXYZ>), cloud_f(new pcl::PointCloud<pcl::PointXYZ>);//��ȡ������ơ��������
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);//�����洢���Ʒ��߶���
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());

	//ͳ���˲�ǰ�ĵ��Ƹ���
	std::cerr << "PointCloud before Norfiltering: " << cloud->width * cloud->height << " data points." <<endl;


	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal; //���߹��ƶ���
	normal.setSearchMethod(tree);
	normal.setInputCloud(cloud);
	normal.setKSearch(100);
	normal.compute(*cloud_normals);//�����ƺ�ķ��ߴ���cloud_normals

	//����ģ��ϵ��
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());   //����ģ��ϵ������
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices());                  //�����洢ƽ���ϵ�������Ķ���

	//�ָ���Ʒ�Ϊ�����֣�1��ѡ������ 2����ȡ����

	//��һ����ѡ����Ҫ�ָ���ǲ��ֵ���
	pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;//�ָ���� 
	seg.setOptimizeCoefficients(true);                    //���öԹ���ģ�Ͳ��������Ż�����
	seg.setModelType(pcl::SACMODEL_PLANE);                //���÷ָ�ģ�����
	seg.setMethodType(pcl::SAC_RANSAC);                   //�������ĸ�����������Ʒ���
	seg.setMaxIterations(200);                            //��������������
	seg.setDistanceThreshold(0.02);                       //�ж��Ƿ�Ϊģ���ڵ�ľ��뷧ֵ 0.02
	seg.setInputCloud(cloud);
	seg.setInputNormals(cloud_normals);
	seg.segment(*inliers, *coefficients);


	pcl::PCDWriter writer;

	// ����ExtractIndices��ʵ�ʲ�������һ���ǽ�ѡ���ĵ�����������ȡ������
	pcl::ExtractIndices<pcl::PointXYZ> extract;			   //����������ȡ����
	extract.setInputCloud(cloud);
	extract.setIndices(inliers);

	//��ȡ����
	extract.setNegative(false);
	extract.filter(*cloud_f);
	std::cerr << "PointCloud representing the planar component: " << cloud_f->width * cloud_f->height << " data points." << std::endl;


	//��ȡ����
	extract.setNegative(true);
	extract.filter(*cloud_p);
	std::cerr << "PointCloud representing the remain component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;
	

	writer.write<pcl::PointXYZ>(savename, *cloud_p, false);
	writer.write<pcl::PointXYZ>(floorname, *cloud_f, false);
}

#endif