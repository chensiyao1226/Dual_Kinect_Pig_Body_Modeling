#ifndef PCD_SAMPLING_H
#define PCD_SAMPLING_H

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <boost/thread/thread.hpp>
#include <pcl/visualization/pcl_visualizer.h>

void pcd_sampling(char pcdname[30], char savename[30])
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile(pcdname, *cloud);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sampled(new pcl::PointCloud<pcl::PointXYZ>);

	std::cerr << "PointCloud before sampling: " << cloud->width * cloud->height
		<< " data points (" << pcl::getFieldsList(*cloud) << ").";

	//����һ��Ҷ��СΪ1cm��pcl::VoxelGrid�˲���
	pcl::VoxelGrid<pcl::PointXYZ> sor;  //�����˲�����
	sor.setInputCloud(cloud);            //������Ҫ���˵ĵ��Ƹ��˲�����
	sor.setLeafSize(0.01f, 0.01f, 0.01f);  //�����˲�ʱ�������������Ϊ1cm��������
	sor.filter(*cloud_sampled);           //ִ���˲������洢���

	std::cerr << "PointCloud after sampling: " << cloud_sampled->width * cloud_sampled->height
		<< " data points (" << pcl::getFieldsList(*cloud_sampled) << ")."<<endl;


	// �����²�����ĵ���
	pcl::PCDWriter writer;
	writer.write<pcl::PointXYZ>(savename, *cloud_sampled, false);

}
#endif