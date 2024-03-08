#ifndef HARRIS_H
#define HARRIS_H

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/keypoints/harris_3d.h>
#include <boost/thread/thread.hpp>
#include <stdlib.h>
#include <iostream>

using namespace std;
typedef pcl::PointXYZ PointType;

void Harris(char pcdname[30], char savename[30])
{
	// ------------------------------------------------------------------
	// -----Read pcd file or create example point cloud if not given-----
	// ------------------------------------------------------------------
	cout << "Harris" << endl;
	//��ȡpcd�ļ������û��ָ���ļ����ʹ���������
	pcl::PointCloud<PointType>::Ptr cloud_ptr(new pcl::PointCloud<PointType>);//���ƶ���ָ��
	pcl::PointCloud<PointType>& cloud = *cloud_ptr;//���á�������Ƶı���������ָ��
	pcl::io::loadPCDFile<PointType>(pcdname, cloud);


	// --------------------------------
	// -----Extract Harri keypoints-----
	// --------------------------------
	// ��ȡHarri�ؼ���
	pcl::HarrisKeypoint3D<pcl::PointXYZ, pcl::PointXYZI, pcl::Normal> harris;
	harris.setInputCloud(cloud_ptr);//����������� ָ��
	harris.setNonMaxSupression(true);
	harris.setRadius(0.002f);//������뾶0.02f
	harris.setThreshold(0.02f);//������ֵ0.01f
	cout << "parameter set successful" << endl;

	//�½��ĵ��Ʊ����ʼ�������㣬����ָ���Խ��
	//ע��Harris��������Ʊ�������ǿ��(I)��Ϣ�� pcl::PointXYZI����Ϊ����ֵ������I������
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_out_ptr(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI>& cloud_out = *cloud_out_ptr;
	cloud_out.height = 1;
	cloud_out.width = 100;
	cloud_out.resize(cloud_out.height * cloud_out.width);
	cloud_out.clear();
	cout << "extracting... " << endl;
	// ����������
	harris.compute(cloud_out);


	int size = cloud_out.size();
	cout << "extraction : " << size << " n keypoints." << endl;

	pcl::PCDWriter writer;
	writer.write<pcl::PointXYZI>(savename, cloud_out, false);

	// --------------------------------------------
	// -----Open 3D viewer and add point cloud-----
	// --------------------------------------------
	// 3D������ʾ
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_show(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud(*cloud_out_ptr, *cloud_show);

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer);//���ӻ�����ָ��
	viewer->addPointCloud(cloud_ptr);//ָ��
	//��3Dͼ�δ�������ʾ�ؼ���
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> harris_color_handler(cloud_show, 0, 255, 0);//��һ����������Ϊ��ָ��
	viewer->addPointCloud(cloud_show, harris_color_handler, "harris");//��һ����������Ϊ��ָ��
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "harris");


	while (!viewer->wasStopped())
	{
		viewer->spinOnce();
		pcl_sleep(0.1);
	}

}

#endif