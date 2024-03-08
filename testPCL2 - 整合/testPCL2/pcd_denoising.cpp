#ifndef PCD_DENOISING_H
#define PCD_DENOISING_H

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>

void pcd_denoising(char pcdname[30], char savename[30], double threshhold)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile(pcdname, *cloud);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_remain(new pcl::PointCloud<pcl::PointXYZ>);

	std::cerr << "cloud before denoising: " << cloud->width * cloud->height << endl;

	// �����˲�������ÿ����������ٽ���ĸ�������Ϊ50 ��������׼��ı�������Ϊ1  ����ζ�����һ
	//����ľ��볬����ƽ������һ����׼�����ϣ���õ㱻���Ϊ��Ⱥ�㣬�������Ƴ����洢����
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;   //�����˲�������
	sor.setInputCloud(cloud);                           //���ô��˲��ĵ���
	sor.setMeanK(30);									 //�����ڽ���ͳ��ʱ���ǲ�ѯ���ٽ�����
	sor.setStddevMulThresh(threshhold);						//�����ж��Ƿ�Ϊ��Ⱥ��ķ�ֵ
	sor.filter(*cloud_remain);							 //�洢
	//��ʾȥ�����������
	std::cerr << "cloud after denoising: " << cloud_remain->width * cloud_remain->height << endl;


	pcl::PCDWriter writer;
	writer.write<pcl::PointXYZ>(savename, *cloud_remain, false);

}
#endif