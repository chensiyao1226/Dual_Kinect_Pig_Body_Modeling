#ifndef PCD_BIFILTER_H
#define PCD_BIFILTER_H

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/fast_bilateral.h>


void pcd_BiFilter()
{
	// ��������ļ�
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::PCDReader reader;
	reader.read("pig_side.pcd", *cloud);    //��ȡ���Ƶ�cloud��

	pcl::PointCloud<pcl::PointXYZ>::Ptr result(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::FastBilateralFilter<pcl::PointXYZ> filter;
	filter.setInputCloud(cloud);
	filter.setSigmaS(0.5);
	filter.setSigmaR(0.004);
	filter.filter(*result);


	pcl::PCDWriter writer;
	writer.write<pcl::PointXYZ>("pig_side(˫���˲�).pcd", *result, false);

}
#endif
