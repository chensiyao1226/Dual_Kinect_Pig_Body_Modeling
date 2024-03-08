#ifndef PCD_BILFILTER_H
#define PCD_BILFILTER_H


#include <pcl/point_types.h>  
#include <pcl/io/pcd_io.h>  
#include <pcl/kdtree/kdtree_flann.h>  
#include <pcl/filters/bilateral.h>  
#include <pcl/common/impl/io.hpp>
#include <pcl/point_cloud.h>


#include <pcl/kdtree/flann.h>  
#include <pcl/kdtree/kdtree.h>  
#include <pcl/search/flann_search.h>  
#include <pcl/search/kdtree.h>  



void pcd_BilFilter()
{
	// ��������ļ�
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr outcloud(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_i(new pcl::PointCloud<pcl::PointXYZI>);


	pcl::PCDReader reader;
	reader.read("pig_side(�²���).pcd", *cloud);    //��ȡ���Ƶ�cloud��
	std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height
		<< " data points (" << pcl::getFieldsList(*cloud) << ").";

	pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
	//pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr tree(new pcl::KdTreeFLANN <pcl::PointXYZI>);
	pcl::BilateralFilter<pcl::PointXYZI> bf;//��Ҫ��ǿ����Ϣxyzi
	pcl::copyPointCloud(*cloud, *cloud_i);//����ǿ��ת�����Ա���ͨ���������в����У���Ҫǿ����Ϣ

	bf.setInputCloud(cloud_i);
	bf.setSearchMethod(tree);
	bf.setHalfSize(0.5);
	bf.setStdDev(0.004);
	bf.filter(*outcloud);

	pcl::PCDWriter writer;
	writer.write<pcl::PointXYZI>("pig_side(˫���˲�).pcd", *outcloud, false);
	std::cerr << "PointCloud after filtering: " << outcloud->width * outcloud->height
		<< " data points (" << pcl::getFieldsList(*outcloud) << ").";

}
#endif
