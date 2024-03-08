#ifndef PCD_CLUSTERING_H
#define PCD_CLUSTERING_H

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <boost/thread/thread.hpp>
#include <pcl/visualization/pcl_visualizer.h>


void pcd_clustering(char pcdname[30], char savename[30])
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile(pcdname, *cloud);

	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree2(new pcl::search::KdTree<pcl::PointXYZ>);
	tree2->setInputCloud(cloud);


	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;   //ŷʽ�������
	ec.setClusterTolerance(0.02);                     // ���ý��������������뾶Ϊ1.5cm
	ec.setMinClusterSize(4000);                 //����һ��������Ҫ�����ٵĵ���ĿΪ100
	ec.setMaxClusterSize(100000);               //����һ��������Ҫ��������ĿΪ25000
	ec.setSearchMethod(tree2);                    //���õ��Ƶ���������
	ec.setInputCloud(cloud);
	ec.extract(cluster_indices);           //�ӵ�������ȡ���࣬������������������cluster_indices��
	
	pcl::PCDWriter writer;

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_class(new pcl::PointCloud<pcl::PointXYZ>);
	//�������ʵ�������cluster_indices,ֱ���ָ���о���
	int j = 0;
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
	{ 
		std::cout << "Clustering" << std::endl;
		//���������еĵ��Ƶ����������ҷֿ����������ĵ���
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
		for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
			//���ñ�����Ƶ���������
			cloud_cluster->points.push_back(cloud->points[*pit]); //*
		cloud_cluster->width = cloud_cluster->points.size();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;

		std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size() << " data points." << std::endl;
		writer.write<pcl::PointXYZ>(savename, *cloud_cluster, false); //*
		cloud_class = cloud_cluster;
		j++;

		break;///
		
	}
}



#endif