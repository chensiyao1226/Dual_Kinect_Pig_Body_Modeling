#ifndef CPCCLUSTER_H
#define CPCCLUSTER_H


#include <pcl/console/parse.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/supervoxel_clustering.h>
#include <vtkPolyLine.h>    //  �����Ҫ��ӣ�����ᱨ��
#include <pcl/segmentation/cpc_segmentation.h>

typedef pcl::PointXYZ PointT;

void CPCcluster()
{
	//�趨�ᾧ����
	float voxel_resolution = 0.008f;
	float seed_resolution = 0.1f;
	float color_importance = 0.2f;
	float spatial_importance = 0.4f;
	float normal_importance = 1.0f;

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile("pig_top(clu).pcd", *cloud);

	//���ɽᾧ��
	pcl::SupervoxelClustering<PointT> super(voxel_resolution, seed_resolution);
	//�͵�����ʽ�й�
	//if (disable_transform)
	//	super.setUseSingleCameraTransform(false);
	//������Ƽ��ᾧ����
	super.setInputCloud(cloud);
	super.setColorImportance(color_importance);
	super.setSpatialImportance(spatial_importance);
	super.setNormalImportance(normal_importance);
	//����ᾧ�ָ����������һ��ӳ���
	std::map <uint32_t, pcl::Supervoxel<PointT>::Ptr > supervoxel_clusters;
	super.extract(supervoxel_clusters);
	std::multimap<uint32_t, uint32_t> supervoxel_adjacency;
	super.getSupervoxelAdjacency(supervoxel_adjacency);



	//����CPC�ָ���
	pcl::CPCSegmentation<PointT>::CPCSegmentation seg;
	//���볬�������
	seg.setInputSupervoxels(supervoxel_clusters, supervoxel_adjacency);
	//���÷ָ����
	setCutting(max_cuts = 20,
		cutting_min_segments = 0,
		cutting_min_score = 0.16,
		locally_constrained = true,
		directed_cutting = true,
		clean_cutting = false)��
		seg.setRANSACIterations(ransac_iterations);
	seg.segment();
	seg.relabelCloud(pcl::PointCloud<pcl::PointXYZL> &labeled_cloud_arg);
}
#endif