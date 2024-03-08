#ifndef PCD_LSMETHOD_H
#define PCD_LSMETHOD_H

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>  //kd-tree����������ඨ���ͷ�ļ�
#include <pcl/surface/mls.h>        //��С���˷�ƽ�������ඨ��ͷ�ļ�

void pcd_LSmethod(char pcdname[30], char savename[30])
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile(pcdname, *cloud);

	// ���� KD-Tree
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

	// Output has the PointNormal type in order to store the normals calculated by MLS
	pcl::PointCloud<pcl::PointNormal> mls_points;

	// ������С����ʵ�ֵĶ���mls
	pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;

	mls.setComputeNormals(true);  //��������С���˼�������Ҫ���з��߹���

	// Set parameters
	mls.setInputCloud(cloud);
	
	mls.setPolynomialFit(true);
	mls.setSearchMethod(tree);
	mls.setSearchRadius(0.05);
	// MovingLeastSquares<PointXYZ, PointXYZ> mls;
	// mls.setInputCloud(filtered);
	// mls.setSearchRadius(0.01);
	// mls.setPolynomialFit(true);
	// mls.setPolynomialOrder(2);
	// mls.setUpsamplingMethod(MovingLeastSquares<PointXYZ, PointXYZ>::SAMPLE_LOCAL_PLANE);
	// mls.setUpsamplingRadius(0.005);
	// mls.setUpsamplingStepSize(0.003);

	// PointCloud<PointXYZ>::Ptr cloud_smoothed (new PointCloud<PointXYZ>());
	// mls.process(*cloud_smoothed);
	// cout << "�ƶ���С����ƽ���˲����" << endl;

	// Reconstruct
	mls.process(mls_points);

	pcl::io::savePCDFile(savename, mls_points);

}
#endif