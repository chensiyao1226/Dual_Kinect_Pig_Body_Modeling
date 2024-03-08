#ifndef TRANSCENTER_H
#define TRANSCENTER_H

#include <vtkAutoInit.h>         


#include <iostream>
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Core>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace std;
typedef pcl::PointXYZL PointType;

Eigen::Matrix4f TransCenter(char pcdname[30], char savename[30])
{
	pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
	pcl::io::loadPCDFile(pcdname, *cloud);

	Eigen::Vector4f pcaCentroid;//����
	pcl::compute3DCentroid(*cloud, pcaCentroid);
	Eigen::Matrix3f covariance;
	pcl::computeCovarianceMatrixNormalized(*cloud, pcaCentroid, covariance);
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
	Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();//��������
	Eigen::Vector3f eigenValuesPCA = eigen_solver.eigenvalues();//����ֵ
	eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1)); //У��������䴹ֱ
	eigenVectorsPCA.col(0) = eigenVectorsPCA.col(1).cross(eigenVectorsPCA.col(2));
	eigenVectorsPCA.col(1) = eigenVectorsPCA.col(2).cross(eigenVectorsPCA.col(0));


	Eigen::Matrix4f tm = Eigen::Matrix4f::Identity();//��λ����
	tm.block<3, 3>(0, 0) = eigenVectorsPCA.transpose();   //R.
	tm.block<3, 1>(0, 3) = -1.0f * (eigenVectorsPCA.transpose()) *(pcaCentroid.head<3>());//  -R*t

	pcl::PointCloud<PointType>::Ptr transformedCloud(new pcl::PointCloud<PointType>);
	pcl::transformPointCloud(*cloud, *transformedCloud, tm);


	pcl::PCDWriter writer;
	//writer.write<PointType>("pig_top(labelEx).pcd", *transformedCloud, false);
	writer.write<PointType>(savename, *transformedCloud, false);

	return(tm);

}
#endif