#ifndef BOUNDINGBOX_H
#define BOUNDINGBOX_H

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

Eigen::Matrix4f BoundingBox(pcl::PointCloud<PointType>::Ptr cloud)
{
	//����һ������ ����boundingbox ��Ϊ  TransCenter
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

	std::cout << "characteristic value va(3x1):\n" << eigenValuesPCA << std::endl;
	std::cout << "feature vector ve(3x3):\n" << eigenVectorsPCA << std::endl;
	std::cout << "centroid(4x1):\n" << pcaCentroid << std::endl;
	/*
	// ��һ�ּ������Э�����������ֵ�����������ķ�ʽ:ͨ��pcl�е�pca�ӿڣ����£���������õ�����������������������
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPCAprojection (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PCA<pcl::PointXYZ> pca;
	pca.setInputCloud(cloudSegmented);
	pca.project(*cloudSegmented, *cloudPCAprojection);
	std::cerr << std::endl << "EigenVectors: " << pca.getEigenVectors() << std::endl;//������������
	std::cerr << std::endl << "EigenValues: " << pca.getEigenValues() << std::endl;//��������ֵ
	*/
	Eigen::Matrix4f tm = Eigen::Matrix4f::Identity();
	Eigen::Matrix4f tm_inv = Eigen::Matrix4f::Identity();
	tm.block<3, 3>(0, 0) = eigenVectorsPCA.transpose();   //R.
	tm.block<3, 1>(0, 3) = -1.0f * (eigenVectorsPCA.transpose()) *(pcaCentroid.head<3>());//  -R*t
	tm_inv = tm.inverse();

	std::cout << "Transformation matrix tm(4x4):\n" << tm << std::endl;
	std::cout << "Inverting matrix tm'(4x4):\n" << tm_inv << std::endl;

	pcl::PointCloud<PointType>::Ptr transformedCloud(new pcl::PointCloud<PointType>);
	pcl::transformPointCloud(*cloud, *transformedCloud, tm);

	PointType min_p1, max_p1;
	Eigen::Vector3f c1, c;
	pcl::getMinMax3D(*transformedCloud, min_p1, max_p1);
	c1 = 0.5f*(min_p1.getVector3fMap() + max_p1.getVector3fMap());

	std::cout << "Core c1(3x1):\n" << c1 << std::endl;

	Eigen::Affine3f tm_inv_aff(tm_inv);
	pcl::transformPoint(c1, c, tm_inv_aff);

	Eigen::Vector3f whd, whd1;
	whd1 = max_p1.getVector3fMap() - min_p1.getVector3fMap();
	whd = whd1;
	float sc1 = (whd1(0) + whd1(1) + whd1(2)) / 3;  //����ƽ���߶ȣ����������������ͷ��С

	std::cout << "width1=" << whd1(0) << endl;
	std::cout << "heght1=" << whd1(1) << endl;
	std::cout << "depth1=" << whd1(2) << endl;
	std::cout << "scale1=" << sc1 << endl;

	const Eigen::Quaternionf bboxQ1(Eigen::Quaternionf::Identity());
	const Eigen::Vector3f    bboxT1(c1);

	const Eigen::Quaternionf bboxQ(tm_inv.block<3, 3>(0, 0));
	const Eigen::Vector3f    bboxT(c);


	//�任��ԭ��ĵ���������
	PointType op;
	op.x = 0.0;
	op.y = 0.0;
	op.z = 0.0;
	Eigen::Vector3f px, py, pz;
	Eigen::Affine3f tm_aff(tm);
	pcl::transformVector(eigenVectorsPCA.col(0), px, tm_aff);
	pcl::transformVector(eigenVectorsPCA.col(1), py, tm_aff);
	pcl::transformVector(eigenVectorsPCA.col(2), pz, tm_aff);
	PointType pcaX;
	pcaX.x = sc1 * px(0);
	pcaX.y = sc1 * px(1);
	pcaX.z = sc1 * px(2);
	PointType pcaY;
	pcaY.x = sc1 * py(0);
	pcaY.y = sc1 * py(1);
	pcaY.z = sc1 * py(2);
	PointType pcaZ;
	pcaZ.x = sc1 * pz(0);
	pcaZ.y = sc1 * pz(1);
	pcaZ.z = sc1 * pz(2);


	pcl::PCDWriter writer;
	//writer.write<pcl::PointXYZL>("pig_top(labelPCA).pcd", *transformedCloud, false);
	writer.write<pcl::PointXYZL>("pig_side(labelPCA).pcd", *transformedCloud, false);
	return(tm);
	
	
	//visualization
	pcl::visualization::PCLVisualizer viewer;

	pcl::visualization::PointCloudColorHandlerCustom<PointType> tc_handler(transformedCloud, 0, 255, 0); //ת����ԭ��ĵ������
	viewer.addPointCloud(transformedCloud, tc_handler, "transformCloud");
	viewer.addCube(bboxT1, bboxQ1, whd1(0), whd1(1), whd1(2), "bbox1");
	viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "bbox1");
	viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "bbox1");

	viewer.addArrow(pcaX, op, 1.0, 0.0, 0.0, false, "arrow_X");
	viewer.addArrow(pcaY, op, 0.0, 1.0, 0.0, false, "arrow_Y");
	viewer.addArrow(pcaZ, op, 0.0, 0.0, 1.0, false, "arrow_Z");

	viewer.addCoordinateSystem(0.5f*sc1);
	while (!viewer.wasStopped())
	{
		viewer.spinOnce(100);
	}
	

}
#endif