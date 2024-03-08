#ifndef SURFACE_H
#define SURFACE_H

#include <iostream>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <pcl/features/normal_3d_omp.h>
#include <pcl/surface/mls.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/poisson.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/surface/bilateral_upsampling.h>
#include <pcl/visualization/pcl_visualizer.h>


using namespace std;
using namespace pcl;

void surface(char pcdname[30], char savename[30])
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile(pcdname, *cloud);

	MovingLeastSquares<PointXYZ, PointXYZ> mls;

	//declares the search method
	search::KdTree<PointXYZ>::Ptr tree(new search::KdTree<PointXYZ>);

	//configures the MLS
	mls.setInputCloud(cloud);
	mls.setPolynomialFit(true);
	mls.setSearchMethod(tree);
	mls.setSearchRadius(0.02);
	mls.setPolynomialOrder(2);
	mls.setUpsamplingMethod(MovingLeastSquares<PointXYZ, PointXYZ>::SAMPLE_LOCAL_PLANE);
	mls.setUpsamplingRadius(0.005);
	mls.setUpsamplingStepSize(0.003);
	cout << "smoothing..." << endl;
	//Estimates the normals and smoothes the surface
	PointCloud<PointXYZ>::Ptr cloud_smoothed(new PointCloud<PointXYZ>());
	mls.process(*cloud_smoothed);

	NormalEstimationOMP<PointXYZ, pcl::Normal> ne;
	ne.setNumberOfThreads(8);
	ne.setInputCloud(cloud_smoothed);
	ne.setRadiusSearch(0.01);
	Eigen::Vector4f centroid;
	compute3DCentroid(*cloud_smoothed, centroid);
	ne.setViewPoint(centroid[0], centroid[1], centroid[2]);

	//Estimates normals
	PointCloud<pcl::Normal>::Ptr cloud_normals(new PointCloud<pcl::Normal>());
	ne.compute(*cloud_normals);

	for (size_t i = 0; i < cloud_normals->size(); ++i){
		cloud_normals->points[i].normal_x *= -1;
		cloud_normals->points[i].normal_y *= -1;
		cloud_normals->points[i].normal_z *= -1;
	}

	PointCloud<PointNormal>::Ptr cloud_smoothed_normals(new PointCloud<PointNormal>());
	concatenateFields(*cloud_smoothed, *cloud_normals, *cloud_smoothed_normals);

	//Performs the poisson reconstruction
	Poisson<PointNormal> poisson;
	poisson.setDepth(8);
	poisson.setInputCloud(cloud_smoothed_normals);
	PolygonMesh mesh;

	cout << "reconstructing..." << endl;
	poisson.reconstruct(mesh);

	pcl::io::savePLYFile(savename, mesh);

	// ÏÔÊ¾½á¹ûÍ¼
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addPolygonMesh(mesh, "my");
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();
	while (!viewer->wasStopped()){
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
}
#endif