#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
void pcd_rotation(char pcdname[30], char savename[30])
{
	//定义原点云对象和变换后点云对象
	pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>());

	// 定义读取对象
	pcl::PCDReader reader;
	// 读取点云文件
	reader.read<pcl::PointXYZ>(pcdname, *source_cloud);


	//  Reminder: how transformation matrices work :

	//  |-------> This column is the translation
	//  | 1 0 0 x |  \
	//  | 0 1 0 y |   }-> The identity 3x3 matrix (no rotation) on the left
	//  | 0 0 1 z |  /
	//  | 0 0 0 1 |    -> We do not use this line (and it has to stay 0,0,0,1)

	// METHOD : Using a Matrix4f
	//This is the "manual" method, perfect to understand but error prone !
	//生成一个4*4的矩阵
	Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();

	/*
	// Define a rotation matrix 定义水平翻转矩阵
	float theta = M_PI; // The angle of rotation in radians
	transform(0, 0) = cos(theta);
	transform(0, 2) = sin(theta);
	transform(1, 1) = 1;
	transform(2, 0) = -sin(theta);
	transform(2, 2) = cos(theta);
	transform(3, 3) = 1;
	*/
	// Set a rotation around the Z axis (right hand rule).
	float theta = 90.0f * (M_PI / 180.0f); // 90 degrees.
	transform(0, 0) = cos(theta);
	transform(0, 1) = -sin(theta);
	transform(1, 0) = sin(theta);
	transform(1, 1) = cos(theta);

	// 利用生成的变换矩阵对点云进行变换处理
	pcl::transformPointCloud(*source_cloud, *transformed_cloud, transform);

	//将新生成的点云写入文件
	pcl::PCDWriter writer;
	writer.write<pcl::PointXYZ>(savename, *transformed_cloud, false);

}