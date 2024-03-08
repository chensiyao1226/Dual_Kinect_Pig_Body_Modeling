#ifndef HARRIS_H
#define HARRIS_H

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/keypoints/harris_3d.h>
#include <boost/thread/thread.hpp>
#include <stdlib.h>
#include <iostream>

using namespace std;
typedef pcl::PointXYZ PointType;

void Harris(char pcdname[30], char savename[30])
{
	// ------------------------------------------------------------------
	// -----Read pcd file or create example point cloud if not given-----
	// ------------------------------------------------------------------
	cout << "Harris" << endl;
	//读取pcd文件；如果没有指定文件，就创建样本点
	pcl::PointCloud<PointType>::Ptr cloud_ptr(new pcl::PointCloud<PointType>);//点云对象指针
	pcl::PointCloud<PointType>& cloud = *cloud_ptr;//引用　上面点云的别名　常亮指针
	pcl::io::loadPCDFile<PointType>(pcdname, cloud);


	// --------------------------------
	// -----Extract Harri keypoints-----
	// --------------------------------
	// 提取Harri关键点
	pcl::HarrisKeypoint3D<pcl::PointXYZ, pcl::PointXYZI, pcl::Normal> harris;
	harris.setInputCloud(cloud_ptr);//设置输入点云 指针
	harris.setNonMaxSupression(true);
	harris.setRadius(0.002f);//　块体半径0.02f
	harris.setThreshold(0.02f);//数量阈值0.01f
	cout << "parameter set successful" << endl;

	//新建的点云必须初始化，清零，否则指针会越界
	//注意Harris的输出点云必须是有强度(I)信息的 pcl::PointXYZI，因为评估值保存在I分量里
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_out_ptr(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI>& cloud_out = *cloud_out_ptr;
	cloud_out.height = 1;
	cloud_out.width = 100;
	cloud_out.resize(cloud_out.height * cloud_out.width);
	cloud_out.clear();
	cout << "extracting... " << endl;
	// 计算特征点
	harris.compute(cloud_out);


	int size = cloud_out.size();
	cout << "extraction : " << size << " n keypoints." << endl;

	pcl::PCDWriter writer;
	writer.write<pcl::PointXYZI>(savename, cloud_out, false);

	// --------------------------------------------
	// -----Open 3D viewer and add point cloud-----
	// --------------------------------------------
	// 3D点云显示
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_show(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud(*cloud_out_ptr, *cloud_show);

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer);//可视化对象指针
	viewer->addPointCloud(cloud_ptr);//指针
	//在3D图形窗口中显示关键点
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> harris_color_handler(cloud_show, 0, 255, 0);//第一个参数类型为　指针
	viewer->addPointCloud(cloud_show, harris_color_handler, "harris");//第一个参数类型为　指针
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "harris");


	while (!viewer->wasStopped())
	{
		viewer->spinOnce();
		pcl_sleep(0.1);
	}

}

#endif