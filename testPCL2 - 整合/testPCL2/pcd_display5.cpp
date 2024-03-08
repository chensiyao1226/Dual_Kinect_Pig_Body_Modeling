#ifndef PCD_DISPALY5_H
#define PCD_DISPLAY5_H

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include<boost/thread/thread.hpp>


void pcd_display5(float k1, char pointTop1[30], float k2, char pointTop2[30], float z, char pointMarked[30], char cloudfile[30])
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr point_Top1(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile(pointTop1, *point_Top1);
	pcl::PointXYZ p1 = point_Top1->points[0];

	pcl::PointCloud<pcl::PointXYZ>::Ptr point_Top2(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile(pointTop2, *point_Top2);
	pcl::PointXYZ p3 = point_Top2->points[0];

	pcl::PointCloud<pcl::PointXYZ>::Ptr point_Marked(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile(pointMarked, *point_Marked);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile(cloudfile, *cloud);

	//y=k(x-x1)+y1  
	float y1 = k1*(z - p1.z) + p1.y;
	pcl::PointXYZ p2(0,y1,z);
	float y2 = k2*(z - p3.z) + p3.y;
	pcl::PointXYZ p4(0, y2, z);

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_final(new pcl::visualization::PCLVisualizer("Line Viewer"));
	//对目标点云着色（红色）并可视化
	viewer_final->addLine<pcl::PointXYZ>(p2, p1, 255, 0, 0, "Envelope line"); //红色线段
	viewer_final->addLine<pcl::PointXYZ>(p4, p3, 255, 0, 0, "Envelope line2"); //红色线段

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_color(point_Marked, 255, 0, 0);
	viewer_final->addPointCloud<pcl::PointXYZ>(point_Marked, target_color, "target point");
	viewer_final->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "target point");

	//对转换后的目标点云着色（绿色）并可视化
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> output_color(cloud, 0, 255, 0);
	viewer_final->addPointCloud<pcl::PointXYZ>(cloud, output_color, "output cloud");
	viewer_final->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "output cloud");
	// 启动可视化
	//viewer_final->addCoordinateSystem(1.0);
	viewer_final->initCameraParameters();
	//等待直到可视化窗口关闭。
	while (!viewer_final->wasStopped())
	{
		viewer_final->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
}
#endif