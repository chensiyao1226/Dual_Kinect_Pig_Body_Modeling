#ifndef PCD_DISPALY3_H
#define PCD_DISPLAY3_H

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include<boost/thread/thread.hpp>


void pcd_display3(char file1[30], char file2[30])
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile(file1, *cloud1);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile(file2, *cloud2);

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_final(new pcl::visualization::PCLVisualizer("3D Viewer"));
	//��Ŀ�������ɫ����ɫ�������ӻ�
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_color(cloud1, 255, 0, 0);
	viewer_final->addPointCloud<pcl::PointXYZ>(cloud1, target_color, "target cloud");
	viewer_final->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "target cloud");
	//��ת�����Ŀ�������ɫ����ɫ�������ӻ�
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> output_color(cloud2, 0, 255, 0);
	viewer_final->addPointCloud<pcl::PointXYZ>(cloud2, output_color, "output cloud");
	viewer_final->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "output cloud");
	// �������ӻ�
	viewer_final->addCoordinateSystem(1.0);
	viewer_final->initCameraParameters();
	//�ȴ�ֱ�����ӻ����ڹرա�
	while (!viewer_final->wasStopped())
	{
		viewer_final->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
}
#endif