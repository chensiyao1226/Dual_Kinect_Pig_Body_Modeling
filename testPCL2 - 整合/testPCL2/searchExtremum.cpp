#ifndef SEARCHEXTREMUM_H
#define SEARCHEXTREMUM_H

#include <iostream>
#include <vector>
#include <ctime>
#include <boost/thread/thread.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/features/eigen.h>
#include <pcl/features/feature.h>
#include <pcl/features/normal_3d.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/features/boundary.h>
#include <pcl/visualization/cloud_viewer.h>


using namespace std;

int searchExtremum(char pcdname[30], pcl::PointXYZ &extremum, float endLength, float extraExplore, int dir, int feild, int biggest)//�������򡢱Ƚϵ�x/yֵ���Ƿ񼫴�ֵ/��Сֵ
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile(pcdname, *cloud);

	float afterPeak = 0;
	int index = 0;
	float start = extremum.z;
	
	for (float position = extremum.z; position*dir <= (start + endLength*dir)*dir; position += 0.01*dir, afterPeak += 0.01)
	{
		if (afterPeak > extraExplore)//����������̷��ּ���ֵ��̽����Χ
			break;
		for (size_t i = 0; i < cloud->points.size(); ++i)
		{
			if (cloud->points[i].z >= position  &&  cloud->points[i].z < position + 0.01)
			{
				if (feild == 1)
				{
					if (((cloud->points[i].x)*biggest >(extremum.x*biggest + 0.01)) || ((cloud->points[i].x)*biggest >(extremum.x*biggest + 1 / 5 * afterPeak)))
					{
						extremum = cloud->points[i];
						index = i;
						afterPeak = 0;//���þ������ֵ�ľ���
					}
				}
				else if (feild == 2)
				{
					if (((cloud->points[i].y)*biggest > (extremum.y*biggest + 0.03)) || ((cloud->points[i].y)*biggest > (extremum.y*biggest + 1/5*afterPeak)))//������߼�ֵ�ص�б��=1/5����
					{
						extremum = cloud->points[i];
						index = i;
						afterPeak = 0;//���þ������ֵ�ľ���
					}
				}
				else if (feild == 3)
				{
					if ((cloud->points[i].z)*biggest > extremum.z*biggest)
					{
						extremum = cloud->points[i];
						index = i;
						afterPeak = 0;//���þ������ֵ�ľ���
					}
				}
				
			}	
		}	
	}
	cout << "extremum:(" << extremum.x << "," << extremum.y << "," << extremum.z << ")" << endl;
	return (index);

}
#endif