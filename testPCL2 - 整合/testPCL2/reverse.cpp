
#include <string>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
using namespace std;
typedef pcl::PointXYZ PointT;

void reverse(char pcdname[30], char savename[30])
{
	pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr sym_cloud(new pcl::PointCloud<PointT>);
	pcl::io::loadPCDFile(pcdname, *cloud);

	sym_cloud->points.resize(cloud->points.size());
	sym_cloud->width = sym_cloud->points.size();
	sym_cloud->height = 1;

	for (size_t i = 0; i < cloud->points.size(); ++i)
	{
		sym_cloud->points[i].x = -(cloud->points[i].x);
		sym_cloud->points[i].y = cloud->points[i].y;
		sym_cloud->points[i].z = cloud->points[i].z;

	}

	pcl::PCDWriter writer;
	writer.write<PointT>(savename, *sym_cloud, false);
}




