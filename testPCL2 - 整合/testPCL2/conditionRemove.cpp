#ifndef CONDITIONREMOVE_H
#define CONDITIONREMOVE_H

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/conditional_removal.h>
#include<pcl/visualization//pcl_visualizer.h>
#include<boost/thread/thread.hpp>

typedef pcl::PointXYZL PointT;

void conditionRemove(char filename[30], char savename[30], float min_x, float max_x, float min_y, float max_y, float min_z, float max_z)
{
	pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
	pcl::io::loadPCDFile(filename, *cloud);

	pcl::PointCloud<PointT>::Ptr cloud_extracted(new pcl::PointCloud<PointT>);


	pcl::ConditionOr<PointT>::Ptr range_cond(new pcl::ConditionOr<PointT>());
	range_cond->addComparison(pcl::FieldComparison<PointT>::ConstPtr(new pcl::FieldComparison<PointT>("y", pcl::ComparisonOps::GT, min_y)));
	range_cond->addComparison(pcl::FieldComparison<PointT>::ConstPtr(new pcl::FieldComparison<PointT>("y", pcl::ComparisonOps::LT, max_y)));
	range_cond->addComparison(pcl::FieldComparison<PointT>::ConstPtr(new pcl::FieldComparison<PointT>("x", pcl::ComparisonOps::GT, min_x)));
	range_cond->addComparison(pcl::FieldComparison<PointT>::ConstPtr(new pcl::FieldComparison<PointT>("x", pcl::ComparisonOps::LT, max_x)));
	range_cond->addComparison(pcl::FieldComparison<PointT>::ConstPtr(new pcl::FieldComparison<PointT>("z", pcl::ComparisonOps::GT, min_z)));
	range_cond->addComparison(pcl::FieldComparison<PointT>::ConstPtr(new pcl::FieldComparison<PointT>("z", pcl::ComparisonOps::LT, max_z)));
	// build the filter
	pcl::ConditionalRemoval<PointT> condrem;
	condrem.setCondition(range_cond);
	condrem.setInputCloud(cloud);
	condrem.setKeepOrganized(true);
	// apply filter
	condrem.filter(*cloud_extracted);

	// 保存下采样后的点云
	pcl::PCDWriter writer;
	writer.write<PointT>(savename, *cloud_extracted, false);

}

#endif