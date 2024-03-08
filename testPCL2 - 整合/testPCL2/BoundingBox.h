#pragma once
#include <Eigen/Core>
#include <iostream>
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
typedef pcl::PointXYZL PointType;
Eigen::Matrix4f BoundingBox(pcl::PointCloud<PointType>::Ptr cloud);