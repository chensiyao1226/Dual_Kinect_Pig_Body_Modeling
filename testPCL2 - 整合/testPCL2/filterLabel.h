#pragma once

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
typedef unsigned int Label;
pcl::PointCloud<pcl::PointXYZL>::Ptr filterLabel(Label label, char pcdname[30], char savename[30]);