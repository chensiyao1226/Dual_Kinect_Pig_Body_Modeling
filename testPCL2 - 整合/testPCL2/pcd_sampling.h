#pragma once
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <boost/thread/thread.hpp>
#include <pcl/visualization/pcl_visualizer.h>
void pcd_sampling(char pcdname[30], char savename[30]);