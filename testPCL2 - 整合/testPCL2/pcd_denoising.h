#pragma once
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
void pcd_denoising(char pcdname[30], char savename[30], double threshhold);