#pragma once

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

void conditionRemove(char filename[30], char savename[30], float min_x, float max_x, float min_y, float max_y, float min_z, float max_z);