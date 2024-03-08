#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
int searchIndex(char pcdname[30], float x_min, float x_max, float y_min, float y_max, float z_min, float z_max, pcl::PointXYZ &p);