#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include "MatSave.h"
#include "MatRead.h"

void configuration()
{
	Eigen::Matrix4f Mat_Iden = Eigen::Matrix4f::Identity();
	MatSave(Mat_Iden, "Mat_Iden.txt");
	Eigen::Matrix4f transMatrix_tPCA = Eigen::Matrix4f::Identity();
	MatSave(transMatrix_tPCA, "Mat_TopPCA.txt");
}