#ifndef MATREVERSE_H
#define MATREVERSE_H

#include "MatSave.h"
#include "MatRead.h"
#include <string>
#include <iostream>
#include <pcl/common/transforms.h>

void MatReverse(char matname[20])
{
	Eigen::Matrix4f Mat;
	Mat = MatRead(matname);

	Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();

	float theta = M_PI; // The angle of rotation in radians
	transform(0, 0) = cos(theta);
	transform(0, 2) = sin(theta);
	transform(1, 1) = 1;
	transform(2, 0) = -sin(theta);
	transform(2, 2) = cos(theta);
	transform(3, 3) = 1;

	transform = transform*Mat;
	MatSave(transform, matname);
	

}
#endif



