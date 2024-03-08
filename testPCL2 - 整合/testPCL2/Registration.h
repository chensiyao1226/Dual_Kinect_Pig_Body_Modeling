#pragma once
#include <Eigen/Core>
Eigen::Matrix4f Registration(double radius_normal, double radius_feature, double radius_icp,
	double min_correspondence_dist, bool sacia, int argc, char** argv);