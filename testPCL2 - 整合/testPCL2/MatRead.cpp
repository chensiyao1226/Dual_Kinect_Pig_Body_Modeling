#ifndef MATREAD_H
#define MATREAD_H
#include <Eigen/Core>
#include <fstream>

Eigen::Matrix4f MatRead(char Matname[10])
{
	Eigen::Matrix4f T;//创建储存多个矩阵的vector对象
	std::ifstream fin;
	fin.open(Matname);

	for (int i = 0; i < 4; i++)
		for (int j = 0; j < 4; j++)
			fin >> T(i, j);

	fin.close();

	return(T);
}

#endif