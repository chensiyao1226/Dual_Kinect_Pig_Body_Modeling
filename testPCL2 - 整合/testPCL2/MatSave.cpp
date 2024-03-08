#ifndef MATSAVE_H
#define MATSAVE_H
#include <Eigen/Core>
#include <fstream>

void MatSave(Eigen::Matrix4f matrix, char Matname[10])
{
	std::ofstream fout;
	fout.open(Matname);//在文件末尾追加写入
	fout << matrix << std::endl;//每次写完一个矩阵以后换行
	fout.close();
}

#endif