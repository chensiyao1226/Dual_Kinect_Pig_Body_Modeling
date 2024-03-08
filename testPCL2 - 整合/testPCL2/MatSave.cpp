#ifndef MATSAVE_H
#define MATSAVE_H
#include <Eigen/Core>
#include <fstream>

void MatSave(Eigen::Matrix4f matrix, char Matname[10])
{
	std::ofstream fout;
	fout.open(Matname);//���ļ�ĩβ׷��д��
	fout << matrix << std::endl;//ÿ��д��һ�������Ժ���
	fout.close();
}

#endif