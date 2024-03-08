#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/console/parse.h>
#include <pcl/console/print.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>

#include "Registration.h"
#include "symmetrize.h"
#include "PLYtoPCD.h"
#include "surface.h"
#include "Sfilter2.h"
#include "conditionRemove.h"
#include "pcd_display.h"
#include "pcd_display2.h"
#include "pcd_display3.h"
#include "pcd_sampling.h"
#include "pcd_denoising.h"
#include "MatSave.h"
#include "merge.h"
#include "peakExtra.h"

void reconstruction()
{
	symmetrize("pig_integral.pcd", "pig_symmetrical.pcd");

	pcd_display3("pig_integral.pcd", "pig_symmetrical.pcd");


	//ֻ�����岿����׼
	pcl::PointCloud<pcl::PointXYZL>::Ptr cloud_symm(new pcl::PointCloud<pcl::PointXYZL>);
	pcl::io::loadPCDFile("pig_symmetrical.pcd", *cloud_symm);
	pcl::PointXYZL minPt_sym, maxPt_sym;
	pcl::getMinMax3D(*cloud_symm, minPt_sym, maxPt_sym);

	Sfilter2("pig_symmetrical.pcd", "x", minPt_sym.x + (maxPt_sym.x - minPt_sym.x) * 3 / 5, maxPt_sym.x , "pig_symmetrical(half).pcd");//��2/5������׼  minPt_sym.x, maxPt_sym.x - (maxPt_sym.x - minPt_sym.x) * 3 / 5
	Sfilter2("pig_symmetrical(half).pcd", "z", minPt_sym.z + (maxPt_sym.z - minPt_sym.z) / 6, maxPt_sym.z - (maxPt_sym.z - minPt_sym.z) / 6, "pig_symmetrical(Reg).pcd");//�ų����䡢β�ͽ���������׼

	pcl::PointCloud<pcl::PointXYZL>::Ptr cloud_inte(new pcl::PointCloud<pcl::PointXYZL>);
	pcl::io::loadPCDFile("pig_integral.pcd", *cloud_inte);
	pcl::PointXYZL minPt_int, maxPt_int;
	pcl::getMinMax3D(*cloud_inte, minPt_int, maxPt_int);

	Sfilter2("pig_integral.pcd", "y", -0.02, 3, "pig_integral(half1).pcd");
	Sfilter2("pig_integral(half1).pcd", "x", minPt_int.x + (maxPt_int.x - minPt_int.x) * 3 / 5, maxPt_int.x, "pig_integral(half2).pcd");//minPt_int.x, maxPt_int.x - (maxPt_int.x - minPt_int.x) * 2 / 5
	Sfilter2("pig_integral(half2).pcd", "z", minPt_sym.z + (maxPt_sym.z - minPt_sym.z) / 6, maxPt_sym.z - (maxPt_sym.z - minPt_sym.z) / 6, "pig_integral(Reg).pcd");//��λ��sym��ȫ��Ӧ

	pcd_display2("pig_integral(half1).pcd", "pig_integral(half2).pcd");

	pcd_display2("pig_integral(Reg).pcd", "pig_symmetrical(Reg).pcd");


	char* pcdArra2[] = {"pig_integral(Reg).pcd", "pig_symmetrical(Reg).pcd"};//
	Eigen::Matrix4f Transform2 = Registration(20, 10, 20, 0.05, false, 2, pcdArra2);//Transform=Registration(50, 40, 50, 0.01, 2, pcdArra);

	MatSave(Transform2, "Mat_Sym.txt");
	merge("pig_symmetrical(Reg).pcd", "pig_integral(Reg).pcd", "Mat_Sym.txt",
	"pig_sym(transReg).pcd", "pig_complete(Reg).pcd");
	pcd_display3("pig_integral(Reg).pcd", "pig_sym(transReg).pcd");

	float min_x_topReg, max_x_topReg;
	peakExtra("pig_top(RegUpBody).pcd", 3, min_x_topReg, max_x_topReg);//���岿�����¼�ֵ
	float min_x_sideReg, max_x_sideReg, min_z_sideReg, max_z_sideReg;
	peakExtra("pig_side(Reg).pcd", 3, min_x_sideReg, max_x_sideReg);
	peakExtra("pig_side(Reg).pcd", 2, min_z_sideReg, max_z_sideReg);
	//�õ��ص����֣������ؽ�ʱ���ص����� �����ü���һ�߲�ȫ��

	////integral top�ü�////
	//�õ��ص����֣������ؽ�ʱ���ص����� �����ü���һ�߲�ȫ��
	conditionRemove("pig_integral.pcd", "pig_integral(Cut1).pcd", 3, min_x_topReg + (max_x_topReg - min_x_topReg)*2 / 3, 3, 0, 3, -3);
	conditionRemove("pig_integral(Cut1).pcd", "pig_integral(Cut).pcd", max_x_topReg, -3, 3, 0, 3, -3);//��ȥ�²�Խ���Գ��Ჿ�� �Լ����ؽ����
	//pcd_display3("pig_integral.pcd", "pig_integral(Cut).pcd");

	////symmetrical side�ü�////
	//�õ��ص����֣������ؽ�ʱ���ص�����
	conditionRemove("pig_symmetrical.pcd", "pig_symmetrical(Cut).pcd", min_x_sideReg + (max_x_sideReg - min_x_sideReg) / 8, -3, 3, -3, max_z_sideReg - (max_z_sideReg - min_z_sideReg) / 5, -3);//
	//pcd_display3("pig_symmetrical.pcd", "pig_symmetrical(Cut).pcd");


	merge("pig_symmetrical(Cut).pcd", "pig_integral(Cut).pcd", "Mat_Sym.txt",
	"pig_sym(trans).pcd", "pig_complete.pcd");


	pcd_sampling("pig_complete.pcd", "pig_complete(sampled).pcd");

	pcd_denoising("pig_complete(sampled).pcd", "pig_complete(ȥ��).pcd", 2);
	pcd_display3("pig_complete(sampled).pcd", "pig_complete(ȥ��).pcd");
	pcd_display("pig_complete(ȥ��).pcd", "pig_complete(sampled).pcd",0);
	

	
	////////////////////��ά�ؽ�/////////////////////
	surface("pig_complete(ȥ��).pcd", "pig(�ؽ�).ply");
	//pois_recons("pig_complete(sampled).pcd", "pig(�ؽ�).ply");
}