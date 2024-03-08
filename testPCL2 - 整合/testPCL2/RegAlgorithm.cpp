
#include "Registration.h"
#include "merge.h"
#include "printMatrix.h"
#include "MatSave.h"
#include "MatRead.h"
#include "peakExtra.h"
#include "Sfilter2.h"
#include "conditionRemove.h"
#include "pcd_display2.h"
#include "pcd_display3.h"

void RegAlgorithm()
{
	//////////////////////////////配准 ////////////////////////////////
	char* pcdArra[] = {"pig_top(二乘transHalfHB).pcd", "pig_side(二乘transHalfHB).pcd"};//学一下 存储矩阵文件
	Eigen::Matrix4f Transform = Registration(40, 35, 40, 0.01, true, 2, pcdArra);//运动实验(40, 35, 40, 0.01, true, 2 Transform=Registration(50, 40, 50, 0.01, 2, pcdArra);
	printMatrix(Transform);

	MatSave(Transform, "Mat_Reg.txt");

	merge("pig_side(二乘transHalfHB).pcd", "pig_top(二乘transHalfHB).pcd", "Mat_Reg.txt",
	"pig_transformed_side.pcd","pig_fusionBF_part.pcd");

	pcd_display3("pig_transformed_side.pcd", "pig_top(二乘transHalfHB).pcd");

	merge("pig_side(最小二乘trans).pcd", "pig_top(最小二乘trans).pcd", "Mat_Reg.txt",
	"pig_side(Reg).pcd", "pig_fusion_integral.pcd");//得到两部分整体点云的配准点云
	pcd_display3("pig_side(Reg).pcd", "pig_top(最小二乘trans).pcd");
	





	////top裁剪////
	//为提取主体部分极值
	float min_z_topReg, max_z_topReg;
	peakExtra("pig_top(最小二乘trans).pcd", 2, min_z_topReg, max_z_topReg);
	Sfilter2("pig_top(最小二乘trans).pcd", "z", min_z_topReg + (max_z_topReg - min_z_topReg) / 5, max_z_topReg - (max_z_topReg - min_z_topReg) / 5, "pig_top(RegUpBody).pcd");
	float min_x_topReg, max_x_topReg;
	peakExtra("pig_top(RegUpBody).pcd", 3, min_x_topReg, max_x_topReg);//主体部分上下极值
	//裁掉重叠部分，减少重建时的重叠干扰 但不裁剪另一边补全用
	conditionRemove("pig_top(最小二乘trans).pcd", "pig_top(RegCut).pcd", 3, min_x_topReg + (max_x_topReg - min_x_topReg) * 2 / 3, 0, -3, 3, -3);

	////side裁剪////
	float min_x_sideReg, max_x_sideReg, min_z_sideReg, max_z_sideReg;
	peakExtra("pig_side(Reg).pcd", 3, min_x_sideReg, max_x_sideReg);
	peakExtra("pig_side(Reg).pcd", 2, min_z_sideReg, max_z_sideReg);
	pcd_display3("pig_side(RegCutT).pcd", "pig_top(RegCut).pcd");
	
	//裁掉重叠部分，减少重建时的重叠干扰 但不裁剪另一边补全用
	conditionRemove("pig_side(Reg).pcd", "pig_side(RegCut).pcd", min_x_sideReg + (max_x_sideReg - min_x_sideReg) / 8, -3, 3, -3, max_z_sideReg - (max_z_sideReg - min_z_sideReg) / 5, -3);//



	/////拼合剪裁部分//////
	//pcd_display3("pig_side(RegCut).pcd", "pig_top(RegCut).pcd");
	merge("pig_side(RegCut).pcd", "pig_top(RegCut).pcd", "Mat_Iden.txt",
	"pig_side(RegCutT).pcd", "pig_integral.pcd");
	pcd_display3("pig_side(RegCutT).pcd", "pig_top(RegCut).pcd");

	
}