
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
	//////////////////////////////��׼ ////////////////////////////////
	char* pcdArra[] = {"pig_top(����transHalfHB).pcd", "pig_side(����transHalfHB).pcd"};//ѧһ�� �洢�����ļ�
	Eigen::Matrix4f Transform = Registration(40, 35, 40, 0.01, true, 2, pcdArra);//�˶�ʵ��(40, 35, 40, 0.01, true, 2 Transform=Registration(50, 40, 50, 0.01, 2, pcdArra);
	printMatrix(Transform);

	MatSave(Transform, "Mat_Reg.txt");

	merge("pig_side(����transHalfHB).pcd", "pig_top(����transHalfHB).pcd", "Mat_Reg.txt",
	"pig_transformed_side.pcd","pig_fusionBF_part.pcd");

	pcd_display3("pig_transformed_side.pcd", "pig_top(����transHalfHB).pcd");

	merge("pig_side(��С����trans).pcd", "pig_top(��С����trans).pcd", "Mat_Reg.txt",
	"pig_side(Reg).pcd", "pig_fusion_integral.pcd");//�õ�������������Ƶ���׼����
	pcd_display3("pig_side(Reg).pcd", "pig_top(��С����trans).pcd");
	





	////top�ü�////
	//Ϊ��ȡ���岿�ּ�ֵ
	float min_z_topReg, max_z_topReg;
	peakExtra("pig_top(��С����trans).pcd", 2, min_z_topReg, max_z_topReg);
	Sfilter2("pig_top(��С����trans).pcd", "z", min_z_topReg + (max_z_topReg - min_z_topReg) / 5, max_z_topReg - (max_z_topReg - min_z_topReg) / 5, "pig_top(RegUpBody).pcd");
	float min_x_topReg, max_x_topReg;
	peakExtra("pig_top(RegUpBody).pcd", 3, min_x_topReg, max_x_topReg);//���岿�����¼�ֵ
	//�õ��ص����֣������ؽ�ʱ���ص����� �����ü���һ�߲�ȫ��
	conditionRemove("pig_top(��С����trans).pcd", "pig_top(RegCut).pcd", 3, min_x_topReg + (max_x_topReg - min_x_topReg) * 2 / 3, 0, -3, 3, -3);

	////side�ü�////
	float min_x_sideReg, max_x_sideReg, min_z_sideReg, max_z_sideReg;
	peakExtra("pig_side(Reg).pcd", 3, min_x_sideReg, max_x_sideReg);
	peakExtra("pig_side(Reg).pcd", 2, min_z_sideReg, max_z_sideReg);
	pcd_display3("pig_side(RegCutT).pcd", "pig_top(RegCut).pcd");
	
	//�õ��ص����֣������ؽ�ʱ���ص����� �����ü���һ�߲�ȫ��
	conditionRemove("pig_side(Reg).pcd", "pig_side(RegCut).pcd", min_x_sideReg + (max_x_sideReg - min_x_sideReg) / 8, -3, 3, -3, max_z_sideReg - (max_z_sideReg - min_z_sideReg) / 5, -3);//



	/////ƴ�ϼ��ò���//////
	//pcd_display3("pig_side(RegCut).pcd", "pig_top(RegCut).pcd");
	merge("pig_side(RegCut).pcd", "pig_top(RegCut).pcd", "Mat_Iden.txt",
	"pig_side(RegCutT).pcd", "pig_integral.pcd");
	pcd_display3("pig_side(RegCutT).pcd", "pig_top(RegCut).pcd");

	
}