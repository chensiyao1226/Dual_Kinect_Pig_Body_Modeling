#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <iostream>

#include "pcd_display2.h"
#include "pcd_rotation.h"
#include "CPCsegmen.h"
#include "LabelExtr.h"
#include "filterLabel.h"
#include "TransCenter.h"
#include "Sfilter2.h"
#include "MatTrans.h"
#include "peakExtra.h"
#include "MatSave.h"
#include "MatRead.h"
#include "reverseZ.h"
#include "conditionRemove.h"
#include "signDir.h"
#include "JudgeDir.h"
#include "pcd_display3.h"
#include "MatReverse.h"
#include "merge.h"
#include "Upright1.h"
#include "Upright2.h"
#include "Upright3.h"
#include "getNormal.h"
#include "printMatrix.h"
#include "createNormal.h"
#include "reverse.h"

void regPosRegulation()
{
	pcl::PCDWriter writer;
	Eigen::Matrix4f Mat_Iden = Eigen::Matrix4f::Identity();

	//////////////////////////pigside תxyz////////////////////////////
	//////Normal-��xyz
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_Nor2(new pcl::PointCloud<pcl::PointNormal>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz2(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile("pig_side(��С����).pcd", *cloud_Nor2);
	pcl::copyPointCloud(*cloud_Nor2, *cloud_xyz2);
	writer.write<pcl::PointXYZ>("pig_side(��С����xyzR).pcd", *cloud_xyz2, false);
	reverse("pig_side(��С����xyzR).pcd", "pig_side(��С����xyz).pcd");
	//pcd_display2("pig_side(��С����xyzR).pcd", "pig_side(��С����xyz).pcd");
	/////////////////////////pigtop ������� ͷ�����//////////////////////////

	//////Normal-��xyz
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_Nor(new pcl::PointCloud<pcl::PointNormal>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile("pig_top(��С����).pcd", *cloud_Nor);
	pcl::copyPointCloud(*cloud_Nor, *cloud_xyz);

	writer.write<pcl::PointXYZ>("pig_top(��С����xyzR).pcd", *cloud_xyz, false);
	reverse("pig_top(��С����xyzR).pcd", "pig_top(��С����xyz).pcd");
	//pcd_display2("pig_top(��С����xyzR).pcd", "pig_top(��С����xyz).pcd");

	//C:\Users\chen\Documents\Visual Studio 2013\Projects\testPCL2\x64\Debug\testPCL2.exe pig_top(��С����).pcd -cut 3,2000,0.4 -o
	char* command[] = { "C:\Users\chen\Documents\Visual Studio 2013\Projects\testPCL2\x64\Debug\testPCL2.exe","pig_top(��С����xyz).pcd", "-cut", "8,300,0.05", "-o", "-add"};
	int argco = 6;
	CPCsegmen(argco, command);


	////��ȡ����////
	unsigned int label;
	label = LabelExtr("pig_top(��С����xyz)_out.pcd");
	cout << label << endl;
	filterLabel(label, "pig_top(��С����xyz)_out.pcd", "pig_top(labExtracted).pcd");//labExtracted--xyzl
	//pcd_display2("pig_top(labExtracted).pcd", "pig_top(��С����)_out.pcd");

	signDir("pig_top(��С����xyz).pcd", "pig_top(labExtracted).pcd", "pig_side(��С����xyz).pcd", 3, "pig_top(headSigned).pcd", "pig_side(headSigned).pcd");//x������ͷ��


	///////////////////�����ϰ벿����ȡ PCA//////////////////////////////////////////////
	//////////////////pigside ��ȡ/////////////////////////
	///////��ȡ�����ϰ벿��//////
	float min_s_y=10,max_s_y=-10,min_s_x=10,max_s_x=-10;
	peakExtra("pig_side(��С����xyz).pcd", 3, min_s_x, max_s_x);
	Sfilter2("pig_side(headSigned).pcd", "x", min_s_x + (max_s_x - min_s_x) / 6, max_s_x - (max_s_x - min_s_x) / 6, "pig_side(body).pcd");//Ϊ׼ȷ�������岿������ֵ ����ͷ���䡢β�͸���

	float median = peakExtra("pig_side(body).pcd", 1, min_s_y, max_s_y);//��ȡ����+��֫�������¼�ֵ�������и�����ӽǶ�Ӧ���� �ų���������ֵ��Ӱ��
	Sfilter2("pig_side(headSigned).pcd", "y", median + (max_s_y - min_s_y)/4, 3, "pig_side(headSigned)_half.pcd");//��ȡ�ϰ벿��//0.15������Ҫ�Ż��ɱ���
	
	//pcd_display3("pig_side(headSigned).pcd", "pig_side(headSigned)_half.pcd");

	
	Eigen::Matrix4f transMatrix_sPCA;
	transMatrix_sPCA = TransCenter("pig_side(headSigned)_half.pcd", "pig_side(����transHalf).pcd");//half ������׼
	MatSave(transMatrix_sPCA, "Mat_SidePCA.txt");

	MatTrans(transMatrix_sPCA, "pig_side(��С����xyz).pcd", "pig_side(��С����Trans).pcd");

	if (!JudgeDir("pig_side(����transHalf).pcd", 2))//�ж�ͷ���Ƿ�z������ ����ת
	{
	reverseZ("pig_side(����transHalf).pcd");
	reverseZ("pig_side(��С����Trans).pcd");
	}

	//pcd_display2("pig_side(����transHalf).pcd", "pig_side(��С����Trans).pcd");





	/////////////////////////pigtop PCA ��ȡ//////////////////////////
	////������Գƻ�////
	float min_z_lab = 10, max_z_lab = -10, height_top;//z����
	peakExtra("pig_top(labExtracted).pcd", 2, min_z_lab, max_z_lab);
	height_top = max_z_lab - min_z_lab;

	Sfilter2("pig_top(labExtracted).pcd", "z", min_z_lab + height_top / 2.0, 3, "pig_top(labAbove).pcd");
	Sfilter2("pig_top(labExtracted).pcd", "z", -3, min_z_lab + height_top / 2.0, "pig_top(labBelow).pcd");

	//��ȥ���岻�Գƶ��ಿ�� ��֤PCA�Գ�
	Sfilter2("pig_top(labExtracted).pcd", "z", min_z_lab + height_top / 2.0, 3, "pig_top(labAbove).pcd");//��Ҫ���ݱ������� -3,max- / min+,3
	Sfilter2("pig_top(labExtracted).pcd", "z", -3, max_z_lab - height_top / 2.0, "pig_top(labBelow).pcd");
	float min_y_lab_A = 10, max_y_lab_A = -10, min_y_lab_B = 10, max_y_lab_B = -10;
	peakExtra("pig_top(labAbove).pcd", 1, min_y_lab_A, max_y_lab_A);
	peakExtra("pig_top(labBelow).pcd", 1, min_y_lab_B, max_y_lab_B);

	if ((max_y_lab_A - min_y_lab_A) > (max_y_lab_B - min_y_lab_B))//�ж�above �� below�ĸ��Ǳ������ĸ��Ǳ�Ե ��ȷ����������
	{
	MatTrans(Mat_Iden, "pig_top(labBelow).pcd", "pig_top(labExtractedBalan).pcd");//����ճ�� ������
	Sfilter2("pig_top(headSigned).pcd", "z", -3, max_z_lab - height_top / 8.0, "pig_top(headSignedBoules).pcd");//ȥ���߽� �߽�ʧ׼��׼��������
	}
	else
	{
	MatTrans(Mat_Iden, "pig_top(labAbove).pcd", "pig_top(labExtractedBalan).pcd");
	Sfilter2("pig_top(headSigned).pcd", "z", min_z_lab + height_top / 8.0, 3, "pig_top(headSignedBoules).pcd");//ȥ���߽� �߽�ʧ׼��׼��������
	}
	pcd_display2("pig_top(headSigned).pcd", "pig_top(headSignedBoules).pcd");

	//pcd_display2("pig_top(labExtracted).pcd", "pig_top(labExtractedBalan).pcd");
	//pcd_display2("pig_top(headSigned).pcd", "pig_top(headSignedBoules).pcd");


	/////������PCA ����λ��////
	Eigen::Matrix4f transMatrix1;
	transMatrix1 = TransCenter("pig_top(labExtractedBalan).pcd", "pig_top(labelPCA).pcd");//���ڷָ��һ��
	MatSave(transMatrix1, "Mat_TopBodyPCA.txt");
	MatTrans(transMatrix1, "pig_top(headSignedBoules).pcd", "pig_top(��С����transSkew).pcd");
	//MatTrans(transMatrix1, "pig_top(headSigned).pcd", "pig_top(headSignedTrans).pcd");
	MatTrans(transMatrix1, "pig_top(����).pcd", "pig_top(����trans).pcd");//����Ҳͬ��ת�� ���Խ������淨����
	MatTrans(transMatrix1, "pig_top(headSigned).pcd", "pig_top(����transSkew).pcd");//δ������////



	createNormal("pig_top(����trans).pcd", "ground_normal.pcd");


	Eigen::Matrix4f transUpright1, transUpright2;
	transUpright1 = Upright1("ground_normal.pcd", "ground_normal(upright1).pcd");//xoyƽ������ת �������淨�߷��� xozƽ��
	transUpright2 = Upright2("ground_normal(upright1).pcd", "ground_normal(upright).pcd");//xozƽ������ת �������淨�߷��� x��������

	MatSave(transUpright1, "Mat_FloorUpright1.txt");
	MatSave(transUpright2, "Mat_FloorUpright2.txt");

	MatTrans(transUpright1, "pig_top(��С����transSkew).pcd", "pig_top(��С����transSkew1).pcd");//������ֱ�ڵ���
	MatTrans(transUpright2, "pig_top(��С����transSkew1).pcd", "pig_top(��С����trans).pcd");//������ֱ�ڵ���

	MatTrans(transUpright1, "pig_top(����transSkew).pcd", "pig_top(����transSkew1).pcd");//δ������////
	MatTrans(transUpright2, "pig_top(����transSkew1).pcd", "pig_top(����trans).pcd");//δ������////





	//pcd_display2("pig_top(��С����transSkew).pcd", "pig_top(��С����trans).pcd");
	if (!JudgeDir("pig_top(��С����trans).pcd", 2))//�ж�ͷ���Ƿ�z������ ����ת
	{
	reverseZ("pig_top(��С����trans).pcd");
	reverseZ("pig_top(����trans).pcd");
	}




	//pcd_display2("pig_top(����trans).pcd", "pig_top(��С����transSkew).pcd");
	//pcd_display2("pig_top(����trans).pcd", "pig_top(����Upright).pcd");
	//pcd_display2("pig_top(��С����trans).pcd", "pig_top(headSignedTrans).pcd");
	//pcd_display2("pig_top(��С����trans1).pcd", "pig_top(����trans1).pcd");
	//pcd_display2("pig_top(��С����trans).pcd", "pig_top(����trans).pcd");

}