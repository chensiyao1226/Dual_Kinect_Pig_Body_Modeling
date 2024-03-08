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

	//////////////////////////pigside 转xyz////////////////////////////
	//////Normal-》xyz
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_Nor2(new pcl::PointCloud<pcl::PointNormal>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz2(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile("pig_side(最小二乘).pcd", *cloud_Nor2);
	pcl::copyPointCloud(*cloud_Nor2, *cloud_xyz2);
	writer.write<pcl::PointXYZ>("pig_side(最小二乘xyzR).pcd", *cloud_xyz2, false);
	reverse("pig_side(最小二乘xyzR).pcd", "pig_side(最小二乘xyz).pcd");
	//pcd_display2("pig_side(最小二乘xyzR).pcd", "pig_side(最小二乘xyz).pcd");
	/////////////////////////pigtop 超体聚类 头部标记//////////////////////////

	//////Normal-》xyz
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_Nor(new pcl::PointCloud<pcl::PointNormal>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile("pig_top(最小二乘).pcd", *cloud_Nor);
	pcl::copyPointCloud(*cloud_Nor, *cloud_xyz);

	writer.write<pcl::PointXYZ>("pig_top(最小二乘xyzR).pcd", *cloud_xyz, false);
	reverse("pig_top(最小二乘xyzR).pcd", "pig_top(最小二乘xyz).pcd");
	//pcd_display2("pig_top(最小二乘xyzR).pcd", "pig_top(最小二乘xyz).pcd");

	//C:\Users\chen\Documents\Visual Studio 2013\Projects\testPCL2\x64\Debug\testPCL2.exe pig_top(最小二乘).pcd -cut 3,2000,0.4 -o
	char* command[] = { "C:\Users\chen\Documents\Visual Studio 2013\Projects\testPCL2\x64\Debug\testPCL2.exe","pig_top(最小二乘xyz).pcd", "-cut", "8,300,0.05", "-o", "-add"};
	int argco = 6;
	CPCsegmen(argco, command);


	////提取主体////
	unsigned int label;
	label = LabelExtr("pig_top(最小二乘xyz)_out.pcd");
	cout << label << endl;
	filterLabel(label, "pig_top(最小二乘xyz)_out.pcd", "pig_top(labExtracted).pcd");//labExtracted--xyzl
	//pcd_display2("pig_top(labExtracted).pcd", "pig_top(最小二乘)_out.pcd");

	signDir("pig_top(最小二乘xyz).pcd", "pig_top(labExtracted).pcd", "pig_side(最小二乘xyz).pcd", 3, "pig_top(headSigned).pcd", "pig_side(headSigned).pcd");//x方向标记头部


	///////////////////猪体上半部分提取 PCA//////////////////////////////////////////////
	//////////////////pigside 提取/////////////////////////
	///////提取整猪上半部分//////
	float min_s_y=10,max_s_y=-10,min_s_x=10,max_s_x=-10;
	peakExtra("pig_side(最小二乘xyz).pcd", 3, min_s_x, max_s_x);
	Sfilter2("pig_side(headSigned).pcd", "x", min_s_x + (max_s_x - min_s_x) / 6, max_s_x - (max_s_x - min_s_x) / 6, "pig_side(body).pcd");//为准确测量主体部分上下值 消除头耳朵、尾巴干扰

	float median = peakExtra("pig_side(body).pcd", 1, min_s_y, max_s_y);//提取主体+四肢部分上下极值，用以切割出两视角对应部分 排除耳朵对最高值的影响
	Sfilter2("pig_side(headSigned).pcd", "y", median + (max_s_y - min_s_y)/4, 3, "pig_side(headSigned)_half.pcd");//截取上半部分//0.15后期需要优化成比例
	
	//pcd_display3("pig_side(headSigned).pcd", "pig_side(headSigned)_half.pcd");

	
	Eigen::Matrix4f transMatrix_sPCA;
	transMatrix_sPCA = TransCenter("pig_side(headSigned)_half.pcd", "pig_side(二乘transHalf).pcd");//half 用来配准
	MatSave(transMatrix_sPCA, "Mat_SidePCA.txt");

	MatTrans(transMatrix_sPCA, "pig_side(最小二乘xyz).pcd", "pig_side(最小二乘Trans).pcd");

	if (!JudgeDir("pig_side(二乘transHalf).pcd", 2))//判断头部是否z正方向 否则翻转
	{
	reverseZ("pig_side(二乘transHalf).pcd");
	reverseZ("pig_side(最小二乘Trans).pcd");
	}

	//pcd_display2("pig_side(二乘transHalf).pcd", "pig_side(最小二乘Trans).pcd");





	/////////////////////////pigtop PCA 提取//////////////////////////
	////猪主体对称化////
	float min_z_lab = 10, max_z_lab = -10, height_top;//z参数
	peakExtra("pig_top(labExtracted).pcd", 2, min_z_lab, max_z_lab);
	height_top = max_z_lab - min_z_lab;

	Sfilter2("pig_top(labExtracted).pcd", "z", min_z_lab + height_top / 2.0, 3, "pig_top(labAbove).pcd");
	Sfilter2("pig_top(labExtracted).pcd", "z", -3, min_z_lab + height_top / 2.0, "pig_top(labBelow).pcd");

	//削去主体不对称多余部分 保证PCA对称
	Sfilter2("pig_top(labExtracted).pcd", "z", min_z_lab + height_top / 2.0, 3, "pig_top(labAbove).pcd");//需要根据背部方向 -3,max- / min+,3
	Sfilter2("pig_top(labExtracted).pcd", "z", -3, max_z_lab - height_top / 2.0, "pig_top(labBelow).pcd");
	float min_y_lab_A = 10, max_y_lab_A = -10, min_y_lab_B = 10, max_y_lab_B = -10;
	peakExtra("pig_top(labAbove).pcd", 1, min_y_lab_A, max_y_lab_A);
	peakExtra("pig_top(labBelow).pcd", 1, min_y_lab_B, max_y_lab_B);

	if ((max_y_lab_A - min_y_lab_A) > (max_y_lab_B - min_y_lab_B))//判断above 和 below哪个是背部，哪个是边缘 （确定背部朝向）
	{
	MatTrans(Mat_Iden, "pig_top(labBelow).pcd", "pig_top(labExtractedBalan).pcd");//复制粘贴 换名字
	Sfilter2("pig_top(headSigned).pcd", "z", -3, max_z_lab - height_top / 8.0, "pig_top(headSignedBoules).pcd");//去掉边界 边界失准配准会产生误差
	}
	else
	{
	MatTrans(Mat_Iden, "pig_top(labAbove).pcd", "pig_top(labExtractedBalan).pcd");
	Sfilter2("pig_top(headSigned).pcd", "z", min_z_lab + height_top / 8.0, 3, "pig_top(headSignedBoules).pcd");//去掉边界 边界失准配准会产生误差
	}
	pcd_display2("pig_top(headSigned).pcd", "pig_top(headSignedBoules).pcd");

	//pcd_display2("pig_top(labExtracted).pcd", "pig_top(labExtractedBalan).pcd");
	//pcd_display2("pig_top(headSigned).pcd", "pig_top(headSignedBoules).pcd");


	/////猪主体PCA 归正位置////
	Eigen::Matrix4f transMatrix1;
	transMatrix1 = TransCenter("pig_top(labExtractedBalan).pcd", "pig_top(labelPCA).pcd");//用于分割顶部一半
	MatSave(transMatrix1, "Mat_TopBodyPCA.txt");
	MatTrans(transMatrix1, "pig_top(headSignedBoules).pcd", "pig_top(最小二乘transSkew).pcd");
	//MatTrans(transMatrix1, "pig_top(headSigned).pcd", "pig_top(headSignedTrans).pcd");
	MatTrans(transMatrix1, "pig_top(地面).pcd", "pig_top(地面trans).pcd");//地面也同样转换 用以矫正地面法向量
	MatTrans(transMatrix1, "pig_top(headSigned).pcd", "pig_top(完整transSkew).pcd");//未经剪裁////



	createNormal("pig_top(地面trans).pcd", "ground_normal.pcd");


	Eigen::Matrix4f transUpright1, transUpright2;
	transUpright1 = Upright1("ground_normal.pcd", "ground_normal(upright1).pcd");//xoy平面内旋转 矫正地面法线方向到 xoz平面
	transUpright2 = Upright2("ground_normal(upright1).pcd", "ground_normal(upright).pcd");//xoz平面内旋转 矫正地面法线方向到 x轴正方向

	MatSave(transUpright1, "Mat_FloorUpright1.txt");
	MatSave(transUpright2, "Mat_FloorUpright2.txt");

	MatTrans(transUpright1, "pig_top(最小二乘transSkew).pcd", "pig_top(最小二乘transSkew1).pcd");//矫正垂直于地面
	MatTrans(transUpright2, "pig_top(最小二乘transSkew1).pcd", "pig_top(最小二乘trans).pcd");//矫正垂直于地面

	MatTrans(transUpright1, "pig_top(完整transSkew).pcd", "pig_top(完整transSkew1).pcd");//未经剪裁////
	MatTrans(transUpright2, "pig_top(完整transSkew1).pcd", "pig_top(完整trans).pcd");//未经剪裁////





	//pcd_display2("pig_top(最小二乘transSkew).pcd", "pig_top(最小二乘trans).pcd");
	if (!JudgeDir("pig_top(最小二乘trans).pcd", 2))//判断头部是否z正方向 否则翻转
	{
	reverseZ("pig_top(最小二乘trans).pcd");
	reverseZ("pig_top(完整trans).pcd");
	}




	//pcd_display2("pig_top(地面trans).pcd", "pig_top(最小二乘transSkew).pcd");
	//pcd_display2("pig_top(地面trans).pcd", "pig_top(地面Upright).pcd");
	//pcd_display2("pig_top(最小二乘trans).pcd", "pig_top(headSignedTrans).pcd");
	//pcd_display2("pig_top(最小二乘trans1).pcd", "pig_top(完整trans1).pcd");
	//pcd_display2("pig_top(最小二乘trans).pcd", "pig_top(完整trans).pcd");

}