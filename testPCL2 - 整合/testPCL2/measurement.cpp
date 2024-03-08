#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/console/parse.h>
#include <pcl/console/print.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>


#include "symmetrize.h"
#include "PLYtoPCD.h"
#include "surface.h"
#include "BoundaryExtr.h"
#include "searchExtremum.h"
#include "pcd_display4.h"
#include "graphCreate.h"
#include "searchIndex.h"
#include "curvature.h"
#include "FeaturePExtr.h"
#include "searchPoint.h"
#include "searchEnvelope.h"
#include "pcd_display5.h"
#include "getMidpoint.h"
#include "getX.h"
#include "calDistance.h"
#include "reverseY.h"
#include "reverseX.h"
#include "peakExtra.h"
#include "Sfilter2.h"
#include "conditionRemove.h"
#include "pcd_display2.h"
#include "pcd_display3.h"
#include "pcd_display4.h"
#include "merge.h"

float size_resolu = 0.004;
void measurement()
{

	pcl::PCDWriter writer;
	float min_x_topReg, max_x_topReg;
	peakExtra("pig_top(RegUpBody).pcd", 3, min_x_topReg, max_x_topReg);//主体部分上下极值
	float min_x_sideReg, max_x_sideReg, min_z_sideReg, max_z_sideReg;
	peakExtra("pig_side(Reg).pcd", 3, min_x_sideReg, max_x_sideReg);
	peakExtra("pig_side(Reg).pcd", 2, min_z_sideReg, max_z_sideReg);
	//////////////////////////提尺测量预处理///////////////////////
	PLYtoPCD("pig(重建).ply", "pig(重建).pcd");
	//考虑加下采样

	//pcd_display3("pig(重建).pcd", "pig_side(Reg).pcd");

	BoundaryExtr("pig_side(Reg).pcd", "pig_side(Boundary).pcd", "pig_side(BoundaryPlane).pcd");
	//Harris("pig_side(Boundary).pcd", "pig_side(Harris).pcd");



	float min_bs_x = 10, max_bs_x = -10;
	peakExtra("pig_side(Boundary).pcd", 3, min_bs_x, max_bs_x);
	Sfilter2("pig_side(Boundary).pcd", "x", -3, (min_bs_x + max_bs_x) / 2, "pig_side(below).pcd");//(min_bs_x+max_bs_x)/2, 3



	//pcd_display3("pig_integral.pcd", "pig(重建).pcd");

	
	////腹围////
	pcl::PointXYZ belly_Extremum(0, 0, -0.1);//胸围下测点 前肢之前x方向极小值点
	float Belly_endLength = 2, Belly_extraExplore = 0.2;

	searchExtremum("pig_side(below).pcd", belly_Extremum, Belly_endLength, Belly_extraExplore, 1, 1, -1);//111

	pcl::PointCloud<pcl::PointXYZ> cloud_belly_Extremum;
	cloud_belly_Extremum.push_back(belly_Extremum);
	writer.write<pcl::PointXYZ>("belly_Extremum.pcd", cloud_belly_Extremum, false);
	pcd_display4("belly_Extremum.pcd", "pig(重建).pcd");


	//剪裁出腹围一圈点云
	Sfilter2("pig(重建).pcd", "z", belly_Extremum.z - 0.01, belly_Extremum.z + 0.01, "pig(belly).pcd");

	int index_bellyB, index_bellyL, index_bellyR;//下、左上、右上侧胸围测点 建立最短路径回路
	pcl::PointXYZ Pb_belly, Pl_belly, Pr_belly;
	index_bellyB = searchIndex("pig(belly).pcd", belly_Extremum.x - size_resolu * 8, belly_Extremum.x + size_resolu * 8,
	belly_Extremum.y - size_resolu * 5, belly_Extremum.y + size_resolu * 5, belly_Extremum.z - size_resolu, belly_Extremum.z + size_resolu, Pb_belly);//由于曲面拟合会缩减 需要大范围搜索
	cout << "1" << endl;
	index_bellyL = searchIndex("pig(belly).pcd", max_x_topReg - 0.1 - size_resolu * 3, max_x_topReg - 0.1 + size_resolu * 3,
	-3, 0, belly_Extremum.z - size_resolu, belly_Extremum.z + size_resolu, Pl_belly);//z值相同 x、y值指定区域左上 尽量靠上 //min_x_topReg + 0.1 0,3
	cout << "2" << endl;
	index_bellyR = searchIndex("pig(belly).pcd", max_x_topReg - 0.1 - size_resolu * 3, max_x_topReg - 0.1 + size_resolu * 3,
	0, 3, belly_Extremum.z - size_resolu, belly_Extremum.z + size_resolu, Pr_belly);//z值相同 x、y值指定区域右上 尽量靠上


	float shortPathBellyL = graphCreate(index_bellyB, index_bellyL, "pig(belly).pcd");//左侧弧线
	float shortPathBellyR = graphCreate(index_bellyB, index_bellyR, "pig(belly).pcd");//右侧弧线
	float shortPathBellyH = graphCreate(index_bellyL, index_bellyR, "pig(belly).pcd");//上侧弧线
	cout << "abdominal circumference:" << shortPathBellyL + shortPathBellyR + shortPathBellyH << endl;

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_BLR_belly(new pcl::PointCloud<pcl::PointXYZ>);
	cloud_BLR_belly->push_back(Pb_belly);
	cloud_BLR_belly->push_back(Pl_belly);
	cloud_BLR_belly->push_back(Pr_belly);
	writer.write<pcl::PointXYZ>("Points_belly.pcd", *cloud_BLR_belly, false);

	pcd_display4("Points_belly.pcd", "pig_side(Boundary).pcd");

	pcd_display4("pig(belly).pcd", "pig(重建).pcd");//示意图
	
	
	////胸围////
	Sfilter2("pig_side(below).pcd", "z", 0.05, 3, "pig_side(rightLeg).pcd");
	pcl::PointXYZ bust_Extremum(-3, 0, 0);//胸围下测点 前肢之前x方向极小值点 3
	float Bust_endLength = 2, Bust_extraExplore = 0.1;

	searchExtremum("pig_side(rightLeg).pcd", bust_Extremum, Bust_endLength, Bust_extraExplore, 1, 1,1);//11-1


	pcl::PointCloud<pcl::PointXYZ> cloud_bust_Extremum;
	cloud_bust_Extremum.push_back(bust_Extremum);
	writer.write<pcl::PointXYZ>("bust_Extremum.pcd", cloud_bust_Extremum, false);
	//pcd_display4("bust_Extremum.pcd", "pig_side(rightLeg).pcd");

	//剪裁出胸围一圈点云
	Sfilter2("pig_complete(去噪).pcd", "z", bust_Extremum.z - 0.01, bust_Extremum.z + 0.01, "pig(bust).pcd");
	pcd_display4("bust_Extremum.pcd", "pig(bust).pcd");
	pcl::PointXYZ Pb_chest, Pl_chest, Pr_chest;
	int index_chestB, index_chestL, index_chestR;//下、左上、右上侧胸围测点 建立最短路径回路
	index_chestB = searchIndex("pig(bust).pcd", bust_Extremum.x - size_resolu * 8, bust_Extremum.x + size_resolu * 8,
	bust_Extremum.y - size_resolu * 8, bust_Extremum.y + size_resolu * 8, bust_Extremum.z - size_resolu, bust_Extremum.z + size_resolu, Pb_chest);
	cout << "1" << endl;
	index_chestL = searchIndex("pig(bust).pcd", max_x_topReg - 0.1 - size_resolu * 5, max_x_topReg - 0.1 + size_resolu * 5,
	-3, 0, bust_Extremum.z - size_resolu, bust_Extremum.z + size_resolu, Pl_chest);//z值相同 x、y值指定区域左上 尽量靠上
	cout << "2" << endl;
	index_chestR = searchIndex("pig(bust).pcd", max_x_topReg - 0.1 - size_resolu * 5, max_x_topReg - 0.1 + size_resolu * 5,
	0, 3, bust_Extremum.z - size_resolu, bust_Extremum.z + size_resolu, Pr_chest);//z值相同 x、y值指定区域右上 尽量靠上


	float shortPathL = graphCreate(index_chestB, index_chestL, "pig(bust).pcd");//左侧弧线
	float shortPathR = graphCreate(index_chestB, index_chestR, "pig(bust).pcd");//右侧弧线
	float shortPathH = graphCreate(index_chestL, index_chestR, "pig(bust).pcd");//上侧弧线
	cout << "bust measurement:" << shortPathL + shortPathR + shortPathH << endl;

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_BLR_chest(new pcl::PointCloud<pcl::PointXYZ>);
	cloud_BLR_chest->push_back(Pb_chest);
	cloud_BLR_chest->push_back(Pl_chest);
	cloud_BLR_chest->push_back(Pr_chest);
	writer.write<pcl::PointXYZ>("Points_chest.pcd", *cloud_BLR_chest, false);

	pcd_display4("Points_chest.pcd", "pig_side(Boundary).pcd");
	pcd_display4("pig(bust).pcd", "pig_complete(去噪).pcd");
	
	
	////臀宽-1曲率////
	////boundary top////
	BoundaryExtr("pig_top(完整trans).pcd", "pig_top(Boundary).pcd", "pig_top(BoundaryPlane).pcd");
	Sfilter2("pig_top(BoundaryPlane).pcd", "y", 0, 3, "pig_top(BPright).pcd");
	Sfilter2("pig_top(BoundaryPlane).pcd", "y", -3, 0, "pig_top(BPleft).pcd");

	curvature("pig_top(Boundary).pcd", "pig_top(BoundaryCurv).pcd");


	FeaturePExtr("pig_top(BoundaryCurv).pcd", "pig_top(BoundaryCurvExtr).pcd");//主体部分曲率平坦 curvature小于0.01 末端为臀宽测点
	Sfilter2("pig_top(BoundaryCurvExtr).pcd", "y", 0, 3, "pig_top(curveRightExtr).pcd");
	Sfilter2("pig_top(BoundaryCurvExtr).pcd", "y", -3, 0, "pig_top(curveLeftExtr).pcd");

	//pcd_display3("pig_top(curveLeftExtr).pcd", "pig_top(curveRightExtr).pcd");
	
	float min_top_z = 10, max_top_z = -10;
	peakExtra("pig_top(BoundaryPlane).pcd", 2, min_top_z, max_top_z);

	//左//
	float min_curLeft_z = 10, max_curLeft_z = -10;
	peakExtra("pig_top(curveLeftExtr).pcd", 2, min_curLeft_z, max_curLeft_z);

	//右//
	float min_curRight_z = 10, max_curRight_z = -10;
	peakExtra("pig_top(curveRightExtr).pcd", 2, min_curRight_z, max_curRight_z);


	///臀宽-2极大值/////
	//左//问题
	pcl::PointXYZ butt_Left_Extremum(0, 0, min_top_z);//耳尖最大值 避免体宽影响设置10cm内极大值
	float butt_Left_endLength = 0.2, butt_left_extraExplore = 0.15;
	searchExtremum("pig_top(BPleft).pcd", butt_Left_Extremum, butt_Left_endLength, butt_left_extraExplore, 1, 2, -1);

	float butt_Left = butt_Left_Extremum.z;//;
	cout << "butt_Left" << butt_Left << endl;
	searchPoint(butt_Left, "pig_top(BPleft).pcd", "butt_Left.pcd");
	//pcd_display4("butt_Left.pcd", "pig_top(BPleft).pcd");

	//右//
	pcl::PointXYZ butt_Right_Extremum(0, 0, min_top_z);//耳尖最大值 避免体宽影响设置10cm内极大值
	float butt_Right_endLength = 0.2, butt_Right_extraExplore = 0.15;
	searchExtremum("pig_top(BPright).pcd", butt_Right_Extremum, butt_Right_endLength, butt_Right_extraExplore, 1, 2, 1);

	float butt_Right = butt_Right_Extremum.z;
	cout << "butt_Right" << butt_Right << endl;
	searchPoint(butt_Right, "pig_top(BPright).pcd", "butt_Right.pcd");
	//pcd_display4("butt_Right.pcd", "pig_top(BPright).pcd");
	///

	//臀宽//
	merge("butt_Left.pcd", "butt_Right.pcd", "Mat_Iden.txt", "instant.pcd", "butt_points.pcd");
	float min_buttCalcu = 10, max_buttCalcu = -10;
	peakExtra("butt_points.pcd", 1, min_buttCalcu, max_buttCalcu);
	float hipWidth = max_buttCalcu - min_buttCalcu;
	cout << "hip width=" << hipWidth << endl;

	pcd_display4("butt_points.pcd", "pig_top(BoundaryPlane).pcd");
	
	
	//臀高 boundary side//
	Sfilter2("pig_side(Boundary).pcd", "x", -0.2, 3, "pig_side(BoundaryTop).pcd");
	searchPoint((butt_Left + butt_Right) / 2, "pig_side(BoundaryTop).pcd", "hip_height.pcd");
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hip_height(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile("hip_height.pcd", *cloud_hip_height);
	pcl::PointXYZ hip_heightP = cloud_hip_height->points[0];


	float min_sideCut_x = 10, max_sideCut_x = -10;
	peakExtra("pig_side(Boundary).pcd", 3, min_sideCut_x, max_sideCut_x);//获得地面值

	float hip_height = abs(hip_heightP.x - min_sideCut_x);
	cout << "Hip height:" << hip_height << endl;
	pcd_display4("hip_height.pcd", "pig_side(Boundary).pcd");
	
	
	///耳尖///
	//左//
	pcl::PointXYZ ear_Left_Extremum(0, 0, max_top_z);//耳尖最大值 避免体宽影响设置10cm内极大值
	float Ear_Left_endLength = 0.5, Ear_left_extraExplore = 0.2;
	searchExtremum("pig_top(BoundaryPlane).pcd", ear_Left_Extremum, Ear_Left_endLength, Ear_left_extraExplore, -1, 2, -1);

	pcl::PointCloud<pcl::PointXYZ> cloud_ear_Left_Extremum;
	cloud_ear_Left_Extremum.push_back(ear_Left_Extremum);
	writer.write<pcl::PointXYZ>("ear_Left_Extremum.pcd", cloud_ear_Left_Extremum, false);
	//pcd_display4("ear_Left_Extremum.pcd", "pig_top(BoundaryPlane).pcd");
	//右//
	pcl::PointXYZ ear_Right_Extremum(0, 0, max_top_z);//耳尖最大值 避免体宽影响设置10cm内极大值
	float Ear_Right_endLength = 0.5, Ear_Right_extraExplore = 0.2;
	searchExtremum("pig_top(BoundaryPlane).pcd", ear_Right_Extremum, Ear_Right_endLength, Ear_Right_extraExplore, -1, 2, 1);

	pcl::PointCloud<pcl::PointXYZ> cloud_ear_Right_Extremum;
	cloud_ear_Right_Extremum.push_back(ear_Right_Extremum);
	writer.write<pcl::PointXYZ>("ear_Right_Extremum.pcd", cloud_ear_Right_Extremum, false);
	//pcd_display4("ear_Right_Extremum.pcd", "pig_top(BoundaryPlane).pcd");



	//计算距离包络最远点为 耳根测点
	//左//
	Sfilter2("pig_top(BPleft).pcd", "z", butt_Left - 0.02, ear_Left_Extremum.z - 0.04, "pig_top(BPleftCut).pcd");//切掉臀部 切掉耳尖前部 并避免前侧耳根影响
	float k_left_ear = searchEnvelope(ear_Left_Extremum, "pig_top(BPleftCut).pcd", -1, "pig_top(earRootLeft).pcd");
	//pcd_display5(k_left_ear, "ear_Left_Extremum.pcd", -0.5, "pig_top(earRootLeft).pcd", "pig_top(BPleftCut).pcd");
	//右//
	Sfilter2("pig_top(BPright).pcd", "z", 0, ear_Right_Extremum.z-0.06, "pig_top(BPrightCut).pcd");//切掉臀部 切掉耳尖前部
	float k_right_ear = searchEnvelope(ear_Right_Extremum, "pig_top(BPrightCut).pcd", 1, "pig_top(earRootRight).pcd");

	//耳根中点//
	getMidpoint("pig_top(earRootLeft).pcd", "pig_top(earRootRight).pcd", "pig_top(earRoot).pcd");

	merge("pig_top(earRootLeft).pcd", "pig_top(earRootRight).pcd", "Mat_Iden.txt", "instant.pcd", "pig_top(earRootLR).pcd");
	merge("pig_top(earRootLR).pcd", "pig_top(earRoot).pcd", "Mat_Iden.txt", "instant.pcd", "pig_top(earRooAll).pcd");

	//pcd_display5(k_left_ear, "ear_Left_Extremum.pcd", k_right_ear, "ear_Right_Extremum.pcd", -0.5, "pig_top(earRooAll).pcd", "pig_top(BoundaryPlane).pcd");




	//计算距离包络最远点为 尾根测点
	//左//
	searchPoint(min_top_z, "pig_top(BoundaryPlane).pcd", "tail_end.pcd");

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tail_tip(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile("tail_end.pcd", *cloud_tail_tip);
	pcl::PointXYZ tail_end = cloud_tail_tip->points[0];

	Sfilter2("pig_top(BPleft).pcd", "z", min_top_z+0.01, butt_Left-0.08, "pig_top(BPleftTail).pcd");//削去尾尖部分 视品种尾巴长度而定 避免影响包络检测

	float k_left_tail = searchEnvelope(tail_end, "pig_top(BPleftTail).pcd", -1, "pig_top(tailRootLeft).pcd");

	//右//
	Sfilter2("pig_top(BPright).pcd", "z", min_top_z+0.01, butt_Right - 0.08, "pig_top(BPrightTail).pcd");//削去尾尖部分 视品种尾巴长度而定 避免影响包络检测

	float k_right_tail = searchEnvelope(tail_end, "pig_top(BPrightTail).pcd", 1, "pig_top(tailRootRight).pcd");
	pcd_display4("tail_end.pcd", "pig_top(BPleftTail).pcd");

	//尾根中点//
	getMidpoint("pig_top(tailRootLeft).pcd", "pig_top(tailRootRight).pcd", "pig_top(tailRoot).pcd");

	merge("pig_top(tailRootLeft).pcd", "pig_top(tailRootRight).pcd", "Mat_Iden.txt", "instant.pcd", "pig_top(tailRootLR).pcd");
	merge("pig_top(tailRootLR).pcd", "pig_top(tailRoot).pcd", "Mat_Iden.txt", "instant.pcd", "pig_top(tailRootAll).pcd");

	pcd_display5(k_left_tail, "tail_end.pcd", k_right_tail, "tail_end.pcd", 0, "pig_top(tailRootAll).pcd", "pig_top(BoundaryPlane).pcd");
	


	
	//体长计算（耳根中点-尾根中点）//
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tail(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile("pig_top(tailRoot).pcd", *cloud_tail);
	pcl::PointXYZ tail_mid = cloud_tail->points[0];

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ear(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile("pig_top(earRoot).pcd", *cloud_ear);
	pcl::PointXYZ ear_mid = cloud_ear->points[0];

	float tail_x = getX("pig_complete(去噪).pcd", tail_mid.y - size_resolu * 4, tail_mid.y + size_resolu * 4, tail_mid.z - size_resolu * 2, tail_mid.z + size_resolu * 2);//找到测点附近x值
	float ear_x = getX("pig_complete(去噪).pcd", ear_mid.y - size_resolu * 4, ear_mid.y + size_resolu * 4, ear_mid.z - size_resolu * 2, ear_mid.z + size_resolu * 2);

	pcl::PointXYZ tail_p(tail_x, tail_mid.y, tail_mid.z), ear_p(ear_x, ear_mid.y, ear_mid.z);

	Sfilter2("pig_complete(去噪).pcd", "x", tail_x - 0.1, 3, "pig_top(完整transCutX).pcd");//截取上半部分
	Sfilter2("pig_top(完整transCutX).pcd", "z", tail_mid.z - 0.01, ear_mid.z + 0.01, "pig_top(完整transCut).pcd");//截取中间部分

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_top(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile("pig_top(完整transCut).pcd", *cloud_top);
	int index_tail = cloud_top->points.size();
	int index_ear = cloud_top->points.size() + 1;
	cloud_top->points.push_back(tail_p);//插入测点 减少采样值对精度影响
	cloud_top->points.push_back(ear_p);

	cloud_top->width = cloud_top->points.size();
	cloud_top->height = 1;
	writer.write<pcl::PointXYZ>("pig_top(完整transMeas).pcd", *cloud_top, false);

	float bodyLength = graphCreate(index_tail, index_ear, "pig_top(完整transMeas).pcd");//体长最短路径计算
	cout << "body length:" << bodyLength << endl;
	

	
	////体高测点 boundary side////
	/*
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ear(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile("pig_top(earRoot).pcd", *cloud_ear);
	*/
	pcl::PointXYZ ear_end = cloud_ear->points[0];
	/*
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tail(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile("pig_top(tailRoot).pcd", *cloud_tail);
	pcl::PointXYZ tail_end = cloud_tail->points[0];
	*/


	Sfilter2("pig_side(Boundary).pcd", "z", tail_end.z, ear_end.z, "pig_side(BoundaryCut).pcd");
	pcl::PointXYZ bodyHeight(0, 0, ear_end.z);
	searchExtremum("pig_side(BoundaryCut).pcd", bodyHeight, 1, 0.2, -1, 1, 1);//-11-1
	/*
	float min_sideCut_x = 10, max_sideCut_x = -10;
	peakExtra("pig_side(BoundaryCut).pcd", 3, min_sideCut_x, max_sideCut_x);//获得地面值
	*/
	pcl::PointCloud<pcl::PointXYZ> cloud_bodyHeight_Extremum;
	cloud_bodyHeight_Extremum.push_back(bodyHeight);
	writer.write<pcl::PointXYZ>("bodyHeight.pcd", cloud_bodyHeight_Extremum, false);
	cout << "Body height:" << bodyHeight.x - min_sideCut_x << endl;

	pcd_display4("bodyHeight.pcd", "pig_side(Boundary).pcd");
	

	////体宽测点 boundary top////
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_earRootLeft(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile("pig_top(earRootLeft).pcd", *cloud_earRootLeft);
	pcl::PointXYZ ear_left = cloud_earRootLeft->points[0];

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_earRootRight(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile("pig_top(earRootRight).pcd", *cloud_earRootRight);
	pcl::PointXYZ ear_right = cloud_earRootRight->points[0];


	pcl::PointXYZ neck_Left_Extremum(0, 0, ear_left.z);//耳尖最大值 避免体宽影响设置10cm内极大值
	float neck_Left_endLength = 0.2, neck_left_extraExplore = 0.15;
	searchExtremum("pig_top(BPleft).pcd", neck_Left_Extremum, neck_Left_endLength, neck_left_extraExplore, -1, 2, -1);

	float neck_Left = neck_Left_Extremum.z;//;
	cout << "neck_Left" << neck_Left << endl;
	searchPoint(neck_Left, "pig_top(BPleft).pcd", "neck_Left.pcd");

	//右//
	pcl::PointXYZ neck_Right_Extremum(0, 0, ear_right.z);//耳尖最大值 避免体宽影响设置10cm内极大值
	float neck_Right_endLength = 0.2, neck_Right_extraExplore = 0.15;
	searchExtremum("pig_top(BPright).pcd", neck_Right_Extremum, neck_Right_endLength, neck_Right_extraExplore, -1, 2, 1);

	float neck_Right = neck_Right_Extremum.z;
	cout << "neck_Right" << neck_Right << endl;
	searchPoint(neck_Right, "pig_top(BPright).pcd", "neck_Right.pcd");

	merge("neck_Left.pcd", "neck_Right.pcd", "Mat_Iden.txt", "instant.pcd", "neck_points.pcd");
	float min_neckCalcu = 10, max_neckCalcu = -10;
	peakExtra("neck_points.pcd", 1, min_neckCalcu, max_neckCalcu);
	float bodyWidth = max_neckCalcu - min_neckCalcu;
	cout << "body width=" << bodyWidth << endl;

	pcd_display4("neck_points.pcd", "pig_top(BoundaryPlane).pcd");

	////体高测点 boundary side////
	searchPoint((neck_Left + neck_Right) / 2, "pig_side(BoundaryTop).pcd", "body_height.pcd");
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_body_height(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile("body_height.pcd", *cloud_body_height);
	pcl::PointXYZ body_heightP = cloud_body_height->points[0];

	/*
	float min_sideCut_x = 10, max_sideCut_x = -10;
	peakExtra("pig_side(Boundary).pcd", 3, min_sideCut_x, max_sideCut_x);//获得地面值
	*/
	float body_height = abs(body_heightP.x - min_sideCut_x);
	cout << "Body height:" << body_height << endl;
	pcd_display4("body_height.pcd", "pig_side(Boundary).pcd");
}