#include <iostream>
#include "pcd_Sfilter.h"
#include "pcd_sampling.h"
#include "pcd_NorFilter.h"
#include "pcd_display.h"
#include "pcd_display2.h"
#include "pcd_BiFilter.h"
#include "pcd_BilFilter.h"
#include "pcd_denoising.h"
#include "pcd_GausFilter.h"
#include "pcd_clustering.h"
#include "pcd_LSmethod.h"
#include "reverseY.h"
#include "reverseX.h"
#include "Sfilter2.h"
#include "merge.h"

void pretreatment()
{
	pcd_Sfilter("selected_side.pcd","pig_side(直通).pcd");//1、直通滤波
	pcd_sampling("pig_side(直通).pcd", "pig_side(下采样).pcd");//2、采样滤波
	reverseY("pig_side(下采样).pcd", "pig_side(下采样reversed).pcd");
	pcd_NorFilter("pig_side(下采样reversed).pcd", "pig_side(地面滤波).pcd", "pig_side(地面).pcd");//3、地面滤波
	//pcd_display2("selected_side.pcd", "pig_side(地面滤波).pcd");


	pcd_clustering("pig_side(地面滤波).pcd", "pig_side(聚类).pcd");//聚类提取猪体
	//pcd_display2("pig_side(聚类).pcd", "pig_side(地面滤波).pcd");


	//滤掉木板车//
	Sfilter2("pig_side(聚类).pcd", "y", -0.3, 3, "pig_side(legAbove).pcd");//裁去猪的主体
	Sfilter2("pig_side(聚类).pcd", "y", -3, -0.3, "pig_side(legBelow).pcd");//裁去猪的主体
	pcd_NorFilter("pig_side(legBelow).pcd", "pig_side(地面滤波2).pcd", "pig_side(地面2).pcd");//滤去木板车
	//pcd_display3("pig_side(地面滤波2).pcd", "pig_side(legBelow).pcd");

	merge("pig_side(地面滤波2).pcd", "pig_side(legAbove).pcd", "Mat_Iden.txt", "pig_side(地面trans).pcd", "pig_side(木板滤波).pcd");
	pcd_clustering("pig_side(木板滤波).pcd", "pig_side(聚类2).pcd");//聚类提取猪体
	//pcd_display2("pig_side(聚类).pcd", "pig_side(木板滤波).pcd");

	//pcd_denoising("pig_side(聚类2).pcd", "pig_side(去噪).pcd", 4);
	//pcd_LSmethod("pig_side(去噪).pcd", "pig_side(最小二乘).pcd");
	Sfilter2("pig_side(聚类2).pcd", "y", -3, 3, "pig_side(最小二乘).pcd");
	//pcd_display2("pig_side(聚类2).pcd", "pig_side(最小二乘).pcd");

	/////pig_top预处理/////
	pcd_Sfilter("selected_top.pcd", "pig_top(直通).pcd");//1、直通滤波
	pcd_sampling("pig_top(直通).pcd", "pig_top(下采样).pcd");//2、采样滤波
	reverseY("pig_top(下采样).pcd", "pig_top(下采样reversed).pcd");

	pcd_NorFilter("pig_top(下采样reversed).pcd", "pig_top(地面滤波).pcd", "pig_top(地面).pcd");//3、地面滤波


	//pcd_display2("selected_top.pcd", "pig_top(地面滤波).pcd");

	Sfilter2("pig_top(地面滤波).pcd", "x", -1, 0.8, "pig_top(地面滤波Fil).pcd");

	pcd_clustering("pig_top(地面滤波Fil).pcd", "pig_top(聚类).pcd");
	//pcd_display2("pig_top(地面滤波Fil).pcd", "pig_top(聚类).pcd");

	//pcd_denoising("pig_top(聚类).pcd", "pig_top(去噪).pcd", 4);
	//pcd_LSmethod("pig_top(去噪).pcd", "pig_top(最小二乘).pcd");
	Sfilter2("pig_top(聚类).pcd", "y", -3, 3, "pig_top(最小二乘).pcd");

	//pcd_display2("pig_side(最小二乘).pcd", "pig_top(最小二乘).pcd");
	
}