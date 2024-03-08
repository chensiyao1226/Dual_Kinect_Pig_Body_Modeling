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
	pcd_Sfilter("selected_side.pcd","pig_side(ֱͨ).pcd");//1��ֱͨ�˲�
	pcd_sampling("pig_side(ֱͨ).pcd", "pig_side(�²���).pcd");//2�������˲�
	reverseY("pig_side(�²���).pcd", "pig_side(�²���reversed).pcd");
	pcd_NorFilter("pig_side(�²���reversed).pcd", "pig_side(�����˲�).pcd", "pig_side(����).pcd");//3�������˲�
	//pcd_display2("selected_side.pcd", "pig_side(�����˲�).pcd");


	pcd_clustering("pig_side(�����˲�).pcd", "pig_side(����).pcd");//������ȡ����
	//pcd_display2("pig_side(����).pcd", "pig_side(�����˲�).pcd");


	//�˵�ľ�峵//
	Sfilter2("pig_side(����).pcd", "y", -0.3, 3, "pig_side(legAbove).pcd");//��ȥ�������
	Sfilter2("pig_side(����).pcd", "y", -3, -0.3, "pig_side(legBelow).pcd");//��ȥ�������
	pcd_NorFilter("pig_side(legBelow).pcd", "pig_side(�����˲�2).pcd", "pig_side(����2).pcd");//��ȥľ�峵
	//pcd_display3("pig_side(�����˲�2).pcd", "pig_side(legBelow).pcd");

	merge("pig_side(�����˲�2).pcd", "pig_side(legAbove).pcd", "Mat_Iden.txt", "pig_side(����trans).pcd", "pig_side(ľ���˲�).pcd");
	pcd_clustering("pig_side(ľ���˲�).pcd", "pig_side(����2).pcd");//������ȡ����
	//pcd_display2("pig_side(����).pcd", "pig_side(ľ���˲�).pcd");

	//pcd_denoising("pig_side(����2).pcd", "pig_side(ȥ��).pcd", 4);
	//pcd_LSmethod("pig_side(ȥ��).pcd", "pig_side(��С����).pcd");
	Sfilter2("pig_side(����2).pcd", "y", -3, 3, "pig_side(��С����).pcd");
	//pcd_display2("pig_side(����2).pcd", "pig_side(��С����).pcd");

	/////pig_topԤ����/////
	pcd_Sfilter("selected_top.pcd", "pig_top(ֱͨ).pcd");//1��ֱͨ�˲�
	pcd_sampling("pig_top(ֱͨ).pcd", "pig_top(�²���).pcd");//2�������˲�
	reverseY("pig_top(�²���).pcd", "pig_top(�²���reversed).pcd");

	pcd_NorFilter("pig_top(�²���reversed).pcd", "pig_top(�����˲�).pcd", "pig_top(����).pcd");//3�������˲�


	//pcd_display2("selected_top.pcd", "pig_top(�����˲�).pcd");

	Sfilter2("pig_top(�����˲�).pcd", "x", -1, 0.8, "pig_top(�����˲�Fil).pcd");

	pcd_clustering("pig_top(�����˲�Fil).pcd", "pig_top(����).pcd");
	//pcd_display2("pig_top(�����˲�Fil).pcd", "pig_top(����).pcd");

	//pcd_denoising("pig_top(����).pcd", "pig_top(ȥ��).pcd", 4);
	//pcd_LSmethod("pig_top(ȥ��).pcd", "pig_top(��С����).pcd");
	Sfilter2("pig_top(����).pcd", "y", -3, 3, "pig_top(��С����).pcd");

	//pcd_display2("pig_side(��С����).pcd", "pig_top(��С����).pcd");
	
}