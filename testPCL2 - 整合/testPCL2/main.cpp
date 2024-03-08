#include <iostream>
#include <boost/thread/thread.hpp>
#include <string>
#include<ctime>
#include<cstdlib>

#include"configuration.h"
#include"pretreatment.h"
#include"regPosRegulation.h"
#include"regCut.h"
#include"reconstruction.h"
#include"RegAlgorithm.h"
#include"measurement.h"

clock_t start, end;
double endtime;
//pig_complete 认定体尺分辨率


int main(int argc, char** argv)
{
	/*
	configuration();
	
	/////pig_side预处理///////设置参数 直通滤波y、z 
	start = clock();
	pretreatment();
	
	regPosRegulation();
	
	
	regCut();
	

	RegAlgorithm();*/

	reconstruction();
	/*
	measurement();
	*/
	system("PAUSE");
	return (0);
}