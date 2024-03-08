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
//pig_complete �϶���߷ֱ���


int main(int argc, char** argv)
{
	/*
	configuration();
	
	/////pig_sideԤ����///////���ò��� ֱͨ�˲�y��z 
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