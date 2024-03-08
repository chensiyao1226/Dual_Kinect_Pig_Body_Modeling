#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include "peakExtra.h"
#include "Sfilter2.h"
#include "pcd_display2.h"
#include "pcd_display3.h"

void regCut()
{
	pcl::PCDWriter writer;
	///////��׼�� ȥ��ͷ���������м䡢β������////////
	//////////////side����////////////
	float min_s_z = 10, max_s_z = -10, length_side, cut_sHeadF, cut_sHeadB, cut_sButtF, cut_sButtB;
	peakExtra("pig_side(��С����Trans).pcd", 2, min_s_z, max_s_z);//�������ͼ��ȡ����
	length_side = max_s_z - min_s_z;

	////////////top����/////////////
	float min_t_y = 10, max_t_y = -10;
	peakExtra("pig_top(��С����Trans).pcd", 1, min_t_y, max_t_y);//��������ȡ���
	Sfilter2("pig_top(��С����trans).pcd", "y", -2, -(max_t_y - min_t_y)/6, "pig_top(����transHalf).pcd");//�����02 ���Ҳ�-20//��
	//pcd_display3("pig_top(��С����trans).pcd", "pig_top(����transHalf).pcd");

	float min_t_z = 10, max_t_z = -10, length_top, cut_tHeadF, cut_tHeadB, cut_tButtF, cut_tButtB;
	peakExtra("pig_top(��С����trans).pcd", 2, min_t_z, max_t_z);
	length_top = max_t_z - min_t_z;
	cout << "length_side - length_top = " << length_side - length_top << endl;
	///side top β���ȳ���////��Ϊ������ͼ���Ȳ��������β�͵ĸ�����״ ��Եʧ׼
	Sfilter2("pig_top(��С����trans).pcd", "z", min_t_z + (length_top - length_side), 3, "pig_top(����SameL).pcd");//top//β�͵ȳ���
	Sfilter2("pig_side(��С����Trans).pcd", "z", min_s_z + (length_side - length_top), 3, "pig_side(����SameL).pcd");//side//β�͵ȳ���


	///�ȳ���side����///
	peakExtra("pig_side(����SameL).pcd", 2, min_s_z, max_s_z);
	cut_sHeadF = max_s_z ;//ͷ��ǰ���е�
	cut_sHeadB = max_s_z - (max_s_z - min_s_z) / 3;//ͷ������е�
	cut_sButtF = min_s_z + (max_s_z - min_s_z) / 3;//�β�ǰ���е�
	cut_sButtB = min_s_z;//�β�����е�
	///�ȳ���top����///
	peakExtra("pig_top(����SameL).pcd", 2, min_t_z, max_t_z);
	cut_tHeadF = max_t_z ;//ͷ��ǰ���е�
	cut_tHeadB = max_t_z - (max_t_z - min_t_z) / 3;//ͷ������е�
	cut_tButtF = min_t_z + (max_t_z - min_t_z) / 3;//�β�ǰ���е�
	cut_tButtB = min_t_z;//�β�����е�

	//cut_tpN = max_t_z - length_top / 3.0;
	//cut_tpF = max_t_z - length_top / 12.0;
	//cut_tpB = min_t_z + length_top / 3;

	///////////side ��ȡ/////////////
	Sfilter2("pig_side(����transHalf).pcd", "z", cut_sHeadB, cut_sHeadF, "pig_side(����transHalfHead).pcd");//��ȡ�����ǰ����
	Sfilter2("pig_side(����transHalf).pcd", "z", cut_sButtB, cut_sButtF, "pig_side(����transHalfButt).pcd");//��ȡ�β�


	pcl::PointCloud<pcl::PointXYZL>::Ptr cloud_side_head(new pcl::PointCloud<pcl::PointXYZL>);//
	pcl::PointCloud<pcl::PointXYZL>::Ptr cloud_side_butt(new pcl::PointCloud<pcl::PointXYZL>);
	pcl::PointCloud<pcl::PointXYZL>::Ptr cloud_side_HB(new pcl::PointCloud<pcl::PointXYZL>);
	pcl::io::loadPCDFile("pig_side(����transHalfHead).pcd", *cloud_side_head);
	pcl::io::loadPCDFile("pig_side(����transHalfButt).pcd", *cloud_side_butt);
	*cloud_side_HB = *cloud_side_head;
	*cloud_side_HB += *cloud_side_butt;


	writer.write<pcl::PointXYZL>("pig_side(����transHalfHB).pcd", *cloud_side_HB, false);//



	//////////////top��ȡ/////////////////
	Sfilter2("pig_top(����transHalf).pcd", "z", cut_tHeadB, cut_tHeadF, "pig_top(����transHalfHead).pcd");//�˵�ͷ�� �������������  0.45
	Sfilter2("pig_top(����transHalf).pcd", "z", cut_tButtB, cut_tButtF, "pig_top(����transHalfButt).pcd");

	pcl::PointCloud<pcl::PointXYZL>::Ptr cloud_top_head(new pcl::PointCloud<pcl::PointXYZL>);//
	pcl::PointCloud<pcl::PointXYZL>::Ptr cloud_top_butt(new pcl::PointCloud<pcl::PointXYZL>);
	pcl::PointCloud<pcl::PointXYZL>::Ptr cloud_top_HB(new pcl::PointCloud<pcl::PointXYZL>);
	pcl::io::loadPCDFile("pig_top(����transHalfHead).pcd", *cloud_top_head);
	pcl::io::loadPCDFile("pig_top(����transHalfButt).pcd", *cloud_top_butt);
	*cloud_top_HB = *cloud_top_head;
	*cloud_top_HB += *cloud_top_butt;


	writer.write<pcl::PointXYZL>("pig_top(����transHalfHB).pcd", *cloud_top_HB, false);//


	//pcd_display2("pig_side(����transHalfHB).pcd", "pig_top(����transHalfHB).pcd");
	//pcd_display3("pig_side(����transHalfHB).pcd", "pig_top(����transHalfHB).pcd");

	//pcd_display2("pig_side(��С����trans).pcd", "pig_top(��С����trans).pcd");
	//��ȡ��ͷ������׼ ��Ϊͷ����̫�� ���ӽ��ڵ��� ��׼ʧ׼
	//���Լ������岿�� ���ò��������β�������׼
	//���ÿ�����ͷ��˦β ��ɵ�ֱͨ�˲���ʧ��λ ֻ��β���Ͳ�����׼
	//����Ҫ��ȡmax min��ֵ
	//����Ҳ�ܼ�����׼ʱ��
	//pcd_display("pig_top(��С����trans).pcd", "trans", 0);
	
}