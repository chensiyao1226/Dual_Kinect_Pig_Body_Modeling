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
	///////配准用 去掉头部、主体中间、尾部部分////////
	//////////////side测量////////////
	float min_s_z = 10, max_s_z = -10, length_side, cut_sHeadF, cut_sHeadB, cut_sButtF, cut_sButtB;
	peakExtra("pig_side(最小二乘Trans).pcd", 2, min_s_z, max_s_z);//整猪侧视图提取长度
	length_side = max_s_z - min_s_z;

	////////////top测量/////////////
	float min_t_y = 10, max_t_y = -10;
	peakExtra("pig_top(最小二乘Trans).pcd", 1, min_t_y, max_t_y);//整猪俯视提取宽度
	Sfilter2("pig_top(最小二乘trans).pcd", "y", -2, -(max_t_y - min_t_y)/6, "pig_top(二乘transHalf).pcd");//猪左侧02 猪右侧-20//改
	//pcd_display3("pig_top(最小二乘trans).pcd", "pig_top(二乘transHalf).pcd");

	float min_t_z = 10, max_t_z = -10, length_top, cut_tHeadF, cut_tHeadB, cut_tButtF, cut_tButtB;
	peakExtra("pig_top(最小二乘trans).pcd", 2, min_t_z, max_t_z);
	length_top = max_t_z - min_t_z;
	cout << "length_side - length_top = " << length_side - length_top << endl;
	///side top 尾部等长化////认为整体视图长度差距是由于尾巴的复杂形状 边缘失准
	Sfilter2("pig_top(最小二乘trans).pcd", "z", min_t_z + (length_top - length_side), 3, "pig_top(二乘SameL).pcd");//top//尾巴等长化
	Sfilter2("pig_side(最小二乘Trans).pcd", "z", min_s_z + (length_side - length_top), 3, "pig_side(二乘SameL).pcd");//side//尾巴等长化


	///等长后side测量///
	peakExtra("pig_side(二乘SameL).pcd", 2, min_s_z, max_s_z);
	cut_sHeadF = max_s_z ;//头部前侧切点
	cut_sHeadB = max_s_z - (max_s_z - min_s_z) / 3;//头部后侧切点
	cut_sButtF = min_s_z + (max_s_z - min_s_z) / 3;//臀部前侧切点
	cut_sButtB = min_s_z;//臀部后侧切点
	///等长后top测量///
	peakExtra("pig_top(二乘SameL).pcd", 2, min_t_z, max_t_z);
	cut_tHeadF = max_t_z ;//头部前侧切点
	cut_tHeadB = max_t_z - (max_t_z - min_t_z) / 3;//头部后侧切点
	cut_tButtF = min_t_z + (max_t_z - min_t_z) / 3;//臀部前侧切点
	cut_tButtB = min_t_z;//臀部后侧切点

	//cut_tpN = max_t_z - length_top / 3.0;
	//cut_tpF = max_t_z - length_top / 12.0;
	//cut_tpB = min_t_z + length_top / 3;

	///////////side 截取/////////////
	Sfilter2("pig_side(二乘transHalf).pcd", "z", cut_sHeadB, cut_sHeadF, "pig_side(二乘transHalfHead).pcd");//截取耳朵和前背部
	Sfilter2("pig_side(二乘transHalf).pcd", "z", cut_sButtB, cut_sButtF, "pig_side(二乘transHalfButt).pcd");//截取臀部


	pcl::PointCloud<pcl::PointXYZL>::Ptr cloud_side_head(new pcl::PointCloud<pcl::PointXYZL>);//
	pcl::PointCloud<pcl::PointXYZL>::Ptr cloud_side_butt(new pcl::PointCloud<pcl::PointXYZL>);
	pcl::PointCloud<pcl::PointXYZL>::Ptr cloud_side_HB(new pcl::PointCloud<pcl::PointXYZL>);
	pcl::io::loadPCDFile("pig_side(二乘transHalfHead).pcd", *cloud_side_head);
	pcl::io::loadPCDFile("pig_side(二乘transHalfButt).pcd", *cloud_side_butt);
	*cloud_side_HB = *cloud_side_head;
	*cloud_side_HB += *cloud_side_butt;


	writer.write<pcl::PointXYZL>("pig_side(二乘transHalfHB).pcd", *cloud_side_HB, false);//



	//////////////top截取/////////////////
	Sfilter2("pig_top(二乘transHalf).pcd", "z", cut_tHeadB, cut_tHeadF, "pig_top(二乘transHalfHead).pcd");//滤掉头部 保留耳朵和主体  0.45
	Sfilter2("pig_top(二乘transHalf).pcd", "z", cut_tButtB, cut_tButtF, "pig_top(二乘transHalfButt).pcd");

	pcl::PointCloud<pcl::PointXYZL>::Ptr cloud_top_head(new pcl::PointCloud<pcl::PointXYZL>);//
	pcl::PointCloud<pcl::PointXYZL>::Ptr cloud_top_butt(new pcl::PointCloud<pcl::PointXYZL>);
	pcl::PointCloud<pcl::PointXYZL>::Ptr cloud_top_HB(new pcl::PointCloud<pcl::PointXYZL>);
	pcl::io::loadPCDFile("pig_top(二乘transHalfHead).pcd", *cloud_top_head);
	pcl::io::loadPCDFile("pig_top(二乘transHalfButt).pcd", *cloud_top_butt);
	*cloud_top_HB = *cloud_top_head;
	*cloud_top_HB += *cloud_top_butt;


	writer.write<pcl::PointXYZL>("pig_top(二乘transHalfHB).pcd", *cloud_top_HB, false);//


	//pcd_display2("pig_side(二乘transHalfHB).pcd", "pig_top(二乘transHalfHB).pcd");
	//pcd_display3("pig_side(二乘transHalfHB).pcd", "pig_top(二乘transHalfHB).pcd");

	//pcd_display2("pig_side(最小二乘trans).pcd", "pig_top(最小二乘trans).pcd");
	//截取除头以外配准 因为头弧度太大 两视角遮挡多 配准失准
	//尝试减少主体部分 利用脖颈部、臀部进行配准
	//不用考虑歪头、甩尾 造成的直通滤波丢失部位 只用尾根和脖颈配准
	//不需要提取max min定值
	//这样也能减少配准时间
	//pcd_display("pig_top(最小二乘trans).pcd", "trans", 0);
	
}