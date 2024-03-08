#ifndef PLYTOPCD_H
#define PLYTOPCD_H

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>


void PLYtoPCD(char pcdname[30],char savename[30])
{
	pcl::PCLPointCloud2 clod;

	pcl::PLYReader reader;
	reader.read(pcdname, clod);
	pcl::PCDWriter writer;
	writer.writeASCII(savename, clod);
}

#endif
