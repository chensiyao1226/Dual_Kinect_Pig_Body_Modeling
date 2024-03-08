#ifndef LABELEXTR_H
#define LABELEXTR_H

#include <string>

#include <pcl/console/parse.h>
#include <pcl/console/print.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

typedef unsigned int Label;
typedef unsigned long long size_t;

unsigned int LabelExtr(char pcdname[30])
{
	pcl::PointCloud<pcl::PointXYZL> cloud;
	pcl::io::loadPCDFile(pcdname, cloud);

	std::map<Label, std::vector<size_t>> overlap;
	Label label;
	size_t max = 0;
	for (size_t i = 0; i < cloud.size(); ++i)
	{
		auto& ps = cloud.at(i);
		if (pcl::isFinite(ps))
			overlap[ps.label].push_back(i);
		if (overlap[ps.label].size() > max)
		{
			label = ps.label;//first<label>
			max = overlap[ps.label].size();//second<std::vector<size_t>>
		}
	}
	return(label);
}
#endif

/*std::pair<Label, size_t> getMostLabel
Label label;
size_t max = 0;
for (const auto& li_pair : overlap)//¿½±´
{
	if (li_pair.second.size() > max)
	{
		label = li_pair.first;//first<label>
		max = li_pair.second.size();//second<std::vector<size_t>>
	}
}
return std::make_pair(label, max);



void overlapsWith(Label l, size_t index)
{
	size += 1;
	if (overlap.count(l))//////////
		overlap[l].push_back(index);///
	else
		overlap[l] = { index };
}
*/



