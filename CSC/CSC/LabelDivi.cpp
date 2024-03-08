if (cloud->points[ii].label == 1)
{
	cloud1->push_back(cloud->points[ii]);
}
lccp_segma.hpp label_ID_map[sv_label] = node_id;
largest_neigh_size = seg_label_to_sv_list_map_[*neighbors_itr].size();

<pre name = "code" class = "objc">#include<iostream>
#include<pcl\io\io.h>
#include<pcl\point_cloud.h>
#include<pcl\point_types.h>
#include<pcl\filters\conditional_removal.h>
using namespace std;
int main()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	cloud->width = 5;
	cloud->height = 1;
	cloud->points.resize(cloud->width*cloud->height);
	for (size_t i = 0; i<cloud->points.size(); i++)
	{
		cloud->points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
		cloud->points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
		cloud->points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
	}
	//创建条件限定下的滤波器
	pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond(new pcl::ConditionAnd<pcl::PointXYZ>);
	pcl::FieldComparison<pcl::PointXYZ>::ConstPtr cond_1(new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::GT, 0.0));
	range_cond->addComparison(cond_1);
	pcl::FieldComparison<pcl::PointXYZ>::ConstPtr cond_2(new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::LT, 0.8));
	range_cond->addComparison(cond_2);
	//创建滤波器并用条件定义对象初始化
	pcl::ConditionalRemoval<pcl::PointXYZ> condrem(range_cond);
	condrem.setInputCloud(cloud);
	condrem.setKeepOrganized(true);
	condrem.filter(*cloud_filtered);
	std::cerr << "cloud before filtering:" << std::endl;
	for (size_t i = 0; i<cloud->points.size(); i++)
		std::cerr << ' ' << cloud->points[i].x << ' ' << cloud->points[i].y << ' ' << cloud->points[i].z << std::endl;
	std::cerr << "cloud after filtering:" << std::endl;
	for (size_t i = 0; i<cloud_filtered->points.size(); i++)
		std::cerr << ' ' << cloud_filtered->points[i].x << ' ' << cloud_filtered->points[i].y << ' ' << cloud->points[i].z << std::endl;
	system("pause");
	return 0;
}


xperience
Main Page
Related Pages
Modules
+ Namespaces
+ Data Structures
+ Files

Search
main
src
armarx
VisionX
source
VisionX
components
pointcloud_core
PCLUtilities.h
1 /*
  2  * This file is part of ArmarX.
  3  *
  4  * ArmarX is free software; you can redistribute it and/or modify
  5  * it under the terms of the GNU General Public License version 2 as
  6  * published by the Free Software Foundation.
  7  *
  8  * ArmarX is distributed in the hope that it will be useful, but
  9  * WITHOUT ANY WARRANTY; without even the implied warranty of
  10  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  11  * GNU General Public License for more details.
  12  *
  13  * You should have received a copy of the GNU General Public License
  14  * along with this program. If not, see <http://www.gnu.org/licenses/>.
  15  *
  16  * @package    VisionX::Tools
  17  * @author     Markus Grotz ( markus dot grotz at kit dot edu )
  18  * @date       2015
  19  * @copyright  http://www.gnu.org/licenses/gpl-2.0.txt
  20  *             GNU General Public License
  21  */
  22 #ifndef VISIONX_PCLHELPER_H_
  23 #define VISIONX_PCLHELPER_H_
  24
  25 #include <map>
  26 #include <tuple>
  27 #include <iostream>
  28
  29 #include <pcl/point_types.h>
  30 #include <pcl/point_cloud.h>
  31 #include <pcl/common/io.h>
  32 #include <pcl/io/pcd_io.h>
  33
  34 #ifdef PCL_COMMON_COLORS_H
  35 #include <pcl/common/colors.h>
  36 #endif
  37
  38 namespace visionx
  39 {
	40
		41     namespace tools
		42     {
		43
			44
			45         template<typename PointT>
			46         std::tuple<uint8_t, uint8_t, uint8_t> colorizeSegment(typename pcl::PointCloud<PointT>::Ptr& segment)
			47         {
			48             const uint8_t r = rand() % 255;
			49             const uint8_t g = rand() % 255;
			50             const uint8_t b = rand() % 255;
			51
				52             for (auto& p : segment->points)
				53             {
				54                 p.r = r;
				55                 p.g = g;
				56                 p.b = b;
				57             }
			58
				59             return std::make_tuple(r, g, b);
			60         }
		61
			62
			63         void colorizeLabeledPointCloud(pcl::PointCloud<pcl::PointXYZL>::Ptr sourceCloudPtr, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr targetCloudPtr)
			64         {
			65             pcl::copyPointCloud(*sourceCloudPtr, *targetCloudPtr);
			66
				67             std::map<uint32_t, float> colorMap;
			68
				69             for (size_t i = 0; i < sourceCloudPtr->points.size(); i++)
				70             {
				71                 pcl::PointXYZL p = sourceCloudPtr->points[i];
				72
					73                 if (!colorMap.count(p.label))
					74                 {
					75 #ifdef PCL_COMMON_COLORS_H
						76                     pcl::RGB c = pcl::GlasbeyLUT::at(p.label % pcl::GlasbeyLUT::size());
					77                     colorMap.insert(std::make_pair(p.label, c.rgb));
					78 #else
						79                     const uint8_t r = rand() % 255;
					80                     const uint8_t g = rand() % 255;
					81                     const uint8_t b = rand() % 255;
					82                     float color = r << 16 | g << 8 | b;
					83                     colorMap.insert(std::make_pair(p.label, color));
					84 #endif
						85                 }
				86
					87                 targetCloudPtr->points[i].rgb = colorMap[p.label];
				88             }
			89         }
		90
			91         template <typename PointCloudType>
			92         void fillLabelMap(PointCloudType labeledCloudPtr, std::map<uint32_t, pcl::PointIndices>& labeledPointMap)
			93         {
			94             for (size_t i = 0; i < labeledCloudPtr->points.size(); i++)
				95             {
				96                 uint32_t currentLabel = labeledCloudPtr->points[i].label;
				97
					98                 if (!currentLabel)
					99                 {
					100                     continue;
					101                 }
				102                 else if (labeledPointMap.count(currentLabel))
					103                 {
					104                     labeledPointMap[currentLabel].indices.push_back(i);
					105                 }
				106                 else
					107                 {
					108                     pcl::PointIndices labelIndices;
					109                     labelIndices.indices.push_back(i);
					110                     labeledPointMap.insert(std::make_pair(currentLabel, labelIndices));
					111                 }
				112             }
			113         }
		114
			115
			116
			117
			118     }
	119 }
120
121 #endif





#include <string>

#include <pcl/console/parse.h>
#include <pcl/console/print.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <pcl/visualization/pcl_visualizer.h>

#include "tviewer/tviewer.h"
#include "hungarian/hungarian.h"

using namespace tviewer;

class SegmentationEvaluation
{

public:

	SegmentationEvaluation(const std::string& segmentation, const std::string& groundtruth)
		: filename_segmentation_(segmentation)
		, filename_groundtruth_(groundtruth)
	{
	}

	int run(bool with_gui = false)
	{
		if (!loadCloud(filename_segmentation_, segmentation_cloud_) ||
			!loadCloud(filename_groundtruth_, groundtruth_cloud_))
			return (1);

		assert(segmentation_cloud_.points.size() == groundtruth_cloud_.points.size());

		computeOverlap();
		computeError();

		size_t error = error_indices_.size();

		if (with_gui)
		{
			auto viewer = create();
			size_t size = getRealCloudSize(groundtruth_cloud_);
			float p = 100.0 * error / size;
			pcl::console::print_error("Erroneus points: %i (%.2f%% of %zu)\n", error, p, size);
			viewer->add
				(CreatePointCloudObject<pcl::PointXYZRGBA>("incorrect", "i")
				.description("Incorrectly segmented points")
				.data(getCloud(MODE_BY_ERROR_FLAG))
				.pointSize(2)
				.visibility(0.95)
				);
			viewer->add
				(CreatePointCloudWithColorShufflingObject("groundtruth", "t")
				.description("Groundtruth labeling")
				.data(getCloud(MODE_BY_GROUNDTRUTH_LABEL))
				.pointSize(2)
				.visibility(0.95)
				);
			viewer->add
				(CreatePointCloudWithColorShufflingObject("segmentation", "s")
				.description("Segmentation labeling")
				.data(getCloud(MODE_BY_SEGMENTATION_LABEL))
				.pointSize(2)
				.visibility(0.95)
				);
			viewer->show("incorrect");
			viewer->run();
		}
		else
		{
			std::cout << error << "\n";
		}

		return (0);
	}

private:

	typedef uint32_t Label;
	typedef std::map<Label, Color> LabelColorMap;
	typedef std::map<size_t, Color> IndexColorMap;

	enum Mode
	{
		MODE_BY_SEGMENTATION_LABEL,
		MODE_BY_GROUNDTRUTH_LABEL,
		MODE_BY_ERROR_FLAG,
	};

	struct Segment
	{
		// Number of vertices
		size_t size;

		// Overlaps with other labels
		std::map<Label, std::vector<size_t>> overlap;

		// Register an overlap with a point
		void overlapsWith(Label l, size_t index)
		{
			size += 1;
			if (overlap.count(l))
				overlap[l].push_back(index);
			else
				overlap[l] = { index };
		}

		// Get number of different labels with which segment overlaps
		size_t getNumberOfOverlaps() const
		{
			return overlap.size();
		}

		std::pair<Label, size_t> getLargestOverlap() const
		{
			Label label;
			size_t max = 0;
			for (const auto& li_pair : overlap)
			{
				if (li_pair.second.size() > max)
				{
					label = li_pair.first;
					max = li_pair.second.size();
				}
			}
			return std::make_pair(label, max);
		}
	};

	static bool
		loadCloud(const std::string& filename, pcl::PointCloud<pcl::PointXYZL>& cloud)
	{
		if (pcl::io::loadPCDFile(filename, cloud) < 0)
			return (false);
		return (true);
	}

	void computeOverlap()
	{
		for (size_t i = 0; i < groundtruth_cloud_.size(); ++i)
		{
			auto& pg = groundtruth_cloud_.at(i);
			auto& ps = segmentation_cloud_.at(i);
			if (pcl::isFinite(pg))
				overlap_map_[pg.label].overlapsWith(ps.label, i);
		}
	}

	void computeError()
	{
		typedef std::tuple<Label, Label, size_t> Triple;
		std::vector<Triple> triples;
		for (const auto& label_segment_pair : overlap_map_)
			for (const auto& label_indices_pair : label_segment_pair.second.overlap)
				triples.push_back(Triple(label_segment_pair.first,
				label_indices_pair.first,
				label_indices_pair.second.size()));
		auto assignment = findAssignment(triples);
		error_indices_.clear();
		for (const auto& label_segment_pair : overlap_map_)
			for (const auto& label_indices_pair : label_segment_pair.second.overlap)
				if (label_indices_pair.first != assignment[label_segment_pair.first])
					error_indices_.insert(error_indices_.end(), label_indices_pair.second.begin(), label_indices_pair.second.end());
	}

	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr colorizeByLabel(const pcl::PointCloud<pcl::PointXYZL>& original, LabelColorMap& color_map)
	{
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
		pcl::copyPointCloud(original, *cloud);
		for (size_t i = 0; i < cloud->size(); i++)
		{
			cloud->at(i).rgba = color_map[original.at(i).label];
		}
		return cloud;
	}

	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr colorizeByIndex(const pcl::PointCloud<pcl::PointXYZL>& original, IndexColorMap& color_map)
	{
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
		pcl::copyPointCloud(original, *cloud);
		for (size_t i = 0; i < cloud->size(); i++)
		{
			cloud->at(i).rgba = color_map[i];
		}
		return cloud;
	}

	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr getCloud(Mode mode)
	{
		switch (mode)
		{
		case MODE_BY_SEGMENTATION_LABEL:
		{
			std::map<Label, Color> label_map;
			for (const auto& pt : segmentation_cloud_)
			{
				Label label = pt.label;
				if (!label_map.count(label))
					label_map[label] = generateRandomColor();
			}
			return colorizeByLabel(segmentation_cloud_, label_map);
		}
		case MODE_BY_GROUNDTRUTH_LABEL:
		{
			std::map<Label, Color> label_map;
			for (const auto& pt : groundtruth_cloud_)
			{
				Label label = pt.label;
				if (!label_map.count(label))
					label_map[label] = generateRandomColor();
			}
			return colorizeByLabel(groundtruth_cloud_, label_map);
		}
		case MODE_BY_ERROR_FLAG:
		{
			std::map<size_t, Color> index_map;
			Color normal = 0x0EEBA1;
			Color error = 0xEB0E58;
			for (size_t i = 0; i < groundtruth_cloud_.size(); i++)
			{
				index_map[i] = normal;
			}
			for (size_t i = 0; i < error_indices_.size(); i++)
			{
				index_map[error_indices_[i]] = error;
			}
			return colorizeByIndex(groundtruth_cloud_, index_map);
		}
		}
	}

	size_t
		getRealCloudSize(const pcl::PointCloud<pcl::PointXYZL>& cloud)
	{
		size_t cnt = 0;
		for (const auto& point : cloud.points)
			if (pcl::isFinite(point))
				++cnt;
		return cnt;
	}

	std::map<Label, Segment> overlap_map_;
	std::vector<size_t> error_indices_;

	std::string filename_segmentation_;
	std::string filename_groundtruth_;

	pcl::PointCloud<pcl::PointXYZL> segmentation_cloud_;
	pcl::PointCloud<pcl::PointXYZL> groundtruth_cloud_;

};


int main(int argc, char** argv)
{
	std::string segmentation, groundtruth;

	if (argc >= 3)
	{
		segmentation = argv[1];
		groundtruth = argv[2];
	}
	else
	{
		pcl::console::print_error("Usage: %s segmentation groundtruth [options]\n", argv[0]);
		return (1);
	}

	bool with_gui = pcl::console::find_switch(argc, argv, "--gui");

	SegmentationEvaluation se(segmentation, groundtruth);

	return (se.run(with_gui));
}