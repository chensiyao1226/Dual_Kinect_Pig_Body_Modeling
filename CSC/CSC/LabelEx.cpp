std::map<Label, std::vector<size_t>> overlap;
std::map<Label, Segment> overlap_map_;

for (size_t i = 0; i < groundtruth_cloud_.size(); ++i)
	overlap_map_[groundtruth_cloud_.at(i).label].overlapsWith(segmentation_cloud_.at(i).label, i);

return overlap.size();
typedef uint32_t Label;
std::pair<Label, size_t> getLargestOverlap() const
{
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
}
