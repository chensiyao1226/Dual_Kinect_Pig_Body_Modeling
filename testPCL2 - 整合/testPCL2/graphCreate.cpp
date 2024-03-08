#ifndef GRAPHCTREATE_H
#define  GRAPHCTREATE_H

#include <iostream>
#include <cmath>
#include <stdlib.h>
#include <fstream>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>

#include <boost/config.hpp>  
#include <boost/graph/graph_traits.hpp>  
#include <boost/graph/adjacency_list.hpp>  
#include <boost/graph/dijkstra_shortest_paths.hpp>  

#include "point_cloud_graph.h"
#include "nearest_neighbors_graph_builder.h"
#include "edge_weight_computer.h"
using namespace boost;
using namespace std;
float graphCreate(int p1, int p2, char pcdname[30])//p1,p2=index
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile(pcdname, *cloud);

	typedef property<vertex_name_t, int, property<vertex_index2_t, int> > VertexProperties;
	typedef pcl::graph::point_cloud_graph<pcl::PointXYZ, boost::vecS, boost::undirectedS,
		VertexProperties, boost::property<boost::edge_weight_t, float>, boost::listS> Graph;
	typedef boost::property_map<Graph, vertex_index_t>::type VertexMap;
	typedef pcl::graph::NearestNeighborsGraphBuilder<pcl::PointXYZ, Graph> Builder;
	typedef boost::property_map<Graph, boost::edge_weight_t>::type EdgeWeightMap;
	typedef graph_traits<Graph>::vertex_iterator vertex_iter;
	Graph g;
	VertexMap vertexmap;
	EdgeWeightMap weightmap;
	Graph::vertex_descriptor pt1, pt2;
	Builder graph_builder;
	int num_nodes = num_vertices(g);
	float distance;
	vertexmap = get(vertex_index, g);

	pt1 = vertex(p1, g);
	pt2 = vertex(p2, g);

	//pcl::graph::point_cloud_graph<pcl::PointXYZ> g(cloud);
	//float num_edge = num_edges(g);

	//graph_t g;

	graph_builder.setInputCloud(cloud);
	graph_builder.setNumberOfNeighbors(10);
	graph_builder.useNearestKSearch();

	cout << "computing distance..." << endl;
	graph_builder.compute(g);

	//float num_edge1 = num_edges(g);


	weightmap = get(edge_weight, g);


	graph_traits<Graph>::edge_iterator ei, ei_end;
	int j = 0;
	for (tie(ei, ei_end) = edges(g); ei != ei_end; ++ei)
	{
		Graph::vertex_descriptor sd = source(*ei, g);
		Graph::vertex_descriptor td = target(*ei, g);
		float distance = sqrt((g[sd].x - g[td].x)*(g[sd].x - g[td].x) +
			(g[sd].y - g[td].y)*(g[sd].y - g[td].y) +
			(g[sd].z - g[td].z)*(g[sd].z - g[td].z));
		weightmap[*ei] = distance;
		//	std::cout << "(" << source(*ei, g) << "-->" << target(*ei, g) << ") " << "Edge has weight  " << distance << "==" << weightmap[*ei] << std::endl;

	}
	

	std::pair<vertex_iter, vertex_iter> vrange = vertices(g);
	Graph::vertex_descriptor s = *(vertices(g).first);
	std::vector<Graph::vertex_descriptor> p(num_vertices(g));
	std::vector<float> d(num_vertices(g));
	dijkstra_shortest_paths(g, pt2,
		predecessor_map(boost::make_iterator_property_map(p.begin(), get(boost::vertex_index, g))).
		distance_map(boost::make_iterator_property_map(d.begin(), get(boost::vertex_index, g))));

	//std::cout << "distances between two feature points: " << pt2 << std::endl;
	graph_traits < Graph>::vertex_iterator vi, vend;
	for (boost::tie(vi, vend) = vertices(g); vi != vend; ++vi)
	{
		if (*vi == pt1)
		{
			/*std::cout << "distance(" << *vi << ") = " << d[*vi] << ", ";
			std::cout << "parent(" << *vi << ") = " << p[*vi] << std::endl;*/
			distance = d[*vi];
		}
	}

	return distance;

}
#endif