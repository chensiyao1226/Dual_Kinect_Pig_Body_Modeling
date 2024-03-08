
#ifndef PCL_GRAPH_POINT_CLOUD_GRAPH_CONCEPT_H
#define PCL_GRAPH_POINT_CLOUD_GRAPH_CONCEPT_H

#include <boost/graph/graph_concepts.hpp>
#include <boost/concept/detail/concept_def.hpp>

namespace pcl
{

   namespace graph
   {

 namespace concepts
 {

      BOOST_concept(PointCloudGraph, (G))
		       : boost::concepts::Graph<G>
				  {
				
				        typedef typename boost::vertex_bundle_type<G>::type vertex_bundled;
				    typedef typename boost::graph_traits<G>::vertex_descriptor vertex_descriptor;
				    typedef typename point_cloud_graph_traits<G>::point_type point_type;
				    typedef typename point_cloud_graph_traits<G>::point_cloud_type point_cloud_type;
				    typedef typename point_cloud_graph_traits<G>::point_cloud_ptr point_cloud_ptr;
				    typedef typename point_cloud_graph_traits<G>::point_cloud_const_ptr point_cloud_const_ptr;
				
				        BOOST_STATIC_ASSERT((boost::mpl::not_<boost::is_same<vertex_bundled, boost::no_property> >::value));
				    BOOST_STATIC_ASSERT((boost::is_same<vertex_bundled, point_type>::value));
				
				        BOOST_CONCEPT_USAGE(PointCloudGraph)
				        {
				          BOOST_CONCEPT_ASSERT((boost::concepts::PropertyGraph<G, vertex_descriptor, boost::vertex_bundle_t>));
				          p = point_cloud(g);
				          i = indices(g);
				          G a(p); // require that graph can be constructed from a point cloud pointer
				          const_constraints(g);
				        }
				
				        void const_constraints(const G& cg)
				        {
				          pc = point_cloud(cg);
				          i = indices(cg);
				        }
				
				        G g;
				    point_cloud_ptr p;
				    point_cloud_const_ptr pc;
				    pcl::PointIndices::Ptr i;
				
				      };
			
			     } // namespace concepts
	
	     using pcl::graph::concepts::PointCloudGraphConcept;
	
	   } // namespace graph
	
	 } // namespace pcl

#include <boost/concept/detail/concept_undef.hpp>

#endif /* PCL_GRAPH_POINT_CLOUD_GRAPH_CONCEPT_H */

