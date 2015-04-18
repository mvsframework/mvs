// 
//  Copyright © 2015 Claus Christmann <hcc |ä| gatech.edu>.
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
// 

#ifndef __CREATE_DUAL_2D_GRAPH_HPP__
#define __CREATE_DUAL_2D_GRAPH_HPP__
 
#include <vector>
 
#include <boost/config.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/property_map/property_map.hpp>
#include <boost/graph/planar_face_traversal.hpp>

#include "Geometry/geometry.h"

namespace Graph {

  
template< typename VertexPositionMap,
          typename Ring  = Geometry::Polygon_2D::ring_type
        >
class FaceGeometry_visitor : public boost::planar_face_traversal_visitor
{
public:  
  
  using Point = boost::range_value<Ring>::type;
  
  FaceGeometry_visitor(VertexPositionMap vertexPositionMap)
  :pos_map(vertexPositionMap)
  {
    using namespace boost;
    BOOST_CONCEPT_ASSERT(( geometry::concept::Ring< typename Ring > ));
    BOOST_CONCEPT_ASSERT(( geometry::concept::Point< typename Point > ));
    
//     BOOST_CONCEPT_ASSERT(( ));
    BOOST_CONCEPT_ASSERT(( geometry::concept::Point< typename property_traits<PositionMap>::value_type >  ));
  };
  
  void begin_face()
  {
    // start by creating a new face geometry 
    face_geometry.clear();
  };
  

  
  void end_face()
  {
    boost::geometry::correct(face_geometry);
    geometries.push_back(face_geometry); 
    
//       std::cout 
//         << boost::geometry::dsv(face_geometry) 
//         << " -> " 
//         << boost::geometry::area(face_geometry) << std::endl;
  };
  
  std::vector<FaceGeometry> getFaceGeometries() const
  { return geometries; };
  
  
protected:
  Ring face_geometry;
  VertexPositionMap pos_map;
  std::vector<Ring> geometries;
  
};  
  
  
/** \brief A planar graph visitor to extract the 2D positioned dual graph.
  * 
  * \note The visitor logic to create the dual graph is copy-and-pasted from 
  * http://aawblog.wordpress.com/2009/12/16/boost-graph-library-planar-dual/
  * (Retrieved 8/16/2013)
  * 
  */
template< typename InputGraph,
          typename OutputGraph,
          typename EdgeIndexMap,
          typename VertexPositionMap>
class dual_graph_2D_visitor : public FaceGeometry_visitor<VertexPositionMap>
{
public:
  
  using vertex_t  = typename boost::graph_traits<OutputGraph>::vertex_descriptor;
  using edge_t    = typename boost::graph_traits<InputGraph>::edge_descriptor;
  using vertex_vector_t = typename std::vector<vertex_t>;
  using edge_to_face_map_t = boost::iterator_property_map< typename vertex_vector_t::iterator, EdgeIndexMap >;

  dual_graph_2D_visitor(InputGraph& graph_in,
                        OutputGraph& graphDual_out,
                        EdgeIndexMap edgeIndexMap,
                        VertexPositionMap vertexPosMap
                        )
    :FaceGeometry_visitor<VertexPositionMap>(vertexPosMap)
    ,g(graph_in)
    ,dual_g(graphDual_out)
    ,em(edgeIndexMap)
    ,edge_to_face_vector(boost::num_edges(g), boost::graph_traits<OutputGraph>::null_vertex())
    ,edge_to_face( edge_to_face_vector.begin(), em)
  {};
  ~dual_graph_2D_visitor() = default;
  
  void begin_face()
  {
    FaceGeometry_visitor<VertexPositionMap>::begin_face();
    current_face = boost::add_vertex(dual_g);
  };
  
  template< typename Edge >
  void next_edge(Edge e)
  {
    vertex_t existing_face = edge_to_face[e];
    if (existing_face == boost::graph_traits<OutputGraph>::null_vertex())
    {
      edge_to_face[e] = current_face;
    }
    else
    {
      boost::add_edge(existing_face, current_face, dual_g);
    }
  };
  
  void end_face()
  {
    FaceGeometry_visitor<VertexPositionMap>::end_face();
    Geometry::Point_2D face_centroid;
    boost::geometry::centroid(FaceGeometry_visitor<VertexPositionMap>::face_geometry,face_centroid);
    
    boost::put(boost::vertex_position_2D,dual_g,current_face,face_centroid);
  };
  
protected:
  
  
  InputGraph& g; ///< \brief The Voronoi graph.
  OutputGraph& dual_g; ///< \brief  The obstacle graph.
  EdgeIndexMap em;
  vertex_t current_face; ///< \brief The currently processed face of the Voronoi graph.
  vertex_vector_t edge_to_face_vector;
  edge_to_face_map_t edge_to_face;
};


template <typename InputGraph,
          typename OutputGraph,
          typename PlanarEmbedding,
          typename EdgeIndexMap,
          typename VertexPositionMap
          >
void create_dual_2D_graph(InputGraph& g,
                          OutputGraph& dual_g,
                          PlanarEmbedding embedding,
                          EdgeIndexMap em,
                          VertexPositionMap pm)
{
  using namespace boost;

  dual_graph_2D_visitor<InputGraph, OutputGraph, EdgeIndexMap, VertexPositionMap>
  visitor(g, dual_g, em, pm);
  planar_face_traversal(g, embedding, visitor, em);
}

template <typename InputGraph,
          typename OutputGraph,
          typename PlanarEmbedding
          >
void create_dual_2D_graph(InputGraph& g,
                          OutputGraph& dual_g,
                          PlanarEmbedding embedding
                          )
{
  using namespace boost;
  create_dual_2D_graph(g, dual_g, embedding,
                        get(edge_index,g),
                        get(vertex_position_2D,g)
                      );
}

  
} // namespace Graph
 
#endif //__CREATE_DUAL_2D_GRAPH_HPP__