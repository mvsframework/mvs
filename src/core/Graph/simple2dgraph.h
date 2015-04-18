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

#ifndef SIMPLEGRAPH_H
#define SIMPLEGRAPH_H


#include <boost/graph/adjacency_list.hpp>
#include <boost/tuple/tuple.hpp> // provides tie(a,b)

#include "Graph/properties.h"
#include "Geometry/geometry.h" // provides Geometry::Point_2D


/** \brief The dedicated namespace for a simple 2D graph. */
namespace Simple2dGraph {

using namespace boost;  

//
// Properties
//

///< \brief Vertex descriptor type of a SimpleGraph.
using Vertex_Descriptor = adjacency_list_traits<setS, listS, undirectedS>::vertex_descriptor; 


/** \brief The (nested) internal vertex properties of the graph. 
 * \todo The vertex coordinates don't make much sense withouth some sort of 
 * reference, i.e. a datum.
 */
using Vertex_Properties = // (nested) properties
   property<vertex_index_t,         unsigned int  
  ,property<vertex_position_2D_t,   Geometry::Point_2D
  > >;  
  
/** \brief The (nested) internal edge properties of the graph. */  
using Edge_Properties = 
   property<edge_index_t,           unsigned int  
  >;
//
// The Graph
//

/** \brief The actual Simple2dGraph graph type.
 * 
 * In general, if you want your vertex and edge descriptors to be stable (never 
 * invalidated) then use listS or setS for the VertexList and OutEdgeList 
 * template parameters of adjacency_list. If you are not as concerned about 
 * descriptor and iterator stability, and are more concerned about memory 
 * consumption and graph traversal speed, use vecS for the VertexList and/or 
 * OutEdgeList template parameters.
 * 
 * \tparam OutEdgeList The selector for the container used to represent the 
 *  edge-list for each of the vertices. (default: \e vecS)
 * 
 * \tparam VertexList The selector for the container used to represent the 
 *  vertex-list of the graph. (default: \e vecS)
 * 
 * \tparam Directed A selector to choose whether the graph is directed, 
 *  undirected, or directed with bidirectional edge access (access to both 
 *  out-edges and in-edges). The options are \e directedS, \e undirectedS, and 
 *  \e bidirectionalS. (default: \e directedS)
 * 
 * \tparam VertexProperties for specifying internal property storage. (default
 *  \e no_property)
 * 
 * \tparam EdgeProperties for specifying internal property storage. (default
 *  \e no_property)
 * 
 * \tparam GraphProperties for specifying property storage for the graph object.
 *  (default: \e no_property)
 * 
 * \tparam EdgeList The selector for the container used to represent the 
 *  edge-list for the graph. (default: \e listS)
 */  
using Graph =  adjacency_list<
  /* OutEdgeList      */ setS // setS dissallows parallel edges
  /* VertexList       */,listS 
  /* Directed         */,undirectedS 
  /* VertexProperties */,Vertex_Properties
  /* EdgeProperties   */,Edge_Properties
  /* GraphProperties  */
  /* EdgeList         */
  > ;

//
// Graph Traits
//
using Graph_Traits = graph_traits<Graph>;
using Vertex_Iterator  = Graph_Traits::vertex_iterator;   ///< \brief Vertex iterator type of a RouteGraph.
using Edge_Descriptor  = Graph_Traits::edge_descriptor;   ///< \brief Edge descriptor type of a RouteGraph.
using Edge_Iterator    = Graph_Traits::edge_iterator;   ///< \brief Edge descriptor type of a RouteGraph.
  

//
// Property Maps
//

using Vertex_Index_Map        = property_map<Graph,vertex_index_t>::type;
using Vertex_Position2d_Map   = property_map<Graph,vertex_position_2D_t>::type;

using Edge_Index_Map          = property_map<Graph,edge_index_t>::type;
// using Edge_Weigth_Map         = property_map<Graph,edge_weight_t>::type;

} // end namespace Simple2dGraph


#endif // SIMPLEGRAPH_H
