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

#ifndef ROUTEGRAPH_H
#define ROUTEGRAPH_H

#include "vronigraph.h"
namespace RouteGraph=VroniGraph;

#if 0

#include <boost/graph/adjacency_list.hpp>
#include <boost/tuple/tuple.hpp> // provides tie(a,b)

#include "Graph/properties.h"
#include "Geometry/geometry.h" // provides Geometry::Point_2D


/** \brief The dedicated namespace for the route graph. */
namespace RouteGraph {

using namespace boost;  

//
// Properties
//

///< \brief Vertex descriptor type of a RouteGraph.
using Vertex_Descriptor = adjacency_list_traits<setS, listS, undirectedS>::vertex_descriptor; 


/** \brief The (nested) internal vertex properties of the graph. 
 * \todo The vertex coordinates don't make much sense withouth some sort of 
 * reference, i.e. a datum. (Currently the assumed datum is the one of the UAV
 * utilizing the route graph.
 */
using Vertex_Properties = // (nested) properties
   property<vertex_vroniIndex_t,   int
  ,property<vertex_index_t,        unsigned int  
  ,property<vertex_predecessor_t,  Vertex_Descriptor
  ,property<vertex_position_2D_t,  Geometry::Point_2D
  > > > >;  
  
/** \brief The (nested) internal edge properties of the graph. */  
using Edge_Properties = // (nested) properties
   property<edge_weight_t, double
  ,property<edge_vroniIndex_t, int
  > >;  

//
// The Graph
//

/** \brief The actual RouteGraph graph type.
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
using Vertex_VroniIndex_Map   = property_map<Graph,vertex_vroniIndex_t>::type;
using Vertex_Position2d_Map   = property_map<Graph,vertex_position_2D_t>::type;
using Vertex_Predecessor_Map  = property_map<Graph,vertex_predecessor_t>::type;

using Edge_Weigth_Map         = property_map<Graph,edge_weight_t>::type;
using Edge_VroniIndex_Map     = property_map<Graph,edge_vroniIndex_t>::type;




//
// Property Traits
//

// using Vertex_Index       = property_traits<Vertex_Index_Map>::value_type;
// using Vertex_Position2d  = property_traits<Vertex_Position2d_Map>::value_type;


} // end namespace RouteGraph

#endif // if 0

#endif // ROUTEGRAPH_H
