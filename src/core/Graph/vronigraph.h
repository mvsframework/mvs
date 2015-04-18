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

#ifndef VRONIGRAPH_H
#define VRONIGRAPH_H

#include <boost/graph/adjacency_list.hpp>
#include <boost/tuple/tuple.hpp> // provides tie(a,b)

#include "Graph/properties.h"
#include "Geometry/geometry.h" // provides Geometry::Point_2D

#include "vronidefines.h"  
#include "defs.h" //NOTE: it is necessary to include "vronidefines.h" before "defs.h" to get the correct #defines!

/** \brief The dedicated namespace for a VRONI generated graph. */
namespace VroniGraph {
  
using namespace boost;  

//
// Properties
//

///< \brief Vertex descriptor type of a VroniGraph.
using Vertex_Descriptor = adjacency_list_traits<setS, listS, undirectedS>::vertex_descriptor; 


/** \brief The (nested) internal vertex properties of the graph. 
 * \todo The vertex coordinates don't make much sense withouth some sort of 
 * reference, i.e. a datum. (Currently the assumed datum is the one of the UAV
 * utilizing the route graph.
 */
using Vertex_Properties = // (nested) properties
   property<vertex_index_t,         unsigned int  
  ,property<vertex_vroniIndex_t,    int 
  ,property<vertex_predecessor_t,   Vertex_Descriptor
  ,property<vertex_position_2D_t,   Geometry::Point_2D
  ,property<vertex_vroniNodeData_t, VRONI::node
  > > > > >;  
  
/** \brief The (nested) internal edge properties of the graph. */  
using Edge_Properties = // (nested) properties
   property<edge_index_t,           unsigned int  
  ,property<edge_weight_t,          double
  ,property<edge_vroniIndex_t,      int
  ,property<edge_vroniEdgeData_t,   VRONI::edge
  > > > >;  

//
// The Graph
//

/** \brief The actual VroniGraph graph type.
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
// Traits
//  

//NOTE: Vertex_Descriptor is alreadey declare above
  
using Vertex_Iterator  = graph_traits<Graph>::vertex_iterator;  ///< \brief Vertex iterator type of a VroniGraph.
using Edge_Descriptor  = graph_traits<Graph>::edge_descriptor;  ///< \brief Edge descriptor type of a VroniGraph.
using Edge_Iterator    = graph_traits<Graph>::edge_iterator;    ///< \brief Edge descriptor type of a VroniGraph.

  
//
// Property Maps
// 
using Vertex_Index_Map          = property_map<Graph,vertex_index_t>::type;
using Vertex_Position2d_Map     = property_map<Graph,vertex_position_2D_t>::type;
using Vertex_VroniNodeData_Map  = property_map<Graph,vertex_vroniNodeData_t>::type;  
using Vertex_VroniIndex_Map     = property_map<Graph,vertex_vroniIndex_t>::type;
using Edge_VroniEdgeData_Map    = property_map<Graph,edge_vroniEdgeData_t>::type;  
using Edge_Weight_Map           = property_map<Graph,edge_weight_t>::type;
using Edge_VroniIndex_Map       = property_map<Graph,edge_vroniIndex_t>::type;  
}

#endif // VRONIGRAPH_H