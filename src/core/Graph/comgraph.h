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

#ifndef COMGRAPH_H
#define COMGRAPH_H

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/subgraph.hpp>
#include <boost/tuple/tuple.hpp> // provides tie(a,b)

#include "Graph/properties.h"
#include "Geometry/geometry.h" // provides Geometry::Point_2D
#include "nodeid.h"


/** \brief The dedicated namespace for the COM graph. 
 * \todo Make the COM graph some sort of dynamic R/F graph.
 */
namespace ComGraph {

using namespace boost;  

//
// Properties
//
/** \brief The (nested) internal vertex properties of the graph. */
using VertexProperties_t = // (nested) properties
   property<vertex_index_t, std::size_t,  //NOTE: remember, this doesn't get automagically filled in! (well, vecS does it IF the value is std::size_t)
   property<vertex_name_t, std::string,
   property<vertex_id_t, NodeId,
   property<vertex_position_2D_t,Geometry::Point_2D
   > > > >
 ; 

// using VertexProperties_t = no_property;
  
/** \brief The (nested) internal edge properties of the graph. */  
using EdgeProperties_t = // (nested) properties
   property<edge_index_t, unsigned int 
  ,property<edge_weight_t,  double
  ,property<edge_quality_t, unsigned int
  > > >;  
 

//
// The Graph
//

/** \brief The actual type of a ComGraph.
 * \note It is important that this graph type has a working \c vertex_index 
 *  property, as that is (seems to be) needed by Graph::components()!
 * 
 * The current way to solve the vertex_index issue is to use \c vecS as the 
 * vertex list type (which automatically provides the "index" of std::vector<>,
 * i.e.std::size_t, as an \c vertex_index, \e if the property is actually there,
 * i.e. it needs to be in the VertexProperties_t !
 */  
using Graph_t =  adjacency_list<
   setS // store out-edges as a std::set. This dissallows parallel edges!
//   ,listS // store the vertex set in a std::list (which does _NOT_ provide an automatic vertex_index !!)
  ,vecS // store the vertex set in a std::vec. //NOTE: using std::vec also ensures the _automatic_ availability of the vertex_index property! (see: http://stackoverflow.com/questions/2411017/using-custom-and-built-in-properties-in-boostgraph)
  ,undirectedS // the graph is undirected
  ,VertexProperties_t
  ,EdgeProperties_t
  > ;

//
// Graph Traits
//
using Vertex_Descriptor_t = graph_traits<Graph_t>::vertex_descriptor; ///< \brief Vertex descriptor type of a ComGraph
using Vertex_it_t = graph_traits<Graph_t>::vertex_iterator;   ///< \brief Vertex iterator type of a ComGraph
using Edge_t      = graph_traits<Graph_t>::edge_descriptor;   ///< \brief Edge descriptor type of a ComGraph
using Edge_Descriptor_t      = graph_traits<Graph_t>::edge_descriptor;   ///< \brief Edge descriptor type of a ComGraph

//
// Property Maps
//
// using VertexIndexMap_t    = property_map<Graph_t,vertex_index_t>::type;
// using VertexPositionMap_t = property_map<Graph_t,vertex_position_2D_t>::type;
using VertexNameMap_t = property_map<Graph_t,vertex_name_t>::type;
using VertexIdMap_t   = property_map<Graph_t,vertex_id_t>::type;
using VertexPosition2dMap_t = property_map<Graph_t,vertex_position_2D_t>::type;
//
// Property Traits
//
// using VertexIndex_t     = property_traits<VertexIndexMap_t>::value_type;
// using VertexPosition_t  = property_traits<VertexPositionMap_t>::value_type;


} // end namespace ComGraph

#endif // COMGRAPH_H
