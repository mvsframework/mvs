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

#ifndef FORMATIONGRAPH_H
#define FORMATIONGRAPH_H

#include "Graph/rfgraph.h"

namespace FormationGraph = RfGraph;


// #include "Graph/locationgraph.h"
// 
// /** \brief The dedicated namespace for the formation graph. */
// namespace FormationGraph {
// 
// using namespace boost;  
//   
// //
// // Properties
// //
//   
// /** \brief The (nested) internal vertex properties of the graph. */
// using Vertex_Properties = // (nested) properties
//   LocationGraph::Vertex_Properties;
//   
//   
// /** \brief The (nested) internal edge properties of the graph. */  
// using Edge_Properties = 
//    property< edge_weight_t, double
//   ,property< edge_quality_t, unsigned int //NOTE: this should match the underlying type of the RfZone enum
//   ,LocationGraph::Edge_Properties
//   > >;
// 
// /** \brief The actual  graph type.
//  * 
//  * \tparam OutEdgeList The selector for the container used to represent the 
//  *  edge-list for each of the vertices. (default: \e vecS)
//  * 
//  * \tparam VertexList The selector for the container used to represent the 
//  *  vertex-list of the graph. (default: \e vecS)
//  * 
//  * \tparam Directed A selector to choose whether the graph is directed, 
//  *  undirected, or directed with bidirectional edge access (access to both 
//  *  out-edges and in-edges). The options are \e directedS, \e undirectedS, and 
//  *  \e bidirectionalS. (default: \e directedS)
//  * 
//  * \tparam VertexProperties for specifying internal property storage. (default
//  *  \e no_property)
//  * 
//  * \tparam EdgeProperties for specifying internal property storage. (default
//  *  \e no_property)
//  * 
//  * \tparam GraphProperties for specifying property storage for the graph object.
//  *  (default: \e no_property)
//  * 
//  * \tparam EdgeList The selector for the container used to represent the 
//  *  edge-list for the graph. (default: \e listS)
//  */  
// using Graph =  adjacency_list<
// //   /* OutEdgeList      */ setS // setS dissallows parallel edges
//   /* OutEdgeList      */ listS // 
//   /* VertexList       */,listS // listS allows for never invalidated vertex descriptors
//   /* Directed         */,undirectedS 
//   /* VertexProperties */,Vertex_Properties
//   /* EdgeProperties   */,Edge_Properties
//   /* GraphProperties  */
//   /* EdgeList         */
//   > ;  
//   
// //
// // Graph Traits
// //
// using Traits = graph_traits<Graph>;
// using Vertex_Descriptor= Traits::vertex_descriptor; ///< \brief Vertex descriptor type of an ElementGraph.
// using Vertex_Iterator  = Traits::vertex_iterator; ///< \brief Vertex iterator type of an ElementGraph.
// using Edge_Descriptor  = Traits::edge_descriptor; ///< \brief Edge descriptor type of an ElementGraph.
// using Edge_Iterator    = Traits::edge_iterator;   ///< \brief Edge descriptor type of an ElementGraph.
//   
// 
// //
// // Property Maps
// //
// 
// using Vertex_Index_Map        = property_map<Graph,vertex_index_t>::type;
// using Vertex_LocationId_Map   = property_map<Graph,vertex_locationId_t>::type;
// 
// using Edge_Index_Map          = property_map<Graph,edge_index_t>::type;
// using Edge_Weight_Map         = property_map<Graph,edge_weight_t>::type;
// using Edge_Quality_Map        = property_map<Graph,edge_quality_t>::type;
//    
// } // end namespace PathGraph

#endif // FORMATIONGRAPH_H
