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


#ifndef GRAPHSUPPORT_H
#define GRAPHSUPPORT_H

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/connected_components.hpp> // get the connected_components algorithm
#include <boost/graph/graph_traits.hpp> // gets null_vertex()
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/hawick_circuits.hpp>

#include <QtCore/QDebug>

#include <vector>
#include <list>
#include <functional>
#include <type_traits>

#include "Graph/properties.h"

namespace Graph {
  using namespace boost;
  
  /** \todo Introduce a method to create the inverse of a property map. E.g.
   * given a vertex index map, create a hash that maps the index to the 
   * associated vertex descriptor, i.e. create a reverse lookup map.
   */
  
/** \brief Compute the connected components of the COM graph.
 * A graph lookig like this
 * \code A<-->B    C    D<-->E<-->F \endcode
 * (i.e. 6 vertices, a total of 3 connected subgraphs) would return a vector 
 * like this
 * \code 1    1    2    3    3    3 \endcode
 * (indicating that the first two vertices are in one, the first detected, 
 * component, the third vertex in the seconde componentn, etc.)
 * 
 * \param[in] graph A reference to the graph to map.
 * \return A map associating each vertex with a component (connected subgraph)
 *  of the graph.
 */
template<typename Graph_t>  
inline std::vector<int> components(const Graph_t& graph)
{
  using namespace boost;
  
  // Create an "external" container to hold the component id (an int) for 
  // each vertex. The value type shouch be an integer type, preferably the 
  // same as the vertices_size_type of the graph. The key type must be the 
  // graph's vertex descriptor type.
  // NOTE: Using a simple std::vector<> works automagically _if_ the 
  // vertex_descriptor for the graph internal vertex list is derived from the 
  // vecS type, i.e. the offsets are comparable.
  // NOTE: Because we're using vecS for the graph type, we're
  // effectively using identity_property_map for a vertex index map.
  // If we were to use listS instead, the index map would need to be
  // explicity passed to the component_index constructor.
  std::vector<int> component(num_vertices(graph)); //FIXME: this only works with vecS vertex list graphs (I think)!
  

  { // Concept Checking for make_iterator_property_map(Iterator, Offset) 
    /** \todo Find a concept check that checks for the presence of 
     * graph_traits\<Graph_t\>::vertex_descriptor 
     */
    BOOST_CONCEPT_ASSERT(( RandomAccessIteratorConcept<decltype( component.begin() )> ));
    BOOST_CONCEPT_ASSERT(( ReadablePropertyMapConcept< decltype( get(vertex_index, graph) )
                                                      ,typename graph_traits<Graph_t>::vertex_descriptor > ));
  }
  
  auto component_map = make_iterator_property_map(
    component.begin(),
    get(vertex_index, graph)
  );
  
  
  { // Concept Checking for boost::connected_components(g,c)
    // const Graph& g needs to model: Vertex List Graph, Incidence Graph
    BOOST_CONCEPT_ASSERT(( VertexListGraphConcept< Graph_t > ));
    BOOST_CONCEPT_ASSERT(( IncidenceGraphConcept< Graph_t > ));
    // ComponentMap c needs to model: Writable Property Map
    BOOST_CONCEPT_ASSERT(( WritablePropertyMapConcept< decltype(component_map)
                                                      ,typename graph_traits<Graph_t>::vertex_descriptor > ));
  }
  
  //NOTE: The time complexity for the connected components algorithm is 
  //  O(num_vertices()+num_edges()).
  connected_components( graph, component_map );
  
  return component;
};

/** \brief Get the subgraph of all cyclic branches.
 * 
 * This function copies the original graph, and then iterativiley cuts away 
 * non-cyclic branches:
 * 
 *  * Get all vertices of degree one
 *  * if more than zero are found, for each of those vertices:
 *  ** traverse the graph till the first vertex of a higher degree is found
 *  ** delete all visited vertices
 *  * Repeat unitl no degree-one vertices are found
 * 
 * \note This function also resets the vertes_index of all vertices to maintain
 * a consistent [0,num_vertices()] range!
 * 
 * \tparam Graph_t The graph type.
 * \param[in] graph The original, potentially non-cyclic, graph.
 * \return The subgraph of all cyclic branches in the original graph.
 */
template<typename Graph_t>
Graph_t pruneNonCyclicBranches(Graph_t graph)
{
  using namespace boost;
  
  Graph_t subGraph;
  subGraph = graph;
//   print_graph(graph,get(vertex_index,graph));
//   print_graph(subGraph,get(vertex_index,subGraph));
  
  bool removedVertex = true;
  
  int removedVertices = 0;
  
  int removingLoops = 0;
  
  typedef typename property_map<Graph_t,vertex_index_t>::type VertexIndexMap_t;
  
  
  // iterate over all vertices
  typename graph_traits<Graph_t>::vertex_iterator vi, vi_end, next;
  while( removedVertex ) {
    removedVertex = false;
    ++removingLoops;
    int multiDegreeVertices = 0;
    
    VertexIndexMap_t vIndexMap = get(vertex_index,subGraph);
    typename property_traits<VertexIndexMap_t>::value_type newIndex = 0;
    
    tie(vi,vi_end)=vertices(subGraph);
    
    //NOTE: the "unusual" for-loop signatur is neccessary as remove_vertex()
    //      invalidates the iterator it removes. Hence the "standard" ++vi
    //      as a third argument wouldn't work anymore.
    for(next = vi; vi!=vi_end; vi = next )
    { 
      ++next;
      int degree = out_degree(*vi,subGraph)/*+in_degree(*vi,subGraph)*/;
//       qDebug("Vertex %d has a degree of %d.",vIndexMap(*vi),degree);
      if( degree == 1)
      {
        clear_vertex(*vi,subGraph); // remove all edges (well, one) from this vertex
        remove_vertex(*vi, subGraph);
        removedVertex = true;
        ++removedVertices;
      }
      else
      { 
        ++multiDegreeVertices;
        vIndexMap(*vi) = newIndex++;  // correct the vertex index maps
      }
    }
//     qDebug() << "Found" << multiDegreeVertices << "vertices with more than one neighbor.";
  }
//   qDebug("Removed %d vertices in %d loops.",removedVertices, removingLoops);
//   qDebug() << "The resulting graph has" << boost::num_vertices(subGraph) 
//     << "vertices and" << boost::num_edges(subGraph) << "edges.";
    
  return subGraph;
}

/** \brief Find the Vertex closest ot the given point.
 * 
 * This function iterates over the complete Graph graph, comparting the position
 * of the vertices to the given point. The vertex descriptor of the closest
 * vertex is returned.
 * 
 * \internal The position map can be gotten with 
 *  property_map<Graph, vertex_position_2D_t>::type positionMap = get(vertex_position_2D, graph);
 * 
 * \tparam Graph The graph type.
 * \tparam PostionMap The position map type providing the location of the 
 *  vertices.
 * 
 * \param[in] graph The graph whose vertices are to be checked.
 * \param[in] positionMap The property map providing the position of the 
 *  vertices of graph.
 * \param[in] point The location to which the closes vertex is sought-after.
 */
template< typename Graph,
          typename PositionPropertyMap
        >
typename boost::graph_traits<Graph>::vertex_descriptor getNearestVertex(
  Graph& graph,
  const PositionPropertyMap& positionMap,
  const typename boost::property_traits< PositionPropertyMap >::value_type& point)
{
  using namespace boost;
  
  // Vertex descriptor type of a Graph
  using Vertex_t    = typename graph_traits<Graph>::vertex_descriptor; ///< \brief Vertex descriptor type of a Graph.
  Vertex_t closestVertex = graph_traits<Graph>::null_vertex();
  
  using VertexPosition_t  = typename property_traits<PositionPropertyMap>::value_type; ///< \brief Vertex position type of the position property map.
  VertexPosition_t  closestVertexPos;
    
  using Vertex_it_t = typename graph_traits<Graph>::vertex_iterator;   ///< \brief Vertex iterator type of a Graph.
  Vertex_it_t vi, vi_end;
  
  double minFoundDistance = std::numeric_limits<double>::infinity();
  // iterate over all vertices
  for( tie(vi,vi_end) = vertices(graph); vi != vi_end; ++vi )
  {
    auto vertexPos = get(positionMap,*vi);
    
    double distance = geometry::distance(point,vertexPos);
    
    if( distance < minFoundDistance )
    {
      closestVertex = *vi;
      closestVertexPos = vertexPos;
      minFoundDistance = distance;
    }
  }
  return closestVertex;
};

/** \brief Update edge weight from properties of the edge's head and tail.
 * This is mainly used to update edge weight maps for use with the Dijkstra
 * algorithm.
 * 
 * \internal from http://www.boost.org/doc/libs/1_52_0/libs/graph/doc/dijkstra_shortest_paths.html :
 * "The type WeightMap must be a model of Readable Property Map. The edge 
 * descriptor type of the graph needs to be usable as the key type for the 
 * weight map. The value type for this map must be the same as the value type 
 * of the distance map."
 * 
 * \tparam Graph_t The type of the underlying graph.
 * \tparam VertexPositionMap_t The property used to compute the edge weight
 * \tparam EdgeWeightMap_t The property map type for the edge weight
 * 
 * \param[in] graph The underlying graph.
 * \param[in] vertexPositions The map providing the vertex position date used 
 *  to compute the edge weight
 * \param[out] edgeWeight The map holding the computed edge weights (lengths).

 */
template<typename Graph_t
        ,typename VertexPositionMap_t = typename boost::property_map<Graph_t,boost::vertex_position_2D_t>::type
        ,typename EdgeWeightMap_t = typename boost::property_map<Graph_t,boost::edge_weight_t>::type
        >
void updateEdgeWeightMap( Graph_t graph
                         ,VertexPositionMap_t vertexPositions
                         ,EdgeWeightMap_t edgeWeights )
{
  using namespace boost;
  
  // get edge iterators for the first and (one bejond) last edge of the graph
  typename graph_traits<Graph_t>::edge_iterator ei, ei_end;
  tie(ei,ei_end) = edges(graph);
  
  // iterate over all edges
  for( /*empty*/; ei != ei_end; ++ei)
  {
    auto sourceVertex = source(*ei, graph); // the edge starts here...
    auto targetVertex = target(*ei, graph); // ... and ends here.
    
    auto sourcePosition = vertexPositions[sourceVertex];
    auto targetPosition = vertexPositions[targetVertex];
    
    edgeWeights[*ei] = boost::distance(sourcePosition,targetPosition);            
  }
  
  return;
}


/** \brief Remove all vertices of degree 2 from a graph.
 * \deprecated {This function isn't really generic enough to be of much use.}
 * 
 * \todo Implement some sort of visitor that does something with the properties
 * of the newly added and deleted edges (like copying/merging them over) (See 
 * the nast HACK tags down in the code...)
 * 
 * \tparam Graph The graph type.
 * \param[in,out] g The graph to be altered.
 */
template< typename Graph
//         , typename EdgeDescriptor = typename boost::graph_traits<Graph>::edge_descriptor
//         , typename EdgeMerger = typename std::function<void(EdgeDescriptor, EdgeDescriptor, EdgeDescriptor)>
        > 
// void removeDegree2Vertices( Graph& g, EdgeMerger edge_merger )
void removeDegree2Vertices(Graph& g)
{
  using namespace boost;
  
  BOOST_CONCEPT_ASSERT(( MutableGraphConcept< Graph > ));
  
  using VertexIterator =  typename graph_traits<Graph>::vertex_iterator;
  using AdjacencyIterator = typename graph_traits<Graph>::adjacency_iterator;    
  
  using OutEdgeIterator = typename graph_traits<Graph>::out_edge_iterator;
  
  VertexIterator vi, vi_end, next;
  tie(vi,vi_end)=vertices(g);
  for( next=vi; vi!=vi_end; vi=next )
  {
    ++next;
    if( out_degree(*vi,g) == 2 )
    { 
      AdjacencyIterator ai,ai_end;
      tie(ai,ai_end) = adjacent_vertices(*vi,g);
      // add_edge(*ai,*(++ai),g); //NOTE: THIS DOES NOT WORK!!! add_edge() invalidates adjacency iterators...
      typename graph_traits<Graph>::vertex_descriptor vBefore, vAfter;
      vBefore = *ai;
      vAfter = *(++ai);
      
      typename graph_traits<Graph>::edge_descriptor eBefore, eAfter, eMerged;
      eMerged = add_edge(vBefore, vAfter, g).first;
      {
        // getting the outedges
        typename graph_traits<Graph>::out_edge_iterator ei,ei_end;
        tie(ei,ei_end) = out_edges(*vi,g);
        eBefore = *ei;
        eAfter = *(++ei);
        
        // doing the merging..
        
        put(edge_index,   g, eMerged, get(edge_index, g, eBefore)); //HACK: This is done so that the edge index to face index map still contains valid data.
        put(edge_weight,  g, eMerged, get(edge_weight, g, eBefore)); //HACK: This is done to keep an accumulated edge weight(length) for path graphs.  
      }
       
      OutEdgeIterator ei, ei_end;
      
      clear_vertex(*vi,g);
      remove_vertex(*vi,g);
    }
  }
};



/** \brief Compute the boundary vertex set of a given collection of vertices.
 * \details The boundary vertices are the immediate adjacent vertices that 
 * are \e not part of the initial set. (a,b) + boundary( (a,b) ) = [a,b] 
 * \tparam Graph The boost graph type.
 * \tparam StlContainer A STL container type holding the vertices.
 * \param[in] g The Graph to select the neighbors from.
 * \param[in] vertices The vertices to define the closure from.
 * \return The set of boundary vertices.
 */
template< typename Graph
        , typename StlContainer
        >
std::set< typename boost::graph_traits<Graph>::vertex_descriptor> boundaryVertices(Graph const& g, StlContainer const& vertices)
{
  using namespace boost;
  
  using Vertex = typename boost::graph_traits<Graph>::vertex_descriptor;

  BOOST_ASSERT(( std::is_same<Vertex, typename StlContainer::value_type>::value ));
  
  
  using Iterator = typename StlContainer::const_iterator;
  
  
  std::set<Vertex> boundary;
  typename graph_traits<Graph>::adjacency_iterator ai,ai_end;
  
  Iterator const vi_end = vertices.end();
  Iterator vi = vertices.begin();
  for( /*empty*/; vi!=vi_end; ++vi)
  {
    for( tie(ai, ai_end)=adjacent_vertices(*vi,g); ai!=ai_end; ++ai )
    { boundary.insert(*ai); }
  }
  
  // as the spanning vertices are neighbors of some other spanning vertices,
  // they need to be removed from the returned boundary set. (the closure would 
  // include them)
  vi = vertices.begin();
  for( /*empty*/; vi!=vi_end; ++vi)
  { boundary.erase(*vi); }
    
    
  return boundary;  
};





/** \brief Get the closure set of the provided collection of vertices.
 * \details The closuure set is the combination of the boundary with the initial
 * collection of vertices.
 * This method adds all neighbors of each of the input vertices to the 
 * returned output set as well as the originally provided ones.
 * \tparam Graph The boost graph type.
 * \tparam StlContainer A STL container type holding the vertices.
 * \param[in] g The Graph to select the neighbors from.
 * \param[in] vertices The vertices to define the closure from.
 * \return The closure set of vertices.
 */
template< typename Graph
        , typename StlContainer
        >
std::set<typename StlContainer::value_type> closureVertices(Graph const& g, StlContainer const& vertices)
{
  using Vertex = typename StlContainer::value_type;
  using Iterator = typename StlContainer::const_iterator;
  
  std::set<Vertex> closure;
  
  typename graph_traits<Graph>::adjacency_iterator ai,ai_end;
  
  Iterator const vi_end = vertices.end();
  Iterator vi = vertices.begin();
  for( /*empty*/; vi!=vi_end; ++vi)
  {
    for( tie(ai, ai_end)=adjacent_vertices(*vi,g); ai!=ai_end; ++ai )
    { closure.insert(*ai); }
  }
  
  // As the spanning vertices are neighbors of some other spanning vertices,
  // some might already be in the returned closure set. But in case there are
  // spanning vertices that are not connected to any other spanning vertices, 
  // all are explicitely (re-)added here
  vi = vertices.begin();
  for( /*empty*/; vi!=vi_end; ++vi)
  { closure.insert(*vi); }
  
  return closure;
}

/** \brief Compute the midpoint of an edge from the assoc. vertex locations.
 * \details Use the provided position map get the position of the source and 
 * target vertices of the edge and return the mipoint inbetween those two.
 * \todo Make it dimension agnostic, currently it is harcoded to 2D.
 * \tparam Graph A boost graph type.
 * \tparam PositionMap A property map that holds the Position of the vertices of 
 * the Graph
 * \param[in] e The Edge in to compute the midpoint for
 * \param[in] g The Graph the Edge belongs to.
 * \param[in] pMap The Position property map for Graph vertices.
 * \return The midpoint of the Edge e.
 */
template< typename Graph 
        , typename PositionMap
        , typename Edge = typename boost::graph_traits<Graph>::edge_descriptor
        , typename Position = typename boost::property_traits<PositionMap>::value_type
        >
Position edgeMidpoint(Edge e, Graph g, PositionMap pMap)
{
  using namespace boost;
  using Vertex = typename boost::graph_traits<Graph>::vertex_descriptor;
  
  BOOST_CONCEPT_ASSERT(( GraphConcept<Graph> ));
  BOOST_CONCEPT_ASSERT(( ReadablePropertyMapConcept<PositionMap, Vertex> ));
  BOOST_CONCEPT_ASSERT(( geometry::concept::Point< Position >  ));
  
  Position sourcePos = pMap[source(e,g)];
  Position targetPos = pMap[target(e,g)];
  
  Position midpoint;
  geometry::set<0>(midpoint, (geometry::get<0>(sourcePos) + geometry::get<0>(targetPos))/2.0 );
  geometry::set<1>(midpoint, (geometry::get<1>(sourcePos) + geometry::get<1>(targetPos))/2.0 );

  return midpoint;
}; 

} // end namespace Graph


#endif // GRAPHSUPPORT_H