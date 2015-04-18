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

#ifndef FACEGEOMETRY_VISITOR_H
#define FACEGEOMETRY_VISITOR_H

#include <boost/config.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/property_map/property_map.hpp>
#include <boost/graph/planar_face_traversal.hpp>

#include "Graph/properties.h"
#include "Geometry/geometry.h"

#include <QtCore/QtDebug>


namespace Graph {

/** \brief
 * \tparam Graph The traversed graph
 * \tparam VertexPositionMap The position map for the vertices of the Graph
 * \tparam BiGraph The bifurcation graph
 * \tparam FaceGraph The (face) dual of the Graph
 */                 
template< typename Graph
        , typename VertexPositionMap
        , typename BiGraph
        , typename FaceGraph
        >
class FaceGeometry_visitor : public boost::planar_face_traversal_visitor
{
  
public:
  using Point = typename boost::property_traits<VertexPositionMap>::value_type;
  
  using Vertex = typename boost::graph_traits<Graph>::vertex_descriptor; ///< \brief Vertex descriptor of the traversed Graph.
  using Edge = typename boost::graph_traits<Graph>::edge_descriptor; ///< \brief Edge descriptor of the traversed Graph.

  using BiVertex = typename boost::graph_traits<BiGraph>::vertex_descriptor; ///< \brief Vertex descriptor of the BiGraph.
  using BiEdge = typename boost::graph_traits<BiGraph>::edge_descriptor; ///< \brief Edge descriptor of the BiGraph.
  
  using FaceVertex = typename boost::graph_traits<FaceGraph>::vertex_descriptor; ///< \brief Vertex descriptor of the FaceGraph.
  using FaceEdge = typename boost::graph_traits<FaceGraph>::edge_descriptor; ///< \brief Edge descriptor of the FaceGraph.
  
  using FaceIndexMap = typename std::map<FaceVertex, unsigned int>;
  using FaceGeometry = Geometry::Polygon_2D::ring_type;

  using Edge_To_Face_Map = typename std::map<Edge, std::pair<FaceVertex,FaceVertex>>;  
  using Vertex_To_BiVertex_Map = typename std::map<Vertex, BiVertex>;
  
  FaceGeometry_visitor(Graph const& graph,VertexPositionMap const& vertexPositionMap )
    :graph(graph)
    ,vPos(vertexPositionMap)
  {
    using namespace boost;
    
    // concept check the chosen geometries (ring and point) to store the face
    // geometries
    BOOST_CONCEPT_ASSERT(( geometry::concept::Ring< FaceGeometry > ));
    
    // check that the provided position map can be used as a Point
    BOOST_CONCEPT_ASSERT(( geometry::concept::Point< Point >  ));
    
    BOOST_CONCEPT_ASSERT(( MutableGraphConcept< BiGraph > ));
    
  };
  
  virtual void begin_traversal()
  {
    faceIndex = 0;
    faceGraph.clear();
    biIndex = 0;
    biGraph.clear();
    
    firstBiVertex = boost::graph_traits<BiGraph>::null_vertex();
    lastBiVertex = boost::graph_traits<BiGraph>::null_vertex();
  };
  virtual void begin_face()
  {     
    currentFaceVertex = boost::add_vertex(faceGraph);
//     boost::put(boost::vertex_index, faceGraph, currentFaceVertex, faceIndex);
    boost::put(boost::vertex_index, faceGraph, currentFaceVertex, faceIndex++);
    currentFaceGeometry.clear();
    
    firstBiVertex = boost::graph_traits<BiGraph>::null_vertex();
  };
  virtual void next_vertex(Vertex const& vertex)
  {
    using namespace boost;
    
    geometry::append(currentFaceGeometry,vPos[vertex]);
    
    if( out_degree(vertex,graph) > 2 )
    {
      BiVertex currentBiVertex;
      if( true or vertexToBiVertexMap.count(vertex) == 0 )
      {
        currentBiVertex = add_vertex(biGraph);
        put(vertex_position_2D, biGraph, currentBiVertex, vPos[vertex]);
        put(vertex_index, biGraph, currentBiVertex, biIndex++);
        
        vertexToBiVertexMap[vertex] = currentBiVertex;
      }
      else
      { currentBiVertex = vertexToBiVertexMap[vertex]; }
      
      if( firstBiVertex == boost::graph_traits<BiGraph>::null_vertex() )
      { firstBiVertex = currentBiVertex; }
      else
      { boost::add_edge(currentBiVertex, lastBiVertex, biGraph); }
      
      lastBiVertex = currentBiVertex;
    }  
    
  };
  virtual void next_edge(Edge const& edge)
  {
    if( edgeToFaceMap.count( edge ) == 0 )
    { 
      edgeToFaceMap[edge] = 
        std::pair<FaceVertex,FaceVertex>
          (currentFaceVertex,
           boost::graph_traits<FaceGraph>::null_vertex()
          );
    }
    else if( edgeToFaceMap.count( edge ) == 1 )
    { 
      bool printEdgeMap = false;
      if( edgeToFaceMap[edge].second != boost::graph_traits<FaceGraph>::null_vertex() )
      { 
        printEdgeMap = true;
        qCritical() << "Trying to reassign the second face of the edge with index"
         << boost::get(boost::edge_index, graph, edge);
      }
      else
      {
        boost::add_edge(edgeToFaceMap[edge].first, currentFaceVertex, faceGraph);
        edgeToFaceMap[edge].second = currentFaceVertex; 
      }
      
      //DEBUG
      //NOTE: this is running on the VRONI GRAPH, the one that still has all 
      // the branches into the convex parts of buildings!
      if( printEdgeMap or (edgeToFaceMap[edge].first == edgeToFaceMap[edge].second) )
      {
        using namespace boost;
        
//         qDebug() << "edgeToFaceMap[" << get(edge_index, graph, edge) << "] = {"
//           << get(vertex_index, faceGraph, edgeToFaceMap[edge].first)
//           << ","
//           << get(vertex_index, faceGraph, edgeToFaceMap[edge].second)
//           << "}";

      }
      
    }
    else
    {
      qCritical() << "Visiting an edge a THIRD time.";
    }
  };
  
  virtual void end_face()
  {
    using namespace boost;
    geometry::correct(currentFaceGeometry);
    Point faceCentroid;
    geometry::centroid(currentFaceGeometry,faceCentroid);
    
    faceGeometries.push_back(currentFaceGeometry);
    facePositions.push_back(faceCentroid);
    put(vertex_position_2D, faceGraph, currentFaceVertex, faceCentroid);        
    
    // close the last open edge in the biGraph when the face is done.
    add_edge(lastBiVertex, firstBiVertex, biGraph); 
//     ++faceIndex;
  };
  virtual void end_traversal()
  {};
  
  
public:
  
  BiGraph biGraph; ///< \brief The created bifurcated graph.
  FaceGraph faceGraph; ///< \brief The (face) dual of the Graph.
  
  std::vector<FaceGeometry> faceGeometries; ///< \brief Container holding the face geometries, offset by the face index.
  std::vector<Point> facePositions; ///< \brief Container holding the face centroid positions, offset by the face index.
  
  Vertex_To_BiVertex_Map vertexToBiVertexMap;
  
  /** \brief Mapping an edge to the vertices representing the left and right 
   * face. */
  Edge_To_Face_Map edgeToFaceMap;
  
protected:
  Graph const& graph; ///< \brief The visited Graph.
  VertexPositionMap const& vPos; ///< \brief The position map of the Graph vertices.

private:
  unsigned int faceIndex = 0;
  unsigned int biIndex = 0;
  
  FaceVertex currentFaceVertex;
  FaceGeometry currentFaceGeometry;
  
  BiVertex firstBiVertex;
  BiVertex lastBiVertex;
  
  
};
  
} // end namespace Graph

#endif // FACEGEOMETRY_VISITOR_H