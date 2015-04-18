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

#ifndef GRAPHGRAPHICS_H
#define GRAPHGRAPHICS_H

#include <functional>

#include <QObject>
#include <QtWidgets/QGraphicsItem>
#include <QtWidgets/QGraphicsItemGroup>
#include <QtWidgets/QGraphicsLineItem>

#include <boost/graph/properties.hpp>
#include <boost/graph/adjacency_list.hpp>

#include "Graph/comgraph.h"

#include "Graphics/palette.h"


namespace Graphics_2D {

 
  
class SimpleVertexItem : public QGraphicsEllipseItem {
public:  
  SimpleVertexItem( qreal x, qreal y, qreal dia = 1, QGraphicsItem* parent = nullptr)
  :QGraphicsEllipseItem(x-dia/2,y-dia/2,dia,dia,parent)
  {};
    
  void setDiameter(qreal dia)
  { setRect(pos().x()-dia/2,pos().y()-dia/2,dia,dia); };
  
  qreal diameter() const
  { return rect().width(); }
};  
  
  
/** \brief Generate a Qt graphics representation of a graph.
 *
 * This function creates a QGraphicsLineItem for each edge, collects them all 
 * in a QGraphicsItemGroup and returns that.
 * 
 * \note The caller assumes ownership of the returned pointer.
 * 
 * \pre The PositionMap's value_type must satisfy boost::geometry::concept::Point.
 * \pre The LineGraphics must provide an interface like QGraphicsLineItem.
 * 
 * \tparam Graph The boost::graph type.
 * \tparam PositionMap The position property map for the graph. 
 * \tparam LineGraphics (optional) The graphics type to instantiate for each 
 *  edge via a LineGraphics(double x1,double y1,double x2,double y2) 
 *  constructor. 
 * \tparam VertexGraphics (optional) The grpahics type to instantiate for each
 *  vertex via a VertexGraphics(double x,double y,double dx, double dy) 
 *  constructor.
 * \tparam addIndexToolTip (optional) TRUE if the graphics should contain
 * a ToolTip showing the edge and vertex indices.
 * 
 * \todo Make the edgePen and vertexPen lambdas property maps.
 * 
 * \todo Find an intelligent way to define the graphics of a graph. 
 * 
 * \param[in] graph The graph to create the graphics for.
 * \param[in] positionMap The position map for the vertices of the graph.
 * \param[in] edgePen A functional mapping an edge_descriptor to the QPen to be 
 *  used for that edge.
 * \param[in] drawEdges (optional, default TRUE) Draw the edges if TRUE.
 * \param[in] drawVertices (optional, default TRUE) Draw all vertices 
 *  independently of their connectivity.
 * \return A pointer to an item group holding the LineGraphics items 
 *  representing the graph.
 */
template< typename Graph,
          typename PositionMap,
          typename Edge = typename boost::graph_traits<Graph>::edge_descriptor,
          typename Vertex = typename boost::graph_traits<Graph>::vertex_descriptor,
          typename LineGraphics=QGraphicsLineItem,
          typename VertexGraphics=SimpleVertexItem,
          bool addIndexToolTip = true>
QGraphicsItemGroup* generateQtGraphicsFromGraph(
  const Graph& graph,
  const PositionMap& positionMap,
  typename std::function<QPen (Edge)> edgePen,
  typename std::function<QPen (Vertex)> vertexPen,
  bool drawEdges = true,
  bool drawVertices = false
)
{
  using namespace boost;
  
  BOOST_CONCEPT_ASSERT(( geometry::concept::Point< typename property_traits<PositionMap>::value_type >  ));
  
  auto group = new QGraphicsItemGroup;
  
  
  if( drawEdges )
  {     
    
    // iteratare over all edges
    typename graph_traits<Graph>::edge_iterator ei, ei_end;
    for( tie(ei,ei_end)=edges(graph); ei!=ei_end; ++ei )
    {
      
      // get the head and tail vertices
      Vertex head = target(*ei, graph);
      Vertex tail = source(*ei, graph);
      
      // get the coordinates of the head and tail node of the edge
      typename property_traits<PositionMap>::value_type headPosition = positionMap[head];
      typename property_traits<PositionMap>::value_type tailPosition = positionMap[tail];
      
      { // create a new line and add it to the group.
        
        auto line = new LineGraphics(tailPosition.x(), tailPosition.y(),
                                                  headPosition.x(), headPosition.y());
        
        { /// change the graphical appearance of the lines making up the edge(s).
          auto qLine = dynamic_cast<QGraphicsLineItem*>(line);
          if( qLine )
          { qLine->setPen(edgePen(*ei)); }
        }
        
        if( addIndexToolTip )
        { /// add indices to the graphics tooltip
          auto index = get(edge_index,graph,*ei);
          line->setToolTip(QObject::tr("Edge %1").arg(index));
        }
        
        group->addToGroup(line);
      }
    }
  }

  if(drawVertices)
  {  

    
    constexpr qreal radius = 2.0;

    // iterate over all vertices
    typename graph_traits<Graph>::vertex_iterator vi, vi_end;
    for( tie(vi,vi_end)=vertices<>(graph); vi!=vi_end; ++vi )
    {
      
      // get the coordinates of the head and tail node of the edge
      typename property_traits<PositionMap>::value_type position = positionMap[*vi];
      auto vertex = new VertexGraphics(position.x(), position.y(),2*radius);
   
      vertex->setPen(vertexPen(*vi));
      
      if( addIndexToolTip )
        { /// add indices to the graphics tooltip
          auto index = get(vertex_index,graph,*vi);
          vertex->setToolTip(QObject::tr("Vertex %1").arg(index));

          //NOTE: not all graphs are location graphs :(
//           auto locId = get(vertex_locationId, graph, *vi);
//           vertex->setToolTip(QObject::tr("Vertex %1 (LocationID: %2)").arg(index).arg(locId));
        }
      
      group->addToGroup(vertex);
    }
  }
  
  
  return group;
}

template< typename Graph,
          typename PositionMap,
          typename LineGraphics=QGraphicsLineItem,
          typename VertexGraphics=SimpleVertexItem,
          bool addIndexToolTip = true> 
QGraphicsItemGroup* generateQtGraphicsFromGraph(
  const Graph& graph,
  const PositionMap& positionMap,
  const QPen edgePen,
  const QPen vertexPen,
  bool drawEdges = true,
  bool drawVertices = false
)
{
  using Edge = typename boost::graph_traits<Graph>::edge_descriptor;
  using Vertex = typename boost::graph_traits<Graph>::vertex_descriptor;
  
  auto ePen = [=](Edge edge)->QPen {return edgePen;}; 
  auto vPen = [=](Vertex vertex)->QPen {return vertexPen;}; 
  return generateQtGraphicsFromGraph<Graph, PositionMap, Edge, Vertex,LineGraphics, VertexGraphics,addIndexToolTip >
    (graph, positionMap, ePen, vPen, drawEdges, drawVertices);
};


/** \brief Generate a Qt graphics representation of a route graph.
 * 
 * This function serves as a support for legacy code in which the position map
 * hasn't been provided expicitley.
 * 
 * \note The caller assumes ownership of the returned pointer!
 * 
 * \param[in] graph The graph to create the graphics for.
 * \return A pointer to an item group holding the LineGraphics items 
 *  representing the graph.
 */
template<typename Graph>
QGraphicsItemGroup* generateQtGraphicsFromGraph(Graph graph)
{
  return generateQtGraphicsFromGraph(graph,
                                     boost::get(boost::vertex_position_2D, graph),
                                     Graphics_2D::Pens::Outline(Qt::black), // edgePen //FIXME: Magic numbers : edge pen color
                                     Graphics_2D::Pens::Outline(Qt::black), // vertexPan //FIXME: Magic numbers: vertex pen color
                                     true
                                    );
};

} // end namespace Graphics_2D

#endif //GRAPHGRAPHICS_H