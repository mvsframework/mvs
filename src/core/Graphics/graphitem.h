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

#ifndef GRAPHITEM_H
#define GRAPHITEM_H

#include <QtWidgets/QGraphicsItemGroup>
#include <QtGui/QPen>
#include <QtGui/QBrush>
#include <QtWidgets/QMenu>
#include <QtWidgets/QGraphicsSceneContextMenuEvent>

#include <boost/graph/properties.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/geometry/geometries/point_xy.hpp>

#include "vertexitem.h"
#include "edgeitem.h"

class Node; // forward declaration

namespace Graphics_2D {

class GraphItem : public QGraphicsItemGroup
{
public:

  GraphItem(Node* node, QGraphicsItem* parent = nullptr)
  :QGraphicsItemGroup(parent)
  ,node(node)
  {
    vertexItemGroup = new QGraphicsItemGroup(this);
    edgeItemGroup   = new QGraphicsItemGroup(this);
  };

//   template< typename Graph, typename PositionMap >
//   GraphItemGroup(Graph & graph, PositionMap & positionMap
//     , QGraphicsItem* parent = nullptr)
//   :GraphItemGroup(parent)
//   {
//     generate(graph, positionMap);
//   }; 
  
  virtual ~GraphItem()
  {
  };
  
  template< typename Graph, typename PositionMap >
  void generate(Graph & graph, PositionMap & positionMap)
  {
    using namespace boost;
    
    BOOST_CONCEPT_ASSERT(( geometry::concept::Point< typename property_traits<PositionMap>::value_type >  ));
    
//     using Edge   = typename graph_traits<Graph>::edge_descriptor;
    using Vertex = typename graph_traits<Graph>::vertex_descriptor;
    
    
    // iteratare over all edges
    typename graph_traits<Graph>::edge_iterator ei, ei_end;
    for( tie(ei,ei_end)=boost::edges(graph); ei!=ei_end; ++ei )
    { 
      // get the head and tail vertices
      Vertex head = target(*ei, graph);
      Vertex tail = source(*ei, graph);
      
      // get the coordinates of the head and tail node of the edge
      typename property_traits<PositionMap>::value_type headPosition = positionMap[head];
      typename property_traits<PositionMap>::value_type tailPosition = positionMap[tail];
      
      // create a new line and add it to the group.
      auto line = new EdgeItem(geometry::get<0>(tailPosition), geometry::get<1>(tailPosition),
                               geometry::get<0>(headPosition), geometry::get<1>(headPosition));
         
      edges()->addToGroup(line);
    }
    
    
    // iterate over all vertices
    typename graph_traits<Graph>::vertex_iterator vi, vi_end;
    for( tie(vi,vi_end)=boost::vertices(graph); vi!=vi_end; ++vi )
    {
      
      // get the coordinates of the head and tail node of the edge
      typename property_traits<PositionMap>::value_type position = positionMap[*vi];
      auto vertex = new VertexItem(geometry::get<0>(position), geometry::get<1>(position),
                                   vertexDiameter );
         
      vertices()->addToGroup(vertex);
    }
    
    
  };
  
  
  QGraphicsItemGroup* edges() { return edgeItemGroup; };
  QGraphicsItemGroup* vertices() { return vertexItemGroup; };
  
  void  setEdgePen(const QPen & pen)
  { 
    edgePen = pen;
    
    QGraphicsItem* childItem;
    foreach(childItem, edges()->childItems())
    { 
      auto edgeItem = static_cast<EdgeItem*>(childItem);
      edgeItem->setPen(pen);
    }
  };

  void  setVertexPen(const QPen & pen)
  { 
    vertexPen = pen;
    
    QGraphicsItem* childItem;
    foreach(childItem, vertices()->childItems())
    { 
      auto vertexItem = static_cast<VertexItem*>(childItem);
      vertexItem->setPen(pen);
    }
  };
  void  setVertexBrush(const QBrush & brush)
  { 
    vertexBrush = brush;
    
    QGraphicsItem* childItem;
    foreach(childItem, vertices()->childItems())
    { 
      auto vertexItem = static_cast<VertexItem*>(childItem);
      vertexItem->setBrush(brush);
    }
  };
  
  void setVertexDiameter(qreal dia)
  { 
    vertexDiameter = dia;
    
    QGraphicsItem* childItem;
    foreach(childItem, vertices()->childItems())
    { 
      auto vertexItem = static_cast<VertexItem*>(childItem);
      vertexItem->setDiameter(dia);
    }
  };
  
protected:
  
  QGraphicsItemGroup* edgeItemGroup;
  QGraphicsItemGroup* vertexItemGroup;
  
  QPen edgePen;
  QPen vertexPen;
  QBrush vertexBrush;
  qreal vertexDiameter = 1.0;
  
  
//   virtual void contextMenuEvent(QGraphicsSceneContextMenuEvent * event)
//   {
//       QMenu menu;
//       node->buildFormationContextMenu(&menu);
//       menu.exec(event->screenPos());
//   };
    
private:
  
  Node* node;
  
};

} // end namespace Graphics_2D

#endif // GRAPHITEM_H