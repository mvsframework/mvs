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

#ifndef ELEMENT_PRIVATE_H
#define ELEMENT_PRIVATE_H

#include <QtCore/qglobal.h>

#include "config.h"
#include "elementid.h"
#include "Geometry/geometry.h"
#include "Nodes/node_p.h"

#ifdef WITH_GRAPHICS
  #include <QtWidgets/QGraphicsItemGroup>
  #include <QtWidgets/QMenu>
  #include "Graphics/geometryitem.h"
#endif // WITH_GRAPHICS

class Element; // Forward declaration.
class ElementPrivate
{
private:
  Element* const q_ptr;
  Q_DECLARE_PUBLIC(Element)

public:
  ElementPrivate(Element* q);
  virtual ~ElementPrivate();

  ElementId id = NULL_ELEMENT_ID;

  Geometry::Polygon_2D  shape;
  Geometry::Point_2D    shapeCentroid;
  Geometry::Polygon_2D  convexHull;
  
  /** \brief An enumeration collecting assignable tags for Elements. */
  enum Tag {
     None     = 0x0   ///< \brief No special tag. (The default.)
     // Target Group (a tri-state group: hasNoMarker, isTarget, isNotTarget)
    ,HasTargetMarker = 0x2  ///< \brief Indicates wether the following Target Tag is valid
    ,Target         = 0x4   ///< \brief Marks an element as being a target of some Node.
  };
  Q_DECLARE_FLAGS(Tags, Tag)
  Tags tags;
  
#ifdef WITH_GRAPHICS  
  QColor mainColor;
  
  void setColor(QColor const& color);
  
  class PolygonItem : public Graphics_2D::Polygon
  {
  public:
    PolygonItem(ElementPrivate* ep, Geometry::Polygon_2D & geometry, QGraphicsItem* parent = nullptr)
      : Graphics_2D::Polygon(geometry, parent)
      ,d_element(ep)
    {};
    
    virtual ~PolygonItem() = default;
    
  protected:
    virtual void contextMenuEvent(QGraphicsSceneContextMenuEvent* event)
    { d_element->contextMenuEvent(event); };
  private:
    ElementPrivate* d_element = nullptr;
  };
  
  PolygonItem* shapeItem = nullptr;
  PolygonItem* convexHullItem = nullptr;
  
//   NodePrivate* d_node = nullptr;
  
  void contextMenuEvent(QGraphicsSceneContextMenuEvent* event);
 
  /** \brief The menu of actions always available for an Element. */
  QMenu persistentContextMenu;
  
  QAction* showConvexHullAction = nullptr;
  
  /** \brief Update the persistent context menu actions to reflect the current state. */
  void updatePersistentContextMenu();
  
  
#endif // WITH_GRAPHICS
    
};

Q_DECLARE_OPERATORS_FOR_FLAGS(ElementPrivate::Tags)

#endif // ELEMENT_PRIVATE_H
