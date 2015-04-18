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

#ifndef ARENA_H
#define ARENA_H

#include <QtCore/QObject>

#include "Geometry/geometry.h"
#include "Graphics/geometryitem.h"
#include "Nodes/node.h"

namespace Graphics_2D {

  
/** \brief An extension of the Polygon which accepts Node drag-and-drop events. */  
class Arena :public QObject, public Polygon
{
Q_OBJECT

Q_SIGNALS:
 void blipDropped(NodeId /*id*/, QPointF /*scenePos*/);
 void blipMoved( QPointF /*scenePos*/);
 void blipEntered(NodeId /*id*/, QPointF /*scenePos*/);

public:
 Arena(Geometry::Polygon_2D & polygon, QGraphicsItem* parent = nullptr);
 virtual ~Arena();

protected:
// void dragLeaveEvent(QGraphicsSceneDragDropEvent *event);
  virtual void dragMoveEvent(QGraphicsSceneDragDropEvent *event);
  virtual void dropEvent(QGraphicsSceneDragDropEvent *event);
  virtual void dragEnterEvent(QGraphicsSceneDragDropEvent *event);

private:

};

} // end namespace Graphics_2D

#endif // ARENA_H