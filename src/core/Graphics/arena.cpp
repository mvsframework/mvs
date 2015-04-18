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


#include "Graphics/arena.h"
#include <QtWidgets/QGraphicsSceneDragDropEvent>
#include <QtCore/QMimeData>

// namespace Graphics_2D {
  
Graphics_2D::Arena::Arena(Geometry::Polygon_2D& polygon, QGraphicsItem* parent)
  :QObject()
  ,GeometryItem< Geometry::Polygon_2D >(polygon,parent)
{
  setAcceptDrops(true); // accept Drag-n-Drop events.
}

Graphics_2D::Arena::~Arena()
{

}


void Graphics_2D::Arena::dropEvent(QGraphicsSceneDragDropEvent* event)
{
  QGraphicsItem::dropEvent(event);
  
  QMimeData const* mime = event->mimeData();
  
  NodeId id(mime->data("NodeId"));
  
  emit blipDropped( id , event->scenePos());
}

void Graphics_2D::Arena::dragMoveEvent(QGraphicsSceneDragDropEvent* event)
{
  QGraphicsItem::dragMoveEvent(event);
  
  //NOTE: I decided to skip transmitting the NodeId as the move event 
  // happens fairly often and the creation of the ID could slow things down.
  // Furthermore, the moved Blip is alrady identified as BlipItem calls 
  // ControlStation::setDraggedBlip() before exectuing the drag.
  
  emit blipMoved(event->scenePos());
}


void Graphics_2D::Arena::dragEnterEvent(QGraphicsSceneDragDropEvent *event)
{
  if( event->mimeData()->hasFormat("NodeId") )
  {
    QByteArray byteArray = event->mimeData()->data("NodeId");
    NodeId id(byteArray);
    emit blipEntered(id, event->scenePos());
  }
  else
  { event->ignore(); }
}
  
  
// } // end namespace Graphics_2D


#include "arena.moc"