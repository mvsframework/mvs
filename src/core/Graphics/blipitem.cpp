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

#include "blipitem.h"

#include "Graphics/palette.h"

#include <QtWidgets/QGraphicsSceneEvent>
#include <QtWidgets/QApplication>
#include <QtWidgets/QMenu>
#include <QtGui/QDrag>
#include <QtGui/QBitmap>
#include <QtCore/QMimeData>
#include <QtCore/QDebug>

#include "Nodes/controlstation.h"


namespace Graphics_2D {

BlipItem::BlipItem(Blip* blip, QGraphicsItem* parent)
  :QAbstractGraphicsShapeItem(parent)
  ,q(blip)
{
  setBrush(Qt::NoBrush);
  setPen(Graphics_2D::Pens::Outline(Qt::black));
  
  setToolTip(q->objectName());
  setCursor(Qt::ArrowCursor);
  setAcceptedMouseButtons(Qt::LeftButton);
}

BlipItem::~BlipItem()
{

}

QRectF BlipItem::boundingRect() const
{
  QPainterPath shape = generateGustStyleMarker();
  QRectF box = shape.boundingRect();
  
  qreal dp = pen().widthF()/2;

  return box.adjusted(-dp,-dp,dp,dp);
}


void BlipItem::paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget)
{
  auto shape = generateGustStyleMarker();
  
  painter->setPen(pen());
  
  if( q->isConnectedToHost() )
  {
    painter->setBrush(brush());
  }
  else
  {
    QColor c = brush().color();
    c.setAlpha(200); // FIXME: Magic Numbers : opaqueness
    QBrush b = brush();
    b.setColor(c);
    
    painter->setBrush(b);
  }
  
  painter->drawPath(shape);
}




void BlipItem::mouseMoveEvent(QGraphicsSceneMouseEvent* event)
{
  if (QLineF(event->screenPos(), event->buttonDownScreenPos(Qt::LeftButton))
      .length() < QApplication::startDragDistance()) {
      return;
  }

  QDrag *drag = new QDrag(event->widget());
  QMimeData *mime = new QMimeData;
  drag->setMimeData(mime);
  
  mime->setData("NodeId",q->id().toByteArray());
 
//   QPixmap canvas(10,10);
//   canvas.fill(Qt::white);
//   { // draw the Blipitem onto the canvas
//     QPainter painter(&canvas);
//     painter.translate(0,0);  
//     painter.setRenderHint(QPainter::Antialiasing);
//     paint(&painter,/*option*/ nullptr,/*widget*/ nullptr);
//     painter.end();
//     canvas.setMask(canvas.createHeuristicMask());
//     
//     /** \todo Scale to with the current zoom level. The canvas is painted in 
//      * global coordinates (I think), so the level of detail might need to be
//      * considered.
//      */
//   }
//   drag->setPixmap(canvas);
  
  
  ControlStation* cs = dynamic_cast<ControlStation*>(q->parent());

  if( cs != nullptr )
  { cs->setDraggedBlip(q->id());}
    
  drag->exec();
  
  if( cs != nullptr )
  { cs->unsetDraggedBlip();}
}

void BlipItem::mousePressEvent(QGraphicsSceneMouseEvent* event)
{
  switch( event->button() )
  {
    case Qt::LeftButton :
//       setCursor(Qt::CrossCursor);
      //NOTE: There seems to be a bug in Qt (for X11) which resets the cursor
      // for dragging operations to the 'default': ClosedHandCursor :/
      setCursor(Qt::ClosedHandCursor);
      break;
    case Qt::RightButton :
      break;
    default: break;
  }
}

void BlipItem::mouseReleaseEvent(QGraphicsSceneMouseEvent* event)
{
  switch( event->button() )
  {
    case Qt::LeftButton :
      setCursor(Qt::ArrowCursor);
      break;
    default: break;
  }
}


void BlipItem::contextMenuEvent(QGraphicsSceneContextMenuEvent* event)
{ 
  QMenu menu;
  q->parent()->buildBlipContextMenu(&menu,q->id());
  menu.exec(event->screenPos());
}









// Blip* BlipItem::blip() const
// {
//  return q;
// }
// 
// void BlipItem::setBlip(Blip* blip)
// {
//   q = blip;
// }




QPainterPath BlipItem::generateGustStyleMarker(qreal const & r) const
{
  QPainterPath path;
   
  // circle with radius r centered at (0,0)
  path.addEllipse(-r,-r,2*r,2*r); 
  
  
  // line form center to the "back"
  path.moveTo(0,0);
  path.lineTo(-r,0); 
  
  return path;
}





  
  
}