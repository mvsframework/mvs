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

#include "graphicsview.h"
#include <QtCore/QDebug>

GraphicsView::GraphicsView ( QGraphicsScene* scene, QWidget* parent )
  :QGraphicsView (scene, parent)
{
  setCacheMode(CacheBackground);
  setViewportUpdateMode(BoundingRectViewportUpdate);
  setRenderHint(QPainter::Antialiasing);
  setTransformationAnchor(AnchorUnderMouse);
  setDragMode(QGraphicsView::ScrollHandDrag);
  rotate(-90); //FIXME: Magic Numbers
  setMinimumSize(256,256); //FIXME: Magic Numbers
}


void GraphicsView::zoomIn()
{
  scaleView(pow(2.0, 1.0)); //FIXME: Magic Numbers
}

void GraphicsView::zoomOut()
{
  scaleView(pow(2.0, -1.0)); //FIXME: Magic Numbers
}


void GraphicsView::keyPressEvent ( QKeyEvent* event )
{
  switch( event->key() )
  {
    case Qt::Key_Up:
    case Qt::Key_Down:
    case Qt::Key_Left:
    case Qt::Key_Right:
      // not implemented
      break;
    case Qt::Key_Plus:
      zoomIn();
      break;
    case Qt::Key_Minus:
      zoomOut();
      break;
    default:
      QGraphicsView::keyPressEvent ( event );  
  }
}

void GraphicsView::wheelEvent ( QWheelEvent* event )
{
  
  /** \internal Old:
   * My Microsoft Natural Wireless Laser Mouse 7000 seems to send a wheel event
   * for every 1.25° of rotation, although with a value of "120".
   * The y-direction is the wheel forward-backward, the x-direction is 
   * left-right. 
   * 
   * With Qt 5.3 and KDE 4.3 the mouse seems to behave more "standart" like:
   * From the Qt documentation for QWheelEvent::angleDelta():
   * 
   * "Returns the distance that the wheel is rotated, in eighths of a degree. A 
   * positive value indicates that the wheel was rotated forwards away from the 
   * user; a negative value indicates that the wheel was rotated backwards 
   * toward the user.
   * Most mouse types work in steps of 15 degrees, in which case the delta 
   * value is a multiple of 120; i.e., 120 units * 1/8 = 15 degrees."
   */
  
  QPoint numDegrees = event->angleDelta() / 8;
  
//   int x = numDegrees.x();
//   qDebug() << "wheelevent.angleDelta().x() =" << x;
  
  int y = numDegrees.y();
//   qDebug() << "wheelevent.angleDelta().y() =" << y;
  
//   scaleView( pow(2.0, 0.05* -(event->angleDelta().y()/ 120.0) ) ); 
  scaleView( pow(2.0,  0.5 *(y / 15) ) ); 
}

void GraphicsView::scaleView ( qreal scaleFactor )
{
//   qDebug("Scaling the view with a factor of %.3f",scaleFactor);
  qreal factor = transform().scale(scaleFactor, scaleFactor).mapRect(QRectF(0, 0, 1, 1)).width();
    if (factor < 0.07 || factor > 100)
        return;

    scale(scaleFactor, scaleFactor);
}

void GraphicsView::contextMenuEvent(QContextMenuEvent* event)
{
    QGraphicsView::contextMenuEvent(event);
}




#include "graphicsview.moc"