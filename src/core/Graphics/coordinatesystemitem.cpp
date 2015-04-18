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

#include "Graphics/coordinatesystemitem.h"
#include <QtCore/QDebug>
#include <QtCore/QThread>
#include <cmath>

namespace Graphics_2D {

CoordinateSystem::CoordinateSystem( QGraphicsItem* parent )
  :QGraphicsItem(parent)
{
//   arrowLength = 10;
//   arrowHeadSize = 2;
  zDiameter = 2.0 * tan(M_PI / 6) * arrowHeadSize;
  
  xAxisArrow = new Arrow(this);
  xAxisArrow->setColor(Qt::red);
  xAxisArrow->setLine(0.0,0.0,arrowLength,0.0);
  xAxisArrow->setHeadSize(arrowHeadSize);
  
  yAxisArrow = new Arrow(this);
  yAxisArrow->setColor(Qt::green);
  yAxisArrow->setLine(0.0,0.0,0.0,arrowLength);
  yAxisArrow->setHeadSize(arrowHeadSize);
  
//   zCircle = new QGraphicsEllipseItem(this);
//   zCircle->setRect(-zDiameter/2.0, -zDiameter/2.0, zDiameter, zDiameter);
//   zCircle->setPen(QPen(Qt::blue));
//   zCircle->setBrush(Qt::NoBrush);
}

/** \internal From on a chat on #qt:
[19:06] \<odin_\> hcc23, can you graft a virtual destructor on your QGraphicsItem's ?  have it log this, thread affinity, current thread and the address of the caller of the method
[19:07] \<odin_\> hcc23, with GCC (I think you said opensuse/qt5 you use) you can look at this API http://gcc.gnu.org/onlinedocs/gcc/Return-Address.html
[19:08] \<hcc23\> odin_: Hey, thanks a lot for getting back to me :)
[19:08] \<odin_\> hcc23, at least you can visualize correctly with real data who and what is calling destructor and also confirm if threading is a concern
[19:08] \<hcc23\> That actually is a good idea, I'll look into it...
[19:09] \<hcc23\> Since you mention thread affinity and current thread separately, I'd guess they are separate (which I didn't realize). How could I get that information? (All I know is QObject::thread() )
[19:10] \<odin_\> hcc23, 'this' as in C++ this keyword, thread affinity as in QObject::thread() / this::thread(), current thread as in QThread::currentThread() and address of caller using GCC link/url above
[19:10] \<hcc23\> wow, that's detailed :) Thanks a lot :)
[19:10] \<odin_\> hcc23, thread affinity is related to data ('this'),  current thread is related to execution (the logical CPU I am running this code on)
[19:12] \<odin_\> maybe reusing the word 'this' was a poor choice there.... (the logical CPU I am running code on)
*/
CoordinateSystem::~CoordinateSystem()
{
// qDebug() << "Deleting a CoordinateSystem:\n"
//   << "\tCoordinateSystem::this =" << this << "\n"
// //   << "\tCoordinateSystem::this->thread() =" << this->thread() << "\n"
// //   << "\tQApplication::instance()->thread() =" << QApplication::instance()->thread() << "\n"
//   << "\tQThread::currentThread() =" << QThread::currentThread() << "\n"
//   << "\t__builtin_return_address(0) = " << __builtin_return_address(0) << "\n"
//   << "\t__builtin_extract_return_addr(__builtin_return_address(0)) ="<< __builtin_extract_return_addr(__builtin_return_address(0)) <<"\n"
//   << "\t__builtin_frame_address(0) ="<<__builtin_frame_address(0);
}



/** \todo Fix the arrow feathers of the z-Axis to be contained by the circle 
 *    element
 */
void CoordinateSystem::paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget)
{
  // the x-axis
//   xAxisArrow->paint(painter,option,widget); // since this is a child, it is automatically painted
  
  // the y-axis
//   yAxisArrow->paint(painter,option,widget); // since this is a child, it is automatically painted
  
  if( axesVisible )
  {
    // the z-axis
    QPen myPen ;
    myPen.setColor(Qt::blue);
    myPen.setWidthF(0);
    
    painter->setPen(myPen);
    painter->drawPath(zPath());
  } 
}

QPainterPath CoordinateSystem::zPath() const
{
  QPainterPath path;
  
  if( rightHandSystem )
  {
    path.moveTo(-zDiameter/2.0, -zDiameter/2.0);
    path.lineTo( zDiameter/2.0,  zDiameter/2.0);
    
    path.moveTo(-zDiameter/2.0,  zDiameter/2.0);
    path.lineTo( zDiameter/2.0, -zDiameter/2.0);
  }
  else
  {
    path.addEllipse(-zDiameter/2.0, -zDiameter/2.0, zDiameter, zDiameter);
    path.moveTo(0,0);
    path.lineTo(0,0);
  }
  
  return path;
}

QRectF CoordinateSystem::boundingRect() const
{
  QRectF rect(-zDiameter/2.0, -zDiameter/2.0, zDiameter, zDiameter);
  rect.united(yAxisArrow->boundingRect()).united(xAxisArrow->boundingRect());
  
  return rect;
}

QPainterPath CoordinateSystem::shape() const
{
  QPainterPath path = zPath();
  path.addPath(xAxisArrow->shape());
  path.addPath(yAxisArrow->shape());
  
  return path;
}
void CoordinateSystem::setAxesVisible ( bool visible )
{
  axesVisible = visible;
  yAxisArrow->setVisible(visible);
  xAxisArrow->setVisible(visible);
}

bool CoordinateSystem::areAxesVisible() const
{ return axesVisible; }



} // end namespace Graphics_2D

