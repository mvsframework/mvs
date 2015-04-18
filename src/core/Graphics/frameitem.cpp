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

#include "frameitem.h"
#include <QtGui/QPainter>
#include <QtCore/QDebug>
#include <QtCore/QThread>

namespace Graphics_2D {

FrameItem::FrameItem()
{
  { // create the L-shape indicating the origin.
    myPath.moveTo(10,0);
    myPath.lineTo(0,0);
    myPath.lineTo(0,10);
  }
  
  { // initialize pen
    myPen.setWidthF(0.0);
    myPen.setColor(Qt::black);
  }
  
}

FrameItem::~FrameItem()
{
//   qDebug() << "Deleting a FrameItem:\n"
//   << "\nFrameItem::this =" << this << "\n"
// //   << "\tCoordinateSystem::this->thread() =" << this->thread() << "\n"
// //   << "\tQApplication::instance()->thread() =" << QApplication::instance()->thread() << "\n"
//   << "\tQThread::currentThread() =" << QThread::currentThread() << "\n"
//   << "\t__builtin_return_address(0) = " << __builtin_return_address(0) << "\n"
//   << "\t__builtin_extract_return_addr(__builtin_return_address(0)) ="<< __builtin_extract_return_addr(__builtin_return_address(0)) <<"\n"
//   << "\t__builtin_frame_address(0) ="<<__builtin_frame_address(0);
}

void FrameItem::paint ( QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget )
{
  painter->setPen(myPen);
  painter->drawPath(myPath);
}

QPainterPath FrameItem::shape() const
{ return myPath; }

QRectF FrameItem::boundingRect() const
{ return myPath.boundingRect(); }



} // end namespace Graphics_2D