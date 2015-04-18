/*
 *  Copyright © 2015 Claus Christmann <hcc |ä| gatech.edu>.
 * 
 * 
 * Inspired by http://doc.qt.nokia.com/4.7-snapshot/graphicsview-diagramscene-arrow-cpp.html :
 * 
 ** Copyright (C) 2011 Nokia Corporation and/or its subsidiary(-ies).
 ** 
 ** Contact: Nokia Corporation (qt-info@nokia.com)
 **
 ** This file is part of the examples of the Qt Toolkit.
 **
 ** $QT_BEGIN_LICENSE:BSD$
 ** You may use this file under the terms of the BSD license as follows:
 **
 ** "Redistribution and use in source and binary forms, with or without
 ** modification, are permitted provided that the following conditions are
 ** met:
 **   * Redistributions of source code must retain the above copyright
 **     notice, this list of conditions and the following disclaimer.
 **   * Redistributions in binary form must reproduce the above copyright
 **     notice, this list of conditions and the following disclaimer in
 **     the documentation and/or other materials provided with the
 **     distribution.
 **   * Neither the name of Nokia Corporation and its Subsidiary(-ies) nor
 **     the names of its contributors may be used to endorse or promote
 **     products derived from this software without specific prior written
 **     permission.
 **
 ** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 ** "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 ** LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 ** A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 ** OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 ** SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 ** LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 ** DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 ** THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 ** (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 ** OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE."
 ** $QT_END_LICENSE$
 ** 
 * 
 * 
 */

#include "Graphics/arrowitem.h"
#include <QtCore/QDebug>
#include <QtCore/QThread>
#include <cmath>

namespace Graphics_2D {

  
Arrow::Arrow ( QGraphicsItem* parent )
  :QGraphicsLineItem(parent)
{
  QPen pen(arrowColor);
  pen.setWidth(0);
  pen.setJoinStyle(Qt::RoundJoin);
  pen.setCapStyle(Qt::RoundCap);
  
  QGraphicsLineItem::setPen(pen);
  
  arrowHead = new QGraphicsPolygonItem(this);
  arrowHead->setPen(pen);
  arrowHead->setBrush(arrowColor);
  
  updateArrowHead();
}


Arrow::Arrow(qreal x1, qreal y1, qreal x2, qreal y2, QGraphicsItem* parent)
  :Arrow(parent)
{
  setLine(x1,y1,x2,y2);
}


Arrow::~Arrow()
{
//   qDebug() << "Deleting an Arrow:\n"
//   << "\nArrow::this =" << this << "\n"
// //   << "\tCoordinateSystem::this->thread() =" << this->thread() << "\n"
// //   << "\tQApplication::instance()->thread() =" << QApplication::instance()->thread() << "\n"
//   << "\tQThread::currentThread() =" << QThread::currentThread() << "\n"
//   << "\t__builtin_return_address(0) = " << __builtin_return_address(0) << "\n"
//   << "\t__builtin_extract_return_addr(__builtin_return_address(0)) ="<< __builtin_extract_return_addr(__builtin_return_address(0)) <<"\n"
//   << "\t__builtin_frame_address(0) ="<<__builtin_frame_address(0);
}


void Arrow::setColor ( const QColor& color )
{
  arrowColor= color;
  QPen myPen = pen();
  myPen.setColor(color);
  
  QGraphicsLineItem::setPen(myPen);
  arrowHead->setPen(myPen);
  arrowHead->setBrush(arrowColor);
}


void Arrow::updateArrowHead()
{
  QPolygonF poly;
  if( arrowHeadSize != 0 and line().length() > 0.0)
  {
    double angle = ::acos(line().dx() / line().length());
    if (line().dy() >= 0)
    {
      angle = (M_PI * 2) - angle;
    }

    QPointF arrowP1 = line().p2() - QPointF(sin(angle + M_PI / 3.0) * arrowHeadSize,
                                            cos(angle + M_PI / 3.0) * arrowHeadSize);
    QPointF arrowP2 = line().p2() - QPointF(sin(angle + M_PI - M_PI / 3.0) * arrowHeadSize,
                                            cos(angle + M_PI - M_PI / 3.0) * arrowHeadSize);
    poly << line().p2() << arrowP1 << arrowP2;
  }
  arrowHead->setPolygon(poly);
}


void Arrow::setHeadSize ( const qreal& headSize )
{
  arrowHeadSize = headSize;
  
  // head sizes should be positive or zero.
  if( headSize < 0.0 )
  { arrowHeadSize *= -1.0; }
  

  updateArrowHead();
}

void Arrow::setHeadPosition ( const QPointF& position )
{
  auto l = line();
  l.setP2(position);
  setLine(l);
}

void Arrow::setRootPosition ( const QPointF& position )
{
  auto l = line();
  l.setP1(position);
  setLine(l);
}


// QRectF Arrow::boundingRect() const
// {
//   if( arrowHeadSize != 0.0 )
//   {
//     auto lineRect = QGraphicsLineItem::boundingRect();
// //     auto arrowRect = arrowHead.boundingRect();
// //     auto combinedRect = lineRect.united(arrowRect);
//     return  lineRect;
//   }
//   else
//   {
//     auto lineRect = QGraphicsLineItem::boundingRect();
//     return lineRect;
//   }
// }
// 
// 
// QPainterPath Arrow::shape() const
// {
//   if( arrowHeadSize != 0.0 )
//   {
//      QPainterPath path = QGraphicsLineItem::shape();
//      path.addPolygon(arrowHead);
//      return path;
//   }
//   else
//   {
//     return QGraphicsLineItem::shape();
//   }
// }



} // end namespace Graphics_2D











