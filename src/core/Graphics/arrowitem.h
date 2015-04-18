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

#ifndef ARROWITEM_H
#define ARROWITEM_H

#include <QtWidgets/QGraphicsLineItem>
#include <QtWidgets/QStyleOptionGraphicsItem>
#include <QtWidgets/QGraphicsScene>
#include <QtGui/QPolygon>
#include <QtGui/QPainter>

/** \addtogroup Graphics
 *  @{
 */ 

namespace Graphics_2D {

/** \brief A 2D arrow graphics item.
 * 
 * The arrow is pointed from line().p1() to line().p2(), i.e. the arrow head is
 * located at p2.
 * The head is a symmetrical triangle of size arrowHeadSize.
 */
class Arrow : public QGraphicsLineItem
{
public:
  /** \brief Default (empty) Constructor. */
  Arrow(QGraphicsItem* parent = nullptr);
  /** \brief Constructor with coordinates. */
  Arrow(qreal x1, qreal y1, qreal x2, qreal y2,QGraphicsItem* parent = nullptr);
    
  /** \brief Default Destructor. */
  virtual ~Arrow(); 
  
//   virtual void paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget = 0);
//   virtual QRectF boundingRect() const;
//   virtual QPainterPath shape() const;
  
  /** \brief Getting the size of the arrow's head. */
  qreal headSize() const {return arrowHeadSize; }; 
  
  /** \brief Setting the size of the arrow's head. */
  void setHeadSize(const qreal& headSize); 
  
  /**  \brief Getting the color of the arrow. */
  QColor color() const { return arrowColor; }; 
  
  /** \brief Setting the color of the arrow. */
  void setColor(const QColor& color);
  
  /** \brief Getting the position of the arrow's head. */
  QPointF headPosition() const { return line().p2();};
  
  /** \brief Setting the position of the arrow's head. */
  void setHeadPosition(QPointF const & position);
  
  /** \brief Getting the position of the arrow's root. */
  QPointF rootPosition() const {return line().p1();};
  
  /** \brief Setting the position of the arrow's root. */
  void setRootPosition(QPointF const & position);
  
  void setLine(QLineF const& line)
  {
   QGraphicsLineItem::setLine(line);
   updateArrowHead();
  };
  
  void setLine(qreal x1, qreal y1, qreal x2, qreal y2)
  {
    QGraphicsLineItem::setLine(x1,y1,x2,y2);
    updateArrowHead();
  };
  
  
private:
  Q_DISABLE_COPY(Arrow)
  
  /** \brief Update the arrowHead based on the current line and arrowHeadSize. */
  void updateArrowHead();
  
  QColor    arrowColor = Qt::black; ///< \brief The color of the complete arrrow.
  qreal     arrowHeadSize = 2.0; ///< \brief The size of the arrow head (measured along the arrow line)
  
  QGraphicsPolygonItem* arrowHead; ///< \brief The polygon representing the head of the arrow.

};


} // end namespace Graphics_2D

/** @}*/

#endif // ARROWITEM_H
