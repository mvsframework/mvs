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

#ifndef FRAMEITEM_H
#define FRAMEITEM_H

#include <QtWidgets/QGraphicsItem>
#include <QtGui/QPen>


/** \addtogroup Graphics
 *  @{
 */ 

namespace Graphics_2D {

class FrameItem : public QGraphicsItem {
public:
  FrameItem();
  ~FrameItem();
 
  
  virtual QRectF boundingRect() const;
  virtual void  paint(QPainter * painter, const QStyleOptionGraphicsItem * option, QWidget * widget = 0);
  virtual QPainterPath  shape() const;
  
private:
  Q_DISABLE_COPY(FrameItem)

  QPainterPath myPath;
  QPen myPen;
     
};

} // end namespace Graphics_2D
/** @}*/

#endif // FRAMEITEM_H
