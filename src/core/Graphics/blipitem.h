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

#ifndef BLIPITEM_H
#define BLIPITEM_H

#include <QtWidgets/QAbstractGraphicsShapeItem>
#include "blip.h"


/** \addtogroup Graphics
 *  @{
 */ 
namespace Graphics_2D {

/** \brief A GUST style marker to represent a Blip on a map.
 * \details A marked shaped similar to the GUST symbology, i.e. a circle 
 * centered at the location of the Blip and a single spoke from the center 
 * to the circle, indicating the tail of the vehicle (thinkg tail boom). */  
class BlipItem : public QAbstractGraphicsShapeItem
{
public:
  /** \brief Default constructor.
   * \param[in] blip The Blip this graphics item represents.
   * \param[in] parent (optional) The hirarchical parent of this item. */
  BlipItem(Blip* blip, QGraphicsItem* parent = nullptr);
  
  /** \brief Destructor. */
  virtual ~BlipItem();
  
  virtual QRectF  boundingRect() const; 
  virtual void  paint(QPainter * painter, const QStyleOptionGraphicsItem * option, QWidget * widget = 0); 

//   void setBlip(Blip * blip);
//   Blip * blip() const;
  
  
protected:
  virtual void mousePressEvent(QGraphicsSceneMouseEvent* event);
  virtual void mouseMoveEvent(QGraphicsSceneMouseEvent* event);
  virtual void mouseReleaseEvent(QGraphicsSceneMouseEvent* event);
  
  virtual void contextMenuEvent(QGraphicsSceneContextMenuEvent* event);
  
  
private:
  Blip* q;
  
  /** \brief Generate a marked shaped similar to the GUST symbology. 
   * \param[in] r (optional) The radius of the circle.
   * \return The GUST marker shape: A circle with radius \c r and a line from
   *  its center to the "back" (i.e. (x,y)=(0,-r)) 
   */
  virtual QPainterPath generateGustStyleMarker(qreal const & r = 2) const;
};

} // end namespace Graphics_2D
/** @}*/
#endif // BLIPITEM_H
