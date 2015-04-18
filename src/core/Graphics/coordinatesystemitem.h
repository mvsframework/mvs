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

#ifndef COORDINATESYSTEMITEM_H
#define COORDINATESYSTEMITEM_H

#include <QtWidgets/QGraphicsItem>
#include <QtWidgets/QStyleOptionGraphicsItem>
#include <QtWidgets/QGraphicsScene>

#include <QtGui/QPainter>

#include "Graphics/arrowitem.h"

/** \addtogroup Graphics
 *  @{
 */ 

namespace Graphics_2D {

/** \brief A representation of a 2D coordinate system.
 * This item represents a 2-dimensional coordinate system at the origin of the
 * item. It draws 
 * - a Qt::red ArrowItem in the x-axis direction
 * - a Qt::green ArrowItem in the y-axis direction
 * - a Qt::blue crossed out ellipse to indicate the z-axis direction 
 */
class CoordinateSystem : public QGraphicsItem
{
public:
  /** \brief Default (empty) constructor. */
  CoordinateSystem(QGraphicsItem* parent = nullptr);  
  
  /** \brief Copy Constructor.
   * Deleted to dissallow usage.
   */
  CoordinateSystem(const CoordinateSystem& other) = delete;
  
  /** \brief Destructor.
   * ~QGraphicsItem also destroys all its children.
   */
  virtual ~CoordinateSystem();
  
  /** \brief Assingment operator.
   * Deleted to dissallow usage.
   */
  CoordinateSystem& operator=(const CoordinateSystem& rhs) = delete;
  

  virtual void paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget = 0);
  virtual QRectF boundingRect() const;
  virtual QPainterPath shape() const;
  
  /** \brief Set the visibility of the graphics representing the axes. */
  void setAxesVisible(bool visible);
  bool areAxesVisible() const;
  void showAxes() { setAxesVisible(true);  };
  void hideAxes() { setAxesVisible(false); };
    
protected:
  Arrow* xAxisArrow = nullptr;
  Arrow* yAxisArrow = nullptr;

  qreal arrowLength = 10;
  qreal arrowHeadSize = 2;
  
private:
  qreal zDiameter = 0.0;
  bool rightHandSystem = true; ///< \brief TRUE if the CoSy is right-handed, FALSE otherwise.
  
  bool axesVisible = true;
  
  QPainterPath zPath() const;
};

} // end namespace Graphics_2D


/** @}*/

#endif // COORDINATESYSTEMITEM_H
