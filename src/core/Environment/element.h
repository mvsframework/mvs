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

#ifndef ELEMENT_H
#define ELEMENT_H

#include <QtCore/QObject>
#include <QtCore/QScopedPointer>
#include <QtCore/QMetaProperty>

#include "config.h" // provides WITH_GRAPHICS
#include "elementid.h"
#include "location.h"
#include "Geometry/geometry.h"

#ifdef WITH_GRAPHICS
  #include "Graphics/geometryitem.h"
#endif // WITH_GRAPHICS

class ElementPrivate; // Forward declaration.

/** \internal 
 * Elements should not have an API for Locations, although they clearly have a
 * position. The problem is two fold: on the one hande, this would lead to two
 * identifyers, the ElementId and the LocationId. On the other hand, a 
 * Element::location() method would need access to an outside database that 
 * holds these locations. (Currently Obstacle provides a way to associate a 
 * Location to an Element via the ObstacleGraph::Vertex_Descriptor. */
class Element : public QObject
{
Q_OBJECT
Q_PROPERTY(ElementId id READ id WRITE setId NOTIFY idChanged)


#ifdef WITH_GRAPHICS
  Q_PROPERTY(bool shapeVisibility       READ isShapeVisible       WRITE setShapeVisibility      NOTIFY shapeVisibilityChanged )
  Q_PROPERTY(bool convexHullVisibility  READ isConvexHullVisible  WRITE setConvexHullVisibility NOTIFY convexHullVisibilityChanged )
#endif // WITH_GRAPHICS

Q_SIGNALS:
  /** \brief Emitted when the ElementId of this Element has changed. */
  void idChanged(ElementId /*id*/);
  
  /** \brief Emitted when the shape of the obstacle has changed. */
  void shapeChanged();
  
#ifdef WITH_GRAPHICS  
  /** \brief Emitted when the visibility of the shape graphics has changed. */
  void shapeVisibilityChanged(bool /*visibility*/);
  
  /** \brief Emitted when the visibility of the shape's convex hull graphics 
   * has changed. */
  void convexHullVisibilityChanged(bool /*visibility*/);
#endif // WITH_GRAPHICS  
  

public:
  
  using Id = ElementId;
  
  
  /** \brief Default constructor. */
  Element(QObject* parent = nullptr);
  
  /** \brief Copy constructor.
   * \internal Using the copy assingment operator internally.
   */
  Element(Element const & other);
  
  /** \brief Destructor. */
  virtual ~Element();
  
  /** \brief Copy assignment operator. */
  Element& operator=(Element const & other);
  
  /** \brief Equality comparison operator.
   * \details Equality is based upon the ElementId being identical.
   */
  bool operator==(Element const & other);
  
  Geometry::Polygon_2D const & shape() const;///< \brief Get the shape of the Element.
  
  void setShape(Geometry::Polygon_2D const & shape);///< \brief Set the shape of the Element.
  
  /** \brief Get the centroid of the shape of the Element. */
  Geometry::Point_2D   centroid() const;
  
  Geometry::Polygon_2D convexHull() const; ///< \brief Get the convex hull of the Element.

  /** \brief Get the identifyer of this Element. */
  ElementId id() const;
 
  /** \brief Set the ElementId of this Element. */
  void setId(ElementId const & id);  
  
#ifdef WITH_GRAPHICS
  
  void  setParentItem(QGraphicsItem * newParent);
  
  QColor  mainColor() const;
  virtual void setMainColor(QColor const & color);
  
  /** \brief Change the color of the Element graphics to the target color.
   * \details Overrides the Element's mainColor information with the global 
   * color information for targtes given by Graphics_2D::Colors::targetElement.
   * \sa markAsNonTarget, removeTargetMarker */
  virtual void markAsTarget();
  
  /** \brief Change the color of the Element graphics to the non-target color.
   * \details Overrides the Element's mainColor information with the global 
   * color information for non-targtes given by Graphics_2D::Colors::nonTargetElement.
   * \sa markAsTarget, removeTargetMarker */
  virtual void markAsNonTarget();
  
  /** \brief Remove any special markers and restore the main color.
   * \details Removes any special graphical markers and restores the Element's
   * representation to mainColor().
   * \sa markAsTarget, markAsNonTarget */
  virtual void removeTargetMarker();
  
  /** \brief Get a context menu action for the element. */
  QAction* menuAction();
  
  QGraphicsItem* shapeItem();
  
  //@{
  /* \brief Controls the visibility of the COM graph graphics.
   * \sa shapeItem */
  bool isShapeVisible() const;
  virtual void setShapeVisibility(bool const & visibility);
  void showShape();
  void hideShape();
  //@}
  
  QGraphicsItem* convexHullItem();
  
  //@{
  /* \brief Controls the visibility of the COM graph graphics.
   * \sa convexHullItem */
  bool isConvexHullVisible() const;
  virtual void setConvexHullVisibility(bool const & visibility);
  void showConvexHull();
  void hideConvexHull();
  //@}
  
  void setZValue( qreal z);
  
#endif // WITH_GRAPHICS  

protected:
  QScopedPointer<ElementPrivate> const d_ptr;
  
  /** \brief Constructor optimized for d-pointer inheritance.
   * See http://qt-project.org/wiki/Dpointer for more explanations.
   */
  explicit Element( ElementPrivate& dd, QObject* parent = 0 );

  Geometry::Polygon_2D& mutableShape();///< \brief Get a mutable shape of the Element.
  

  
private:
//   Q_DISABLE_COPY(Element)
  Q_DECLARE_PRIVATE(Element)
};


//############################################################################//
//                                                                            //
//  I/O Support                                                               //
//                                                                            //
//############################################################################//



inline std::ostream& operator<<(std::ostream& out, const Element& element )
{ 
  return out 
  << "Element" << element.id(); 
};

inline QDebug operator<<(QDebug dbg, const Element& element )
{
  dbg.nospace() << "\nOBJECT Element::" << element.objectName();
  
  { // dump properties
    dbg.nospace() << "\n  PROPERTIES";
    dbg.nospace() << "\n    id: " << element.id();
  }
  
#ifdef WITH_GRAPHICS
  { // dump graphics
    dbg.nospace() << "\n  PROPERTIES (GRAPHICS)";
    dbg.nospace() << "\n    shapeVisibility: " << element.isShapeVisible();      
    dbg.nospace() << "\n    convexHullVisibility: "<< element.isConvexHullVisible();
  }
#endif // WITH_GRAPHICS
  
  return dbg.space();
};






Q_DECLARE_METATYPE(ElementId);

#endif // ELEMENT_H
