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

#ifndef GEOMETRYITEM_H
#define GEOMETRYITEM_H

#include <QtWidgets/QWidget>
#include <QtWidgets/QStyleOptionGraphicsItem>
#include <QtWidgets/QAbstractGraphicsShapeItem>
#include <QtGui/QPainter>

#include <boost/geometry.hpp>
#include <boost/geometry/core/point_type.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/geometries/geometries.hpp>
#include <boost/geometry/geometries/ring.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/graph/graph_concepts.hpp>

#include "Geometry/geometry.h"

#include <iostream>

#include <assert.h>
#include <typeinfo>

/** \addtogroup Graphics
 *  @{
 */ 


namespace Graphics_2D {

  
template <typename Tag> struct dispatch {};  
  
  
// Specialization for points
template <> struct dispatch<boost::geometry::point_tag>
{
  
  template <typename Point>
  static inline void print(Point const& p)
  {
      // Use the Boost.Geometry free function "get"
      // working on all supported point types
      std::cout << "Hello POINT, you are located at: "
          << boost::geometry::get<0>(p) << ", "
          << boost::geometry::get<1>(p)
          << std::endl;
  }

  template <typename Point>
  static inline QPainterPath generatePath(Point const& p)
  {
    QPainterPath path;
    constexpr qreal radius = 1.0;
    path.addEllipse(QPointF(p.x(), p.y()),radius,radius);
    return path; // no real paths for points
  }
  
};  

// Specialization for line strings
template <> struct dispatch<boost::geometry::linestring_tag>
{
  template <typename LineString>
  static inline void print(LineString const& l)
  {
    // Use the Boost.Geometry manipulator "dsv" 
    // working on all supported geometries
    std::cout << "Hello LINESTRING, you look like: "
      << boost::geometry::dsv(l)
      << std::endl;
  };
  
  template <typename LineString>
  static inline QPainterPath generatePath(LineString const& l, bool shiftToMinCorner = false)
  {
    using namespace boost::geometry;
    
    if( num_points(l) == 0 )
    {return QPainterPath(); }
    

    typename point_type<LineString>::type shift;
    if( shiftToMinCorner ) {
      model::box<  typename point_type<LineString>::type > box;
      envelope(l,box);
      assign_point( shift, box.min_corner() ); // shift = box.min_corner();
    } else {
      assign_zero(shift);
    }
    
    QPainterPath path;

   
    auto point_it = l.begin();
    path.moveTo(point_it->x()-shift.x(),point_it->y()-shift.y());
    
    // iterate from the second point to the last point
    for( ++point_it ; point_it != l.end(); ++point_it )
    {
     path.lineTo(point_it->x()-shift.x(),point_it->y()-shift.y()); 
    } //NOTE: the point_it stands at the one-behond-last point as if now
    
    return path;
  };
};


// Specialization for rings
template <> struct dispatch<boost::geometry::ring_tag>
{
  
  template <typename Ring>
  static inline void print(Ring const& r)
  {
      // Use the Boost.Geometry manipulator "dsv" 
      // working on all supported geometries
      std::cout << "Hello RING, you look like: "
          << boost::geometry::dsv(r)
          << std::endl;
  }
  
  template <typename Ring>
  static inline QPainterPath generatePath(Ring const& r, bool shiftToMinCorner = false)
  { 
    using namespace boost::geometry;
    

    if( num_points(r) == 0 )
    {return QPainterPath(); }
    
    typename point_type<Ring>::type shift;
    if( shiftToMinCorner ) {
      model::box<  typename point_type<Ring>::type > box;
      envelope(r,box);
      assign_point( shift, box.min_corner() ); // shift = box.min_corner();
    } else {
      assign_zero(shift);
    }
    
    QPainterPath path;

   
    auto point_it = r.begin();
    path.moveTo(point_it->x()-shift.x(),point_it->y()-shift.y());
    
    // iterate from the second point to the second to last point
    for( ++point_it ; point_it != --r.end(); ++point_it )
    {
     path.lineTo(point_it->x()-shift.x(),point_it->y()-shift.y()); 
    } //NOTE: the point_it stands at the last point as if now
    
    // deal with the closed-ness of the ring
    // NOTE: Rings are assumed to be closed (otherwise they would be 
    // LineStrings), the question is simply how this is represented in the 
    // underlying collection of points of the ring!
    if( closure< Ring >::value == closed ) // i.e. first_point == last_point
    { 
      // skip the last point in the point collection as first_point == last_point
      path.closeSubpath(); // force a closure
    }
    if( closure< Ring >::value == open ) // i.e. first_point != last_point
    {
      // make use of the ending iterator position after the foor loop
      // and add the last point
      path.lineTo(point_it->x()-shift.x(),point_it->y()-shift.y());
      //NOTE: although I assume all rings to be closed, I don't forcefully 
      // close the Qt path here, hoping that his "graphical inconsistency" might
      // provide a hint at the "wrong" code implementation. I.e. I don't want
      // to auto-fix stuff here that "the programmer" would have to consiciously
      // do himself.
    }
    
//     std::cout << "path element count : "<< path.elementCount() << std::endl;
        
    return path;
  }
  
};  
  
// Specialization for polygons
template <> struct dispatch<boost::geometry::polygon_tag>
{
  
  template <typename Polygon>
  static inline void print(Polygon const& p)
  {
      // Use the Boost.Geometry manipulator "dsv" 
      // working on all supported geometries
      std::cout << "Hello POLYGON, you look like: "
          << boost::geometry::dsv(p)
          << std::endl;
  }
  
  template <typename Polygon>
  static inline QPainterPath generatePath(Polygon const& p)
  {
    using namespace boost::geometry;
    
    QPainterPath path;
    
    typedef typename ring_type<Polygon>::type ring_type;    
    ring_type outerRing = exterior_ring(p);
    
//     std::cout << "ring_type : " << typeid(ring_type).name() << std::endl;
//     std::cout << "decltypa  : " << typeid(decltype(outerRing)).name() << std::endl;
    
    // add the outer shape of the polygon, which is a ring
    path.addPath( dispatch< typename tag<ring_type>::type >::generatePath(outerRing) );
    

    // add the inner rings of the polygon
    for( ring_type innerRing : interior_rings(p) )
    {
      auto innerPath = dispatch< typename tag<ring_type>::type >::generatePath(innerRing);
      path.addPath( innerPath );
    }
    
    
    return path;
    
  }
};  



/** \brief A basic graphical item for an underlying Boost geometry
 * 
 * Remember, all drawing is done in local coordinates. For most simple
 * geometries that means that "local" coordinates are coordinates in the 
 * determining coordinate system:
 * 
\verbatim
  Y
  |
 3|           c
  |   d+------+
 2|    |      |
  |    |      |
 1|    x------+b
  |    a       
 0+------------------ X
  0    1    2    3
\endverbatim 
 *
 * When this rectangle (polygon) is painted, the local origin of the item
 * is the origin of the coordinate system. This means that the line (ab) is
 * drawn as the line from (1,1) to (2.5,1). (this also means that rotations
 * are executed around the origin of the coordinate system!)
 * If this were to be drawn as a "proper" QGraphics Item, the item would 
 * have its own origin (say "x" located at a) and the line (ab) would be
 * drawn as a line from (0,0) to (1.5,0), wrt. the origin "x". (In this case
 * rotations would be executed around a!)
 */
template<typename Geometry>
// class GeometryItem : public QGraphicsItem
class GeometryItem : public QAbstractGraphicsShapeItem
{   
public:
  /** \brief Constructor. */
  GeometryItem(Geometry& geometry, QGraphicsItem* parent = nullptr)
    :QAbstractGraphicsShapeItem(parent)
//     :QGraphicsItem(parent)
    ,geometry(geometry)
  {
    QPen pen;
    
    pen.setColor(Qt::black);
    pen.setWidthF(0.0);
    pen.setJoinStyle(Qt::RoundJoin);
    pen.setCapStyle(Qt::RoundCap);
    
    setPen( pen );
    //   std::cout << "bounding box: " << boost::geometry::dsv(envelope()) << std::endl;
    //   std::cout << "shape : " << boost::geometry::dsv(geometry) << std::endl;
    //   std::cout << "area : " << boost::geometry::area(geometry) << std::endl;
  };
  
  /** \brief Destructor. */
  virtual ~GeometryItem() = default;
  
  /** \brief Assingment operator. */
  GeometryItem<Geometry>& operator=(const GeometryItem<Geometry>& rhs) = delete;
    
  using point_type = typename boost::geometry::point_type<Geometry>::type; // this is a meta function: http://www.boost.org/doc/libs/1_51_0/libs/geometry/doc/html/geometry/reference/core.html
  using box_type   =          boost::geometry::model::box<point_type>;
//   using ring_type  = typename boost::geometry::ring_type<Geometry>::type; // this is a meta function: http://www.boost.org/doc/libs/1_51_0/libs/geometry/doc/html/geometry/reference/core.html

  virtual void paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget = 0)
  {
    
    QPainterPath path = dispatch< typename boost::geometry::tag<Geometry>::type >
                          ::generatePath(geometry);
                          
    painter->setPen(pen());                        
    painter->setBrush(brush());
    painter->drawPath(path);
  };
  
  virtual QRectF boundingRect() const
  {
    auto boundingBox = envelope();
    
    qreal delta_x = boundingBox.max_corner().x()-boundingBox.min_corner().x();
    qreal delta_y = boundingBox.max_corner().y()-boundingBox.min_corner().y();


    // Add a half pen width to the bounding box, as pens are drawn centered.
    // Compare https://qt-project.org/doc/qt-5.0/qtwidgets/qgraphicsitem.html#boundingRect
    return QRectF(boundingBox.min_corner().x()-pen().widthF()/2.0,
                  boundingBox.min_corner().y()-pen().widthF()/2.0,
                  delta_x+pen().widthF(),
                  delta_y+pen().widthF());
  };
  
  
  //
  // Accessors
  //
  
  /** \brief Returns the shape of this item as a QPainterPath in local coordinates.
   * The shape is used for many things, including collision detection, hit 
   * tests, and for the QGraphicsScene::items() functions.
   * 
   * \todo The outline of a shape can vary depending on the width and style of the pen
   * used when drawing. This is \e not implemented yet
   */
  virtual QPainterPath shape() const
  {
    return dispatch< typename boost::geometry::tag<Geometry>::type >
            ::generatePath(geometry);
  };
  
//   /** \brief Returns the item's brush.
//    * ...or an empty brush if no brush has been set. 
//    * \sa setBrush()
//    */
//   QBrush brush() const { return p_brush; };  
//   
//   /** \brief Returns the item's pen.
//    * If no pen has been set, this function returns QPen(), a default black solid
//    * line pen with 0 width.
//    * \sa setPen()
//    */
//   QPen   pen() const {return p_pen; };
  
  //
  // Mutators
  //
  
  /** \brief Alter the shape of the geometry of the item. 
   * \details This function does not change which geometry this item represents,
   * but is changes the shape of that geometry.
   * \param[in] newGeometry The geometry this item will copy.
   */
  void setGeometry(const Geometry& newGeometry)
  {
    prepareGeometryChange();
    boost::geometry::assign(geometry,newGeometry);
  };
  
//   /** \brief Sets the item's brush to brush.
//    * The item's brush is used to fill the item.
//    * \note There are a bunch of implicit conversions for QBrush, e.g. from 
//    *  QColor, effectively creating an overload for this function.
//    * \sa brush()
//    */
//   void setBrush(const QBrush& brush) { p_brush = brush; };
// 
//   /** \brief Sets the pen for this item to pen.
//    * The pen is used to draw the item's outline.
//    * \note There are a bunch of implicit conversions for QPen, e.g. from 
//    *  QColor, effectively creating an overload for this function.
//    * \sa pen()
//    */
//   void setPen(const QPen& pen) { p_pen = pen; };

  
  /** \brief Prepares the item for a geometry change.
   * From the Qt documentation:
   * Call this function before changing the bounding rect of an item to keep 
   * QGraphicsScene's index up to date.
   * prepareGeometryChange() will call update() if this is necessary.
   * 
   * Example:
\code
void CircleItem::setRadius(qreal newRadius)
 {
     if (radius != newRadius) {
         prepareGeometryChange();
         radius = newRadius;
     }
 }
\endcode
   * 
   * \note This function simply exposes the protected 
   * QGraphicsItem::prepareGeometryChange() as a public function.
   */ 
//   void prepareGeometryChange()
//   { QGraphicsItem::prepareGeometryChange(); };
   
protected:
//   QBrush p_brush; /// \brief The brush used to paint this item's area(s).
//   QPen   p_pen;   /// \brief The pen used to paint this item's edges.
  
  /** \brief Get the boost envelope (i.e. bounding box) of the shape.
   * This function returns the bounding box in coordinates matching the ones
   * shape uses, i.e. \e not in Qt item coordinates.
   * 
   * \sa boundingRect() for getting the bounding box in item coordinates.
   */
  box_type envelope() const
  {
    boost::geometry::model::box<point_type> boundingBox;
    boost::geometry::envelope(geometry,boundingBox);
    
    return boundingBox;
  };
  
  /** \brief The boost geometry representing the shape of the graphics item. 
   * \todo Should this be a (smart) pointer?
   */
  Geometry& geometry; //NOTE: I needed to get rid of the const in order to make setGeometry() work.... :/
//   const Geometry& geometry; 
};

using Point = GeometryItem<Geometry::Point_2D>;

using Polygon = GeometryItem<Geometry::Polygon_2D>;

} // end namespace Graphics_2D

/** @}*/
#endif // GEOMETRYITEM_H
