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

#ifndef POINT_2D_H
#define POINT_2D_H

#include <boost/mpl/int.hpp>

#include <boost/geometry/core/cs.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/multi/geometries/multi_point.hpp> 

#include <visilibity.hpp>

#include <QtCore/QDebug> 

namespace Geometry {

/** \brief Boost wrapper for the VisiLibity::Point class
 * This class provides the necessary glue to use VisiLibity::Points with 
 * boost::geometry as Point_2D fullfils the boost::geometry point concept
 * for a 2-dimensional point.
 */
class Point_2D : public VisiLibity::Point {
  
public:
  /** \brief Default (empty) constructor.
    * This constructor does not initialize any of the coordinate values.
    */
  Point_2D()
    :Point()
  {};
    
  /** \brief Constructor to a certain coordinate tuple.
    * Initialize the point with the given coordinate values.
    */
  Point_2D(const double& x, const double& y)
    :Point(x,y)
  {};
  
  /** \brief Copy constructor. */
  Point_2D(const Point_2D& other)
    :Point_2D(other.x(), other.y())
  {};
  
  /** \brief Destructor. */
  ~Point_2D() = default;
  
  /** \brief Assignment operator. */
  Point_2D& operator=(const Point_2D& rhs)
  {
    if(this == &rhs) { return *this; };
    x(rhs.x());
    y(rhs.y());
    
    return *this;
  };
    
    
  
  /** \brief Boost-conform access function for getting coordinate values. */
  template< std::size_t K >
  inline const double& get() const
  { 
    BOOST_STATIC_ASSERT(K < 2);
    return K==0 ? x_ : y_ ;
  };
  
  /** \brief Boost-conform mutator function for setting coordinate values. */
  template< std::size_t K >
  inline void set(const double& value)
  { 
    BOOST_STATIC_ASSERT(K < 2);
    K==0 ? (x_=value) : (y_=value) ;
  };
  
  
  /** \brief Get the x-coordinate value. */
  inline double const& x() const
  { return this->get<0>(); }

  /** \brief Get the y-coordinate value. */
  inline double const& y() const
  { return this->get<1>(); }
  
  /** \brief Set the x-coordinate value. */
  inline void x(const double& value)
  { this->set<0>(value); }
  
  /** \brief Set the y-coordinate value. */
  inline void y(const double& value)
  { this->set<1>(value); }
  
};


using MultiPoint_2D = boost::geometry::model::multi_point<Point_2D>;

} // end namespace Geometry

//
// Adapt the Point_2D to the boost::geometry point concept
// =======================================================
// This section provides the glue to properly make Point_2D work with boost
// by providing all the required traits and redirection functions.
//

namespace boost{ namespace geometry { namespace traits {
  
  template<>
  struct tag<Geometry::Point_2D>
  { typedef point_tag type; };
  
  template<>
  struct coordinate_type<Geometry::Point_2D>
  { typedef double type; };
  
  template<>
  struct coordinate_system<Geometry::Point_2D>
  { typedef boost::geometry::cs::cartesian type; };
  
  template<>
  struct dimension<Geometry::Point_2D>
    : boost::mpl::int_<2>{};
    
  template<std::size_t Dimension>
  struct access<Geometry::Point_2D, Dimension>
  {
    static inline double get(const Geometry::Point_2D& p)
    { return p.template get<Dimension>(); };
    
    static inline void set(Geometry::Point_2D& p, const double& value)
    { p.template set<Dimension>(value); };
  };
  
}}} // end namespave boost::geometry::traits



//############################################################################//
//                                                                            //
//  I/O Support                                                               //
//                                                                            //
//############################################################################//



inline std::ostream& operator<<(std::ostream& out, const Geometry::Point_2D& point )
{ return out << "(" << point.x() << ", " << point.y() <<")"; };

inline QDebug operator<<(QDebug dbg, const Geometry::Point_2D& point )
{
  dbg.nospace() << "(" << point.x() << ", " << point.y() <<")";
  return dbg.space();
};


#endif // POINT_2D_H
