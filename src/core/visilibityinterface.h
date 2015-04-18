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

#ifndef VISILIBITYINTERFACE_H
#define VISILIBITYINTERFACE_H

#include <visilibity.hpp>
#include <boost/geometry.hpp>

#include "Geometry/geometry.h"

namespace VisilibityInterface
{

  /** \brief Numerical epsilon for VisiLibity. 
   * \sa VisiLibity::Environment.is_valid()
   */
  constexpr static double epsilon = 1e-9;  


  
  template<typename Point>
  VisiLibity::Point makeVlPoint(const Point& p)
  {
    BOOST_CONCEPT_ASSERT( (boost::geometry::concept::Point< Point >) );
    return VisiLibity::Point( boost::geometry::get<0>(p) ,
                              boost::geometry::get<1>(p) );
  };
  
  template<typename Ring>
  VisiLibity::Polygon makeVlPolygon(const Ring& r)
  {
    BOOST_CONCEPT_ASSERT( (boost::geometry::concept::Ring< Ring >) );
    
    VisiLibity::Polygon polygon;
    
    auto numBoostPoints = boost::geometry::num_points(r);
    Q_ASSERT( numBoostPoints >=4 ); 
    
    if( boost::geometry::closure<Ring>::value == boost::geometry::closed )
    { // closed boost geometries always repeat the first point as the last one
      polygon.reserve( numBoostPoints-1 );
      for( auto point = r.begin(); point != --(r.end()); ++point)
      {
        polygon.push_back( makeVlPoint(*point) ); 
      }
    }
    else
    { // All VisiLibity Polygons are implicitely close, hence this will convert
      // from an open boost ring to a closed VisiLibity polygon
      polygon.reserve( numBoostPoints );
      for( auto point = r.begin(); point != r.end(); ++point)
      {
        polygon.push_back( makeVlPoint(*point) ); 
      }
    }
    
//     auto numVlPoints = polygon.n(); 
//     std::cout << "boost: " << numBoostPoints << ", vl : " << numVlPoints << std:: endl;
    
    return polygon;
  };
  
  template<typename Ring = Geometry::Polygon_2D::ring_type>
  Ring makeRing(const VisiLibity::Polygon& vlpoly) 
  {
//     BOOST_CONCEPT_ASSERT( (boost::geometry::concept::Ring< Ring >) );
    
    using point_type = typename boost::geometry::point_type<Ring>::type;
    
    Ring r;
    
    for( unsigned int i=0; i<vlpoly.n(); ++i )
    {
      point_type p(vlpoly[i].x(), vlpoly[i].y());
      boost::geometry::append(r,p);
    }
    
    boost::geometry::correct(r);
    return r;
  };
  
  
  /** \brief Create a VisiLibity environment.
   * \pre  BOOST_CONCEPT_ASSERT( (concept::Polygon\<Polygon\>) );
   * \tparam Polygon The polygon type. Must fullfill the Boost.Geometry polygon concept.
   * \param[in] environment A polygon representing the environment (i.e. the 
   *  "interior") for visibility computations. Can contain holes.
   */
  template<typename Polygon>
  VisiLibity::Environment createEnvironment(const Polygon& poly)
  {
    namespace bg=boost::geometry;
    
    // check that we can treat the environment as a boost::geometry polygon
    BOOST_CONCEPT_ASSERT( (boost::geometry::concept::Polygon< Polygon >) );
    
//   using Polygon = Geometry::Polygon_2D;
    
    using ring_type = typename boost::geometry::ring_type<Polygon>::type;
//     using point_type = typename boost::geometry::point_type<Polygon>::type;
    
    //
    // create a VisiLibity polygon representing the outer boundary.
    // 
    auto environment = VisiLibity::Environment(); // clear the environment
    
    ring_type outer = poly.outer(); // should be ring_type
    environment.set_outer_boundary(makeVlPolygon(outer));
    
    auto inner_list = poly.inners();
    for( ring_type& inner : inner_list )
    {
      environment.add_hole(makeVlPolygon(inner)); 
    }
    
    environment.enforce_standard_form();  
//   Q_ASSERT( environment.is_valid(epsilon) ); //NOTE: This takes 5 times as long as the actual computation of the visPoly!!!
    
    return environment;
  };
   
  
  
  /** \brief Compute the visibility polygon for point p in polygon poly.
   * 
   * \internal
   * When VisiLibity computes a visibility polygon (which could take a 
   * while), the constructor gets passed a reference to the environement:
   * \code
Visibility_Polygon(const Point& observer,
                   const Environment& environment_temp,
                   double epsilon=0.0);
     \endcode   
   * This could lead to the use of effectively shared data (via the reference).
   * Hence the proposed workround is to construct a new environment/point
   * combination for every call to the Visibility_Polygon constructor.
   * 
   * \tparam Ring The ring type. Must fullfill the Boost.Geometry ring concept.
   * \tparam Point The point type. Must fullfill the Boost.Geometry point concept.
   * \tparam Polygon The polygon type. Must fullfill the Boost.Geometry polygon concept.
   * \param[in] p The point to compute the visibility polygon for, i.e. the 
   *  "guard" location.
   * \param[in] poly The polygon representing the environment the guard is 
   *  placed in.
   * \return The visibility polygon for a guard at p in the poly environment.
   */
  template<typename Ring = Geometry::Polygon_2D::ring_type,
           typename Point = Geometry::Point_2D,
           typename Polygon = Geometry::Polygon_2D>
   Ring computeVisibilityPolygon(const Point& p, const Polygon& poly)
  {
    // check that we can treat the environment as a boost::geometry polygon
    BOOST_CONCEPT_ASSERT( (boost::geometry::concept::Ring< Ring >) );
    BOOST_CONCEPT_ASSERT( (boost::geometry::concept::Point< Point >) );
    BOOST_CONCEPT_ASSERT( (boost::geometry::concept::Polygon< Polygon >) );
      
    VisiLibity::Point guard(makeVlPoint(p));
//   Q_ASSERT( m_environment.is_valid(epsilon) ); //NOTE: This takes 5 times as long as the actual computation of the visPoly!!!
    
    //NOTE: Watch the environment for thread safety!
    auto environment = createEnvironment(poly);
    
    guard.snap_to_boundary_of(environment, epsilon);
    guard.snap_to_vertices_of(environment,epsilon);
    if( guard.in(environment, epsilon) )
    { 
      VisiLibity::Visibility_Polygon visPoly(guard,environment,epsilon); 
      return makeRing<Ring>(visPoly);
    }
    else
    {
//       std::cerr << "VisilibityInterface: guard.in(environment, epsilon) FAILED." << std::endl;
      return Ring();
    }
    
  };
 
 
  
} // end namespace VisilibityInterface




#endif // VISILIBITYINTERFACE_H
