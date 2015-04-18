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

#ifndef SLOT_2D_H
#define SLOT_2D_H

#include <boost/geometry.hpp>
#include <boost/geometry/strategies/transform/matrix_transformers.hpp> 
#include <math.h> 

namespace Geometry {
  
enum class CapStyles {
   Flat ///< \brief Ends run directly through the points.
  ,Square ///< \brief Ends are offset by half the width,
  ,Circular ///< \brief Ends are a (half) circle centered at the points.
};  



/** \brief Make a slot with ends that pass through the points.
 * ~~~~~~~~
 * --------
 * |      |
 * x      x
 * |      |
 * --------
 * ~~~~~~~~
 * \tparam Ring The boost::geometry ring type used to hold the slot's shape
 */
template<typename Ring>
Ring makeFlatSlot(const typename boost::geometry::point_type<Ring>::type& p1
                 ,const typename boost::geometry::point_type<Ring>::type& p2 
                 ,const typename boost::geometry::coordinate_type<Ring>::type& width)
{
  BOOST_CONCEPT_ASSERT( (boost::geometry::concept::Ring< Ring >) );
  
  using namespace boost::geometry;
  
  using point_type = typename point_type<Ring>::type;
  using coordinate_type = typename coordinate_type<Ring>::type;
   
  // establish the direction of the slot 
  
  coordinate_type dx = ( get<0>(p2)-get<0>(p1) )/distance(p2,p1);
  coordinate_type dy = ( get<1>(p2)-get<1>(p1) )/distance(p2,p1);
  
  // compute the 4 corners of the rectangular flat slot

  point_type corner11; 
  set<0>(corner11,get<0>(p1) - dy * width/2);
  set<1>(corner11,get<1>(p1) + dx * width/2);

  point_type corner12; 
  set<0>(corner12,get<0>(p1) + dy * width/2);
  set<1>(corner12,get<1>(p1) - dx * width/2);
  
  point_type corner21; 
  set<0>(corner21,get<0>(p2) - dy * width/2);
  set<1>(corner21,get<1>(p2) + dx * width/2);

  point_type corner22; 
  set<0>(corner22,get<0>(p2) + dy * width/2);
  set<1>(corner22,get<1>(p2) - dx * width/2);

  // build the rectangle
  
  Ring ring; 
  
  append(ring, corner11);
  append(ring, corner12);
  append(ring, corner22);
  append(ring, corner21);
  
  correct( ring );
  
  return ring;
};
/** \brief Make a slot with square ends that extend beyond the points.
 * ~~~~~~~~
 * --------------
 * |            |
 * |   x    x   |
 * |            |
 * --------------
 * ~~~~~~~~
 * \tparam Ring The boost::geometry ring type used to hold the slot's shape
 */
template<typename Ring>
Ring makeSquareSlot(const typename boost::geometry::point_type<Ring>::type& p1
                 ,const typename boost::geometry::point_type<Ring>::type& p2 
                 ,const typename boost::geometry::coordinate_type<Ring>::type& width)
{
  BOOST_CONCEPT_ASSERT( (boost::geometry::concept::Ring< Ring >) );
  
  using namespace boost::geometry;
  
  using point_type = typename point_type<Ring>::type;
  using coordinate_type = typename coordinate_type<Ring>::type;
   
  // establish the direction of the slot 
  
  coordinate_type dx = ( get<0>(p2)-get<0>(p1) )/distance(p2,p1);
  coordinate_type dy = ( get<1>(p2)-get<1>(p1) )/distance(p2,p1);
  
  // compute the 4 corners of the rectangular flat slot

  point_type corner11; 
  set<0>(corner11,get<0>(p1) + (-dy-dx) * width/2);
  set<1>(corner11,get<1>(p1) + (+dx-dy) * width/2);

  point_type corner12; 
  set<0>(corner12,get<0>(p1) + (+dy-dx) * width/2);
  set<1>(corner12,get<1>(p1) + (-dx-dy) * width/2);
  
  point_type corner21; 
  set<0>(corner21,get<0>(p2) + (-dy+dx) * width/2);
  set<1>(corner21,get<1>(p2) + (+dx+dy) * width/2);

  point_type corner22; 
  set<0>(corner22,get<0>(p2) + (+dy+dx) * width/2);
  set<1>(corner22,get<1>(p2) + (-dx+dy) * width/2);

  // build the rectangle
  
  Ring ring; 
  
  append(ring, corner11);
  append(ring, corner12);
  append(ring, corner22);
  append(ring, corner21);
  
  correct( ring );
  
  return ring;
};


/** \brief Make a slot with circular ends centered at the points.
 * ~~~~~~~~
 *    --------
 *  /          \
 * |   x    x   |
 *  \          /
 *    --------
 * ~~~~~~~~
 * \tparam Ring The boost::geometry ring type used to hold the slot's shape
 * \tparam N The number of points used to approximate the half circles on each 
 *  side. (The complete slow will have 2N+4 corners.)
 */
template<typename Ring 
        ,unsigned int N = 10 >
Ring makeCircularSlot(const typename boost::geometry::point_type<Ring>::type& p1
                     ,const typename boost::geometry::point_type<Ring>::type& p2 
                     ,const typename boost::geometry::coordinate_type<Ring>::type& width)
{
  BOOST_CONCEPT_ASSERT( (boost::geometry::concept::Ring< Ring >) );
  BOOST_STATIC_ASSERT( N>0 ); // you need more than no points ;)
  
  using namespace boost::geometry;
  
  using point_type = typename point_type<Ring>::type;
  using coordinate_type = typename coordinate_type<Ring>::type;
   
  constexpr double pi2 =  std::atan(1)*2; // 0.5 PI == 90 deg
  constexpr double Delta = 2*pi2/(N+1); // fence counting: N=1 ->  o---x---o
  
  coordinate_type const radius = width/2.0;

  // establish the direction of the slot 
  
  coordinate_type dx = ( get<0>(p2)-get<0>(p1) )/distance(p2,p1);
  coordinate_type dy = ( get<1>(p2)-get<1>(p1) )/distance(p2,p1);
  // the angle wrt. to the x-axis from p1 to p2
  double angle = std::atan2(dy,dx); // range [-pi, pi]
  
  angle += pi2; // rotate 90 deg ccw
  angle -= Delta; // rotate Delta cw to offset for the first addition in the for loop
  
  Ring ring;

  // create the half circle around p1
  for( unsigned int i = 0; i<N+2 ; ++i)
  {
    angle += Delta; // rotate Delta ccw
    
    // counter clockwise circle, starting at (0,-1)...
    coordinate_type rx = radius*std::cos(angle);
    coordinate_type ry = radius*std::sin(angle);
    
    point_type pc( get<0>(p1)+rx, get<1>(p1)+ry ); // a point on the circle
    
    append(ring,pc);
  }
  
  angle -= Delta; // rotate Delta cw to offset for the first addition in the for loop
  
  // create the half circle around p2
  for( unsigned int i = 0; i<N+2 ; ++i)
  {
    angle += Delta; // rotate Delta ccw
    
    // counter clockwise circle, starting at (0,-1)...
    coordinate_type rx = radius*std::cos(angle);
    coordinate_type ry = radius*std::sin(angle);
    
    point_type pc( get<0>(p2)+rx, get<1>(p2)+ry ); // a point on the circle
    
    append(ring,pc);
  }

  correct(ring);
  
  return ring;  
};

  
} // end namespace Geometry
#endif // SLOT_2D_H