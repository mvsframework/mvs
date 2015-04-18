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

#ifndef CIRCLE_2D_H
#define CIRCLE_2D_H

#include <boost/geometry.hpp>
#include <cmath>

namespace Geometry {

/** \brief Create a polygonal approximation of a circle. 
 *
 * \tparam Ring A type fullfilling the boost::ring concept
 * \tparam N The number of corners of the returned ring.
 * \param[in] center The center of the circle
 * \param[in] radius The radius of the circel (in units matching the 
 *  coordinates).
 * \return A N-corner Ring element circle approximation.
 */
template<typename Ring,int N>
Ring makeCircle(const typename boost::geometry::point_type<Ring>::type& center,
                const typename boost::geometry::coordinate_type<Ring>::type& radius)
{
  using point_type = typename boost::geometry::point_type<Ring>::type;
  
  BOOST_CONCEPT_ASSERT( (boost::geometry::concept::Ring< Ring >) );
  BOOST_STATIC_ASSERT( N>2 ); // you need more than 2 points
  
  constexpr double pi =  std::atan(1)*4;
  constexpr double delta = (2.0*pi)/(N);
  
  Ring r;
  
  double angle = 0;
  
  for(int i=0; i<N; ++i)
  {
    namespace bg = boost::geometry;
    
    point_type point;
    point_type offset(radius*std::sin(angle),
                      radius*std::cos(angle)); 

    boost::geometry::assign(point, center);
    boost::geometry::add_point(point,offset);
    boost::geometry::append(r,point);
    
    angle += delta;
  }
  
  boost::geometry::correct(r);
  
  return r;
};

} // end namespace Geometry

#endif // CIRCLE_2D_H
