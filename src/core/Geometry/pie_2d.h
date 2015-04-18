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

#ifndef PIE_2D_H
#define PIE_2D_H

#include <boost/geometry.hpp>
#include <boost/geometry/strategies/transform/matrix_transformers.hpp> 
#include <cmath>

namespace Geometry {

/** \brief Create a polygonal approximation of a circle. 
 * 
 * \pre -\Pi <= orientation < \Pi 
 * \pre 0 < openingAngle < 2*\Pi
 * 
 * \tparam Ring A type fullfilling the boost::ring concept
 * \tparam N The number of corners of the underlying circle.
 * \tparam DegreeOrRadian (optional, defaults to degree) boost::geometry::degree
 *  or boost::geometry::radian
 * 
 * \param[in] center The center of the circle
 * \param[in] radius The radius of the circel (in units matching the 
 *  coordinates).
 * \param[in] orientation The angle of the axis of symmetry wrt. the x-axis in
 *  [DegreeOrRadian].
 * \param[in] openingAngle The total opening angle of the pie section in 
 *  [DegreeOrRadian]. The sections radial bounds are at orientation +/- 0.5 * 
 *  openingAngle.
 * \return A N-corner-circle-based pie-section approximation.
 */
template<typename Ring
        ,int N
        ,typename DegreeOrRadian = boost::geometry::degree
        >
Ring makePie(const typename boost::geometry::point_type<Ring>::type& center
            ,const typename boost::geometry::coordinate_type<Ring>::type& radius
            ,double const & orientation, double const & openingAngle
            )
{
  using point_type = typename boost::geometry::point_type<Ring>::type;
  
  BOOST_CONCEPT_ASSERT( (boost::geometry::concept::Ring< Ring >) );
  BOOST_STATIC_ASSERT( N>2 ); // you need more than 2 points
  
  constexpr double pi =  std::atan(1)*4;
  constexpr double Delta = (2.0*pi)/(N);
  
  
  
  namespace detail = boost::geometry::strategy::transform::detail;
  {
    double openingAngle_rad  = detail::as_radian<DegreeOrRadian>::get(openingAngle);
    Q_ASSERT( (0 < openingAngle_rad ) and (openingAngle_rad < 2*pi) );
  }
  double startAngle = detail::as_radian<DegreeOrRadian>::get(-0.5*openingAngle);
  double stopAngle  = detail::as_radian<DegreeOrRadian>::get(+0.5*openingAngle);

  auto origin = boost::geometry::make_zero<point_type>();

  Ring pie_0; 
  
  { // add the origin and the first corner
    boost::geometry::append( pie_0, origin);
  
    point_type point(radius*std::cos(startAngle),
                      radius*std::sin(startAngle)); 
    boost::geometry::append( pie_0,point);
  }
 
  { // add the circle element corners
    int i_start = std::trunc(startAngle/Delta);
    int i_end   = std::trunc(stopAngle/Delta);
    
    Q_ASSERT(-i_start == i_end);
  
    double angle = i_start * Delta;
    for(int i = i_start; i<i_end ; ++i)
    { // 
      point_type point(radius*std::cos(angle),
                        radius*std::sin(angle)); 
      boost::geometry::append( pie_0,point);
      angle += Delta;
    }
  }
  
  { // add the second corner and the the origin again

    point_type point(radius*std::cos(stopAngle),
                     radius*std::sin(stopAngle)); 
    boost::geometry::append( pie_0,point);
    boost::geometry::append( pie_0,origin);
  }
  
  boost::geometry::correct( pie_0 );
  
  // rotate the pie sector into the correct orientation and translate it tttrasrro 
  // the location
    
  auto const dim = boost::geometry::traits::dimension<point_type>::value;
  
  boost::geometry::strategy::transform::rotate_transformer<DegreeOrRadian,double,dim,dim>
    rotate(-1*orientation);//NOTE : the -1 is on purpose. Compare https://github.com/boost-cmake/geometry/blob/master/test/strategies/transformer.cpp but don't asky me why it is that way.
  boost::geometry::strategy::transform::translate_transformer<double,dim,dim>
    translate(boost::geometry::get<0>(center),
              boost::geometry::get<1>(center));
  
  Ring pie_1;
  boost::geometry::transform( pie_0, pie_1, rotate);
  
  Ring pie_2;
  boost::geometry::transform( pie_1, pie_2,translate);
  
  
  
  return pie_2;
};

} // end namespace Geometry

#endif // PIE_2D_H
