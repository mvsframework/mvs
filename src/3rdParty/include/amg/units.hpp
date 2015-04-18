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


#ifndef UNITS_H
#define UNITS_H

namespace AMG {
  
/** \brief The namespace for all units used in \c AMG 
 * This (headers only) namespace contains empty structs used in templates to 
 * indicate the units of the in-parameters and some related conversion 
 * functions.
 */
namespace Units { 
  
  //
  // angular
  //
  
  /** \brief An empty struct used in templates to indicate the use of degrees*/
  struct degree {};
  
  /** \brief An empty struct used in templates to indicate the use of radians*/
  struct radian {};

  //
  // linear
  //
  /** \brief An empty struct used in templates to indicate the use of meters*/
  struct meter {};
  
  /** \brief An empty struct used in templates to indicate the use of feet*/
  struct foot {};
  
  //
  // Conversions
  //
  /** \brief Convert a degree value to a radian value
   * The internal precision is double.
   */
  template<typename T>
  inline T degree2radian(const T& degreeValue)
  {
    return T(degreeValue*M_PIl/180.0);
  };
  
  /** \brief Convert a radian value to a degree value
   * The internal precision is double.
   */
  template<typename T>
  inline T radian2degree(const T& radianValue)
  {
    return T(radianValue*180.0*M_1_PIl);
  }; 
  
  /** \brief Convert a foot value to a meter value
   * The internal precision is double.
   */
  template<typename T>
  inline T feet2meter(const T& footValue)
  {
    return T(0.3048 * footValue);
  };
  
  /** \brief Convert a meter value to a foot value
   * The internal precision is double.
   */
  template<typename T>
  inline T meter2feet(const T& meterValue)
  {
    return T( meterValue/0.3048);
  };
  
  
} // end namespace Units  
} // end namespace AMG


#endif // UNITS