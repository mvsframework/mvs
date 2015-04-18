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

#ifndef WAYPOINT_H
#define WAYPOINT_H

#include <amg.hpp>
#include "IDL/data.h"

/** \brief An interface for the underlying IDL waypoint.
 * As there is a need to communicate waypoints, there has to be a rather simple
 * waypoint structure in IDL : IDL::Data::Waypoint.
 * However, as direct interaction with data members is discouraged, this 
 * wrapper class hides away that underlying simple structure (through private
 * inheritance) and provides the necessary accessors and mutators.
 */
class Waypoint : private IDL::Data::Waypoint {
public:
  /** \brief Default constructor to (0,0,0). */
  Waypoint(double const& x=0,double const& y=0,double const& z=0);
  
  /** \brief Constructor.
   * \overload
   */
  Waypoint(AMG::Vector const & position);
  
  /** \brief Copy-constructor. */
  Waypoint(IDL::Data::Waypoint const & waypoint);
  
  virtual ~Waypoint() = default;
  
  /** \brief The Euclidian distance between the Waypoints A and B. */
  static double distance(Waypoint const & a, Waypoint const & b);
  
  
  IDL::Data::Position_Datum position() const;
  void setPosition(IDL::Data::Position_Datum const& position);
  void setPosition(double const& x,double const& y,double const& z);
  void setPosition(AMG::Vector const & position);
  
  /** \brief Copy assignment operator. */
  Waypoint& operator=(Waypoint const & other)
  {
   if( this != &other ) // self-assingment check
   { setPosition(other.position()); }
   
   return *this;   
  }
  
  /** \brief Conversion operator to IDL::Data::Waypoint. */
  explicit operator IDL::Data::Waypoint() const
  {
    IDL::Data::Waypoint idlWp;
    idlWp.position = position();
    return idlWp;
  };
};


#endif // WAYPOINT_H
