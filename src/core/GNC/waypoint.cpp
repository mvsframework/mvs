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

#include "GNC/waypoint.h"
#include <cmath>

Waypoint::Waypoint( const double& x, const double& y, const double& z )
{
  setPosition(x,y,z);
}

Waypoint::Waypoint ( const IDL::Data::Waypoint& waypoint )
{
  setPosition(waypoint.position);
}

Waypoint::Waypoint ( const AMG::Vector& position )
{
  setPosition(position);
}

double Waypoint::distance ( const Waypoint& a, const Waypoint& b )
{
 double dx = a.position().x - b.position().x;
 double dy = a.position().y - b.position().y;
 double dz = a.position().z - b.position().z;
 return std::sqrt(dx*dx+dy*dy+dz*dz);  
}



IDL::Data::Position_Datum Waypoint::position() const
{
  return IDL::Data::Waypoint::position;
}

void Waypoint::setPosition ( const IDL::Data::Position_Datum& position )
{
  IDL::Data::Waypoint::position = position;
}

void Waypoint::setPosition ( const double& x, const double& y, const double& z )
{
  IDL::Data::Waypoint::position.x = x;
  IDL::Data::Waypoint::position.y = y;
  IDL::Data::Waypoint::position.z = z;
}

void Waypoint::setPosition ( const AMG::Vector& position )
{
  auto coords = position.absoluteCoordsIn(AMG::CoSy::getDatumFrame());
  setPosition( coords[0],coords[1],coords[2]);
}

