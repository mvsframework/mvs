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

#include "GNC/flightplan.h"
#include "Geometry/geometry.h"

#include <IDL/dataSupport.h>
#include <IDL/idlsupport.h>

#include <QtCore/qglobal.h>
#include <QtCore/QCryptographicHash>

Flightplan::Flightplan(const IDL::Data::Flightplan& idlPlan)
{
  for( int i=0; i<idlPlan.waypoints.length(); ++i)
  { append(idlPlan.waypoints[i]); }
  
  hash = IDL::toQByteArray(idlPlan.hash);
  
  IDL::Data::Md5Hash newHash = hashWaypoints(idlPlan.waypoints);
  Q_ASSERT( hash == IDL::toQByteArray(newHash) ); 
  
  
}


void Flightplan::append ( const Waypoint& waypoint )
{
  waypoints.append(waypoint);
}

void Flightplan::append ( const QList< Waypoint >& waypoints )
{
  this->waypoints.append(waypoints);
}

IDL::Data::Flightplan Flightplan::convertToIdlFlightplan(const Flightplan& gncPlan)
{
  IDL::Data::Flightplan idlPlan;
  
  idlPlan.waypoints.ensure_length(gncPlan.waypoints.size(),gncPlan.waypoints.size());
  
  int i = 0;
  for( const Waypoint & gncWp : gncPlan.waypoints )
  {
    IDL::Data::Waypoint idlWp;
    idlWp.position = gncWp.position(); 
    idlPlan.waypoints[i++] = idlWp;
  }
  
  idlPlan.hash = hashWaypoints(idlPlan.waypoints);
    
  return idlPlan;
}

IDL::Data::Md5Hash Flightplan::hashWaypoints(const IDL::Data::WaypointSeq& wpSequence)
{

  char* start = reinterpret_cast<char*>(wpSequence.get_contiguous_buffer());
  int size = wpSequence.length()* sizeof(IDL::Data::Waypoint);
    
  QCryptographicHash md5(QCryptographicHash::Md5);
  md5.addData( start, size);
  
  QByteArray ba = md5.result();
    
  return IDL::toMd5Hash(ba);
}

bool Flightplan::eliminateDoubleBacks(Flightplan& fp, const double& areaThreshold)
{ 
  bool flightplanAltered = false;
    
  auto computeTriangleArea_2d_xy = [=](Waypoint const& a, Waypoint const& b, Waypoint const& c) -> double
  {
    Geometry::Polygon_2D::ring_type triangle;
    Geometry::Point_2D corner;
    
    // Waypoint a
    corner.x(a.position().x);
    corner.y(a.position().y);
    triangle.push_back(corner);

    // Waypoint b
    corner.x(b.position().x);
    corner.y(b.position().y);
    triangle.push_back(corner);

    // Waypoint c
    corner.x(b.position().x);
    corner.y(b.position().y);
    triangle.push_back(corner);
    
    return boost::geometry::area(triangle);
  };
  
  
  
  // clear the front
  if( fp.waypoints.size() >=3 )
  {
    double area = computeTriangleArea_2d_xy(fp.waypoints[0], fp.waypoints[1], fp.waypoints[2]);
    
    if( area < areaThreshold )
    {
      fp.waypoints.takeAt(1);
      flightplanAltered = true;
    }
  }
  
  // clear the end
  if( fp.waypoints.size() >=4 )
  {
    const auto n = fp.waypoints.size()-1;
    double area = computeTriangleArea_2d_xy(fp.waypoints[n-2], fp.waypoints[n-1], fp.waypoints[n]);
    
    if( area < areaThreshold )
    {
      fp.waypoints.takeAt(n-2);
      flightplanAltered = true;
    }
  }
  
  return flightplanAltered;
}