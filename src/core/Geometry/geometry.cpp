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

#include "Geometry/geometry.h"

#include <boost/geometry/algorithms/distance.hpp>
#include <boost/geometry/algorithms/comparable_distance.hpp> 
#include <boost/geometry/strategies/cartesian/distance_projected_point.hpp>
#include <boost/geometry/io/wkt/wkt.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/range/iterator_range.hpp>

#include <boost/numeric/conversion/bounds.hpp>

#include <assert.h>
// #include <osapi/osapi_ntptime.h>
#include <amg.hpp>

#include "XSD/xmltools.h"

#include <QtCore/qglobal.h>
#include <QtCore/QDebug>

namespace Geometry {

  
Coordinates_ECEF geodetic2ecef(const Coordinates_Geodetic& geoCoords)
{
  Q_ASSERT(AMG::CoSy::getECEF()!=nullptr );
  
  using AMG::GIS::geodetic2ecef;
  using AMG::Units::degree2radian;
  
  auto lambda = get<0>(geoCoords);
  auto phi = get<1>(geoCoords);
  auto h = get<2>(geoCoords);

  double x,y,z;
  geodetic2ecef(degree2radian(lambda),degree2radian(phi),h, x,y,z);
  
//   // debug information
//   Coordinates_ECEF ecefCoords(x,y,z);
//   std::cout << geoCoords << " -> " << ecefCoords << std::endl;
  
  return Coordinates_ECEF(x,y,z);
}
  
void geodetic2ecef(const Coordinates_Geodetic& geoCoords, Coordinates_ECEF& ecefCoords)
{
  ecefCoords = geodetic2ecef(geoCoords);
}
  

  
Coordinates_Datum ecef2datum(const Coordinates_ECEF& ecefCoords)
{
  Q_ASSERT(AMG::CoSy::getDatumFrame()!=nullptr);
  
  AMG::Vector ecefVector(get<0>(ecefCoords),
                         get<1>(ecefCoords),
                         get<2>(ecefCoords),
                         AMG::CoSy::getECEF() );
  
  AMG::CoordinateTupel datumTupel = ecefVector.absoluteCoordsIn(AMG::CoSy::getDatumFrame());
  
//   // debug output
//   Coordinates_Datum datumCoords(datumTupel(0),datumTupel(1),datumTupel(2));
//   std::cout << ecefCoords << " -> " << datumCoords << std::endl;
  
  return Coordinates_Datum(datumTupel(0),  // x-coordinate
                           datumTupel(1),  // y-coordinate
                           datumTupel(2)); // z-coordinate`
  
}
  
  
Box_2D boundingBox ( const Polygon_2D& polygon )
{
  Box_2D box;
  boost::geometry::envelope(polygon,box);
  return box;
}
  
Point_2D closestPointOnLine(Point_2D const& p, Point_2D const& a, Point_2D const& b,
                            bool truncateAtA, bool truncateAtB )
{
 
  double dist_ab = boost::geometry::distance(a,b);
  
  // Missusing a point to store the unit "vector" from a to p.
  Point_2D n( (b.x()-a.x())/dist_ab ,
              (b.y()-a.y())/dist_ab );
 
  // a minus p
  Point_2D amp(a.x()-p.x(),a.y()-p.y());
  
  double ampDotN = amp.x()*n.x()+amp.y()*n.y();
 
  Point_2D closest( a.x() - ampDotN*n.x() ,
                    a.y() - ampDotN*n.y() );
  
  if( truncateAtA 
      and( (closest.x()-a.x())/n.x() < 0  or (closest.y()-a.y())/n.y() < 0 )
    )
  { 
//     qDebug() << "Truncation at A kicked in.";
    return a;
  }
  
  
  if( truncateAtB 
      and( (closest.x()-b.x())/n.x() > 0  or (closest.y()-b.y())/n.y() > 0 )
    )
  { 
//     qDebug() << "Truncation at B kicked in.";
    return b;
  }
    
  return closest;
}
  

Point_2D closestBoundaryPoint(const Point_2D& point, const Polygon_2D& polygon)
{
  return closestPointOnRing(point, polygon.outer()); ///< \todo Implement the case where point is inside a hole of the polygon
}

  
Point_2D closestPointOnRing(const Point_2D& point, const Polygon_2D::ring_type& ring)
{
  Q_ASSERT(ring.size() > 2);
  
    
  using namespace boost;
  using Ring = Polygon_2D::ring_type;
  
  double min_comparable_dist = numeric::bounds<double>::highest();
  Point_2D closestOnRing;
  
  Ring::const_iterator i_corner,i_before;
  for(  i_corner = ++ring.begin(); i_corner != ring.end(); ++i_corner)
  {
    i_before = i_corner-1;
   
    Point_2D closestOnLine = closestPointOnLine(point,*i_corner,*i_before,
                                      /*truncateAtA*/ true,
                                      /*truncateAtB*/ true);
    
    double distanceToLine = boost::geometry::comparable_distance(point, closestOnLine);
    
    if( distanceToLine < min_comparable_dist )
    {   
      closestOnRing = closestOnLine; 
      min_comparable_dist = distanceToLine;
    }
      
  }
  
  // NOTE: Rings are assumed to be closed (otherwise they would be LineStrings),
  // the question is simply how this is represented in the underlying collection
  // of points of the ring!
  if( geometry::closure< Ring >::value == geometry::open ) //  i.e. first_point != last_point, i.e. we have to manually wrap around.
  {
    Point_2D closestOnLine = closestPointOnLine(point,*(ring.end()-1),*ring.begin(),
                                      /*truncateAtA*/ true,
                                      /*truncateAtB*/ true);
    
    double distanceToLine = boost::geometry::comparable_distance(point, closestOnLine);
    
    if( distanceToLine < min_comparable_dist )
    {   
      closestOnRing = closestOnLine; 
      min_comparable_dist = distanceToLine;
    }
  };
   
  return closestOnRing;
}  


Point_2D kmlStringToPoint(const std::string& coordinateString, const std::string& coordinateSeperator)
{
  return kmlStringToPoint(QString(coordinateString.c_str()),
                          QString(coordinateSeperator.c_str()));
}

Point_2D kmlStringToPoint(const QString& coordinateString, const QString& coordinateSeperator)
{
  auto coordinateList = QString(coordinateString).split(coordinateSeperator);
  Point_2D point(coordinateList[0].toDouble(), // latitude in degrees
                 coordinateList[1].toDouble());// longitude in degrees
  return point;
}

/** \todo Incorporate the altitude reference in the KML node */
Coordinates_Geodetic kmlStringToPointCoordinate(const QString& coordinateString, const QString& coordinateSeperator)
{
  auto coordinateList = QString(coordinateString).split(coordinateSeperator);
  Coordinates_Geodetic coordTuple(coordinateList[0].toDouble(), // longitude in degrees
                                  coordinateList[1].toDouble(), // latitude in degrees
                                  coordinateList[2].toDouble());// altitude in meters
  return coordTuple;
}

Coordinates_Geodetic kmlStringToPointCoordinate(const std::string& coordinateString, const std::string& coordinateSeperator)
{
  return kmlStringToPointCoordinate(QString(coordinateString.c_str()),
                                    QString(coordinateSeperator.c_str()));
}




Polygon_2D kmlStringToPolygon(const std::string& coordinateString,
                              const std::string& pointSeperator,
                              const std::string& coordinateSeperator  )
{
  return kmlStringToPolygon(QString(coordinateString.c_str()),
                            QString(pointSeperator.c_str()),
                            QString(coordinateSeperator.c_str()) );
}

Polygon_2D kmlStringToPolygon(const QString& coordinatString, const QString& pointSeperator, const QString& coordinateSeperator)
{
  Polygon_2D polygon;
  
  auto pointStringList = QString(coordinatString).split(pointSeperator);
  QString pointCoordinateString;
  foreach( pointCoordinateString, pointStringList)
  {
    boost::geometry::append(polygon, kmlStringToPoint(pointCoordinateString,coordinateSeperator));
  }
  
  boost::geometry::correct(polygon);
  return polygon;
  
}

  
Polygon_2D::ring_type getBoostEqualSimplified(Polygon_2D::ring_type const& original
                                             ,double const& zeroAreaThreshold )
{
  using Ring = Polygon_2D::ring_type;
  
  Ring simplified = original;
  boost::geometry::correct(simplified);
  
  bool redoLoopSearch = false;
  
  do 
  {
    redoLoopSearch = false;
    
    auto i_vertex_end = simplified.end();
    int iv_count = 0;
    for( auto i_vertex = simplified.begin(); i_vertex != i_vertex_end; ++i_vertex )
    {
      int ic_count = iv_count+1;
      for( auto i_compare = i_vertex+1; i_compare != i_vertex_end; ++i_compare)
      {
        if(*i_vertex == *i_compare)
        {

          
          // create a new ring comprised of all the vertices from i_vertex to
          // i_compare
          auto loopVertices = boost::make_iterator_range(i_vertex,i_compare+1);
          Ring loop;
          boost::geometry::append(loop,loopVertices);
          
// DEBUG          
//           std::cout << "Found a loop: vertex " << iv_count 
//             << " is equal to vertex " << ic_count 
//             << " (total #vertices: "<< simplified.size() << ")" << std::endl; 
//           std::cout << "The loop has " << loopVertices.size() 
//             << " vertices which span an area of " << boost::geometry::area(loop) << std::endl;
//           if(i_vertex == simplified.begin() and i_compare== simplified.end()-1)
//           { std::cout << "(But that might be the complete ring.)"<<std::endl;}
            

          if( boost::geometry::area(loop) <= zeroAreaThreshold ) 
          {
            // if the area of that loop is zero(-ish), remove it from simplified
            // and restart the process  
            
            
            Ring temp;
            boost::geometry::append(temp, boost::make_iterator_range(simplified.begin(), i_vertex+1) );
            boost::geometry::append(temp, boost::make_iterator_range(i_compare+1,simplified.end())   );
            boost::geometry::correct(temp);
            
            simplified = temp;
//             std::cout << "Restarting loop detection..." << std::endl;
            redoLoopSearch = true;
            break;
          }      
          else if( loopVertices.size() < simplified.size() ) // this check ensures that 'complete ring' loops are skipped
          {
            // check if the loop made from the vertices _not_ in the detected 
            // loop has a zero-ish area
            
            Ring temp;
            boost::geometry::append(temp, boost::make_iterator_range(i_compare, simplified.end())   );
            boost::geometry::append(temp, boost::make_iterator_range(simplified.begin(),i_vertex+1) );
            boost::geometry::correct(temp);
            
            if( boost::geometry::area(temp) < zeroAreaThreshold )
            {
              simplified = loop;
//               std::cout << "Restarting loop detection..." << std::endl;
              redoLoopSearch = true;
              break;
            }
          }
        }
      
        ++ic_count;
      }
      if( redoLoopSearch )
      { break; }
      iv_count++;
    }
  }while(redoLoopSearch);
   
    
  return simplified;  
}

  

//############################################################################//
//                                                                            //
//  I/O Support                                                               //
//                                                                            //
//############################################################################//

QString toQString(Geometry::Coordinates_Geodetic const & coords)
{
  return QString() + "("
    + boost::geometry::get<0>(coords) + "°, "
    + boost::geometry::get<1>(coords) + "°, "
    + boost::geometry::get<2>(coords) + " m)";
}

QString toQString(Geometry::Coordinates_ECEF const & coords)
{
  return QString() + "("
    + boost::geometry::get<0>(coords) + " m, "
    + boost::geometry::get<1>(coords) + " m, "
    + boost::geometry::get<2>(coords) + " m)";
}

} // end namespace Geometry