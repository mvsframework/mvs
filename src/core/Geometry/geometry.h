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

#ifndef GEOMETRY_H
#define GEOMETRY_H

#include <iostream>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/polygon.hpp>


#include <QtCore/QString>
#include <QtCore/QDebug>

#include "Geometry/point_2d.h"
#include "Geometry/circle_2d.h"
#include "Geometry/pie_2d.h"



namespace Geometry {
  
  using namespace boost::geometry;
  
  /** \brief A coordainte typle in the cartesian ECEF coordinate system
   * get<0>(coordinate) = x-coordinate [in meter]
   * get<1>(coordinate) = y-coordinate [in meter]
   * get<2>(coordinate) = z-coordinate [in meter]
   */
  using Coordinates_ECEF = model::point<double,3,cs::cartesian>;

  /** \brief A coordainte typle in the datum coordinate system
   * get<0>(coordinate) = x-coordinate [in meter]
   * get<1>(coordinate) = y-coordinate [in meter]
   * get<2>(coordinate) = z-coordinate [in meter]
   * 
   */
  using Coordinates_Datum = model::point<double,3,cs::cartesian>;
  
  /** \brief A coordinate tuple in the WGS84 geographic coordinate system
   * get<0>(coordinate) = Longitude \lambda [in degree]
   * get<1>(coordinate) = Latitude \phi [in degree]
   * get<2>(coordinate) = Altitude \h [in meter]
   */
  using Coordinates_Geodetic = model::point<double,3,cs::geographic<degree>>;

  
  /** \brief Convert geodetic coordinats to ECEF coordinates
   * \pre AMG::CoSy::getECEF() != nullptr
   * \param[in] geoCoords The geodetic coordiante tuple {\lambda, \phi, h} in 
   * degrees and meter.
   * \return The corresponding ECEF coordinate tuple in meter.
   */
  Coordinates_ECEF geodetic2ecef(const Coordinates_Geodetic& geoCoords);
  
  /** \brief Convert geodetic coordinats to ECEF coordinates
   * \pre AMG::CoSy::getECEF() != nullptr
   * \param[in] geoCoords The geodetic coordiante tuple {\lambda, \phi, h} in 
   * degrees and meter
   * \param[out] ecefCoords The corresponding ECEF coordinate tuple in meter.
   */
  void geodetic2ecef(const Coordinates_Geodetic& geoCoords,
                     Coordinates_ECEF& ecefCoords);
  
  /** \brief Convert ECEF coordinates to Datum frame coordinates
   * \pre AMG::CoSy::getDatum() != nullptr
   * \param[in] ecefCoords The ECEF coordinate tuple in meter.
   * \return The corresponding coordinate tupel in the Datum frame in meter.
   */
  Coordinates_Datum ecef2datum(const Coordinates_ECEF& ecefCoords);
  
  /** \brief Convert ECEF coordinates to Datum frame coordinates
   * \pre AMG::CoSy::getDatumFrame() != nullptr
   * \param[in] ecefCoords The ECEF coordinate tuple in meter.
   * \param[out] datumCoords The coordinate tupel in the Datum frame in meter.
   */
  void ecef2datum(const Coordinates_ECEF& ecefCoords,
                  Coordinates_Datum& datumCoords );
 
  /** \brief Create a geodetic coordiante typle from a string of coordinates.
   * Coordinate Mapping:
   * * Longitude -> X [deg]
   * * Latitude  -> Y [deg]
   * * Altitude  -> Z [m]
   * \sa Coordinates_Geodetic
   *  
   * \note The order of the coordinates is not altered and Longitude, Latitude,
   * Altitude is assumed.
   * \param[in] coordinateString A string with coordinate information.
   * \param[in] coordinateSeperator (optional) The sequence used to separate the 
   *  individual coorinates.
   * \return The created coordinate tuple instance.
   */
  Coordinates_Geodetic kmlStringToPointCoordinate(const QString& coordinateString,
                            const QString& coordinateSeperator = ",");

  /** \brief Create a geodetic coordiante typle from a string of coordinates.
   * \overload
   * Coordinate Mapping:
   * * Longitude -> X [deg]
   * * Latitude  -> Y [deg]
   * * Altitude  -> Z [m]
   * \sa Coordinates_Geodetic
   * 
   * \note The order of the coordinates is not altered and Longitude, Latitude,
   * Altitude is assumed.
   * \param[in] coordinateString A string with coordinate information.
   * \param[in] coordinateSeperator (optional) The sequence used to separate the 
   *  individual coorinates.
   * \return The created coordinate tuple instance.
   */
  Coordinates_Geodetic kmlStringToPointCoordinate(const std::string& coordinateString,
                            const std::string& coordinateSeperator = ",");
  
//   using Point_2D    = model::d2::point_xy<double>;
  
  using Box_2D      = model::box<Geometry::Point_2D>;
  using Polygon_2D  = model::polygon<Point_2D,false>;
  
  
  /** \brief Get a simplified ring that is boost::equal to the original.
   * \details Some geometries contain zero-area spikes into their interior. E.g:
   * 
\code
A------------B
|            |
|  D-------(C,E)
|             \
G--------------F
\endcode
   *
   * Technically the ring (A,B,C,D,E,F,G,A) is boost::equal to the 
   * ring (A,B,C,F,G,A). This function tries to find that simplified version.
   * 
   * \param[in] original The orginial 'complicated' ring.
   * \param[in] areaThreshold (optional) The "zero area" threshold value.
   * \return A simplified boost::equal version of the orginal.
   */
  Polygon_2D::ring_type getBoostEqualSimplified(Polygon_2D::ring_type const & original
    ,double const& zeroAreaThreshold = 0.05 );
  
  
  inline Point_2D midpoint(Point_2D const & a, Point_2D const & b)
  {
    return Point_2D(0.5*(a.x()+b.x()),
                    0.5*(a.y()+b.y()));
  };
  
  /** \brief Finds the point on the line through a and b closest to p.
   * \param[in] p The point to find the closest point on the line for
   * \param[in] a One point on the line.
   * \param[in] b Another point on the line.
   * \param[in] truncateAtA (optional) If TRUE, then the returned closest point
   *  is bound to the ray from a through b.
   * \param[in] truncateAtB (optional) If TRUE, then the returned closest point
   *  is bound to the ray from b through a.
   * \return The point on the line through a and b that is closest to p.
   */
  Point_2D closestPointOnLine(Point_2D const& p, Point_2D const& a, Point_2D const& b,
    bool truncateAtA = false, bool truncateAtB = false );
  
  Point_2D closestBoundaryPoint(Point_2D const & point, Polygon_2D const & polygon);
  
  Point_2D closestPointOnRing(Point_2D const &point, Polygon_2D::ring_type const & ring);
  
  Box_2D boundingBox(Polygon_2D const & polygon);
  
  /** \brief Create a point from a string of coordinates.
   * For this 2D point, the altitude is neglegteced.
   * \note The order of the coordinates is not altered.
   * \param[in] coordinateString A string with coordinate information.
   * \param[in] coordinateSeperator (optional) The sequence used to separate the 
   *  individual coorinates.
   * \return The created point instance.
   */
  Point_2D kmlStringToPoint(const QString& coordinateString,
                            const QString& coordinateSeperator = ",");
  /** \brief Create a point from a string of coordinates.
   * \overload
   */
  Point_2D kmlStringToPoint(const std::string& coordinateString,
                            const std::string& coordinateSeperator = ",");
  
  /** \brief Create a (simple) Polygon from a string cointaining point coordinates
   * For this 2D polygon all infomration in the third coordinates are 
   * neglegteced.
   */ 
  Polygon_2D kmlStringToPolygon(const QString& coordinateString,
                                const QString& pointSeperator=" ",
                                const QString& coordinateSeperator = ",");
  /** \brief Create a (simple) Polygon from a string cointaining point coordinates
   * \overload
   */
  Polygon_2D kmlStringToPolygon(const std::string& coordinateString,
                                const std::string& pointSeperator=" ",
                                const std::string& coordinateSeperator = ",");
  
  QString toQString(Geometry::Coordinates_Geodetic const & coords);
  QString toQString(Geometry::Coordinates_ECEF const & coords); 
      
}; // end namespace Geometry


//############################################################################//
//                                                                            //
//  I/O Support                                                               //
//                                                                            //
//############################################################################//



inline std::ostream& operator<<(std::ostream& out, const Geometry::Coordinates_Geodetic& coords )
{ return out << Geometry::toQString(coords).toStdString(); };

inline std::ostream& operator<<(std::ostream& out, const Geometry::Coordinates_ECEF& coords )
{ return out << Geometry::toQString(coords).toStdString(); };

inline QDebug operator<<(QDebug dbg, const Geometry::Coordinates_Geodetic& coords )
{
  dbg.nospace() << Geometry::toQString(coords);
  return dbg.space();
};

inline QDebug operator<<(QDebug dbg, const Geometry::Coordinates_ECEF& coords )
{
  dbg.nospace() << Geometry::toQString(coords);
  return dbg.space();
};



#endif // GEOMETRY_H
