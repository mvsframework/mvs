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


#ifndef GIS_H
#define GIS_H

#include <cmath>
#include <iostream>
#include "vector.h"

namespace AMG {
  
/** \brief The Geo Information Services namespace
  */   
namespace GIS {
  
  /** \brief The 16 cardinal and ordinal directions.
   * \sa https://en.wikipedia.org/wiki/Boxing_the_compass
   */
  enum class CompassPoint : int {
   N         =   000, ///< \brief North
                      ///< \details Heading 0.00°
        NbE  =  1125, ///< \brief North by East
                      ///< \details Heading 11.25°
      NNE    =  2250, ///< \brief North-northeast
                      ///< \details Heading 22.50°
        NEbN =  3375, ///< \brief Northeast by North
                      ///< \details Heading 33.75°
    NE       =  4500, ///< \brief Northeast
                      ///< \details Heading 45.00°
        NEbE =  5625, ///< \brief Northeast by East
                      ///< \details Heading 56.25°
      ENE    =  6750, ///< \brief East-northeast
                      ///< \details Heading 67.50°
        EbN  =  7875, ///< \brief East by North
                      ///< \details Heading 78.75°
   E         =  9000, ///< \brief East
                      ///< \details Heading 90.00°
        EbS  = 10125, ///< \brief East by South
                      ///< \details Heading 101.25°
      ESE    = 11250, ///< \brief East-southeast
                      ///< \details Heading 112.50°
        SEbE = 12375, ///< \brief Southeast by East
                      ///< \details Heading 123.75°
    SE       = 13500, ///< \brief Southeast
                      ///< \details Heading 135.00°
        SEbS = 14625, ///< \brief Southeast by South
                      ///< \details Heading 146.25°
      SSE    = 15750, ///< \brief South-southeast
                      ///< \details Heading 157.50°
        SbE  = 16875, ///< \brief South by East
                      ///< \details Heading 168.75°
   S         = 18000, ///< \brief South
                      ///< \details Heading 180.00°
        SbW  = 19125, ///< \brief South by West
                      ///< \details Heading 191.25°
      SSW    = 20250, ///< \brief South-southwest
                      ///< \details Heading 202.50°
        SWbS = 21375, ///< \brief Southwest by South
                      ///< \details Heading 213.75°
    SW       = 22500, ///< \brief Southwest
                      ///< \details Heading 225.00°
        SWbW = 23625, ///< \brief Southwest by West
                      ///< \details Heading 236.25°
      WSW    = 24750, ///< \brief West-southwest
                      ///< \details Heading 247.50°
        WbS  = 25875, ///< \brief West by South
                      ///< \details Heading 258.75°
   W         = 27000, ///< \brief West
                      ///< \details Heading 270.00°
        WbN  = 28125, ///< \brief West by North
                      ///< \details Heading 281.25°
      WNW    = 29250, ///< \brief West-northwest
                      ///< \details Heading 292.50°
        NWbW = 30375, ///< \brief Northwest by West
                      ///< \details Heading 303.75°
    NW       = 31500, ///< \brief Northwest
                      ///< \details Heading 315.00°
        NWbN = 32625, ///< \brief Northwest by North
                      ///< \details Heading 326.25°
      NNW    = 33750, ///< \brief North-norhtwest
                      ///< \details Heading 337.50°
        NbW  = 34875  ///< \brief North by West
                      ///< \details Heading 348.75°
  };
 
  std::string convertToString(CompassPoint const & cp);
  
  /** \brief Ensure that heading is in (-180,180] degrees.
   * \param[in] heading A heading in degree.
   * \return The corresponding heading in (-180,180] degree.
   */
  double ensureHeading(double const & heading_deg);
  
  /** \brief Get the opposite direction heading. 
   * \param[in] heading A headig in degree.
   * \return The oppsing heading in (-180,180] degree.
   */
  double opposingHeading(double const & heading_deg);
  
  /** \brief Convert a literal compass point to a numeric heading. 
   * \param[in] cp Literal compass point.
   * \return Associated heading in degree in (-180,180].
   */
  double compassPointToHeading(CompassPoint const & cp) ;
  
  /** \brief Check if a test heading it toward the left of a reference heading.
   * \param[in] refHdg The referece heading in degree.
   * \param[in] testHdg The heading to test relative to the referece in degree.
   * \return TRUE if testHdg is in [refHdg-180,refHdg); FALSE otherwise.
   */
  bool isInLeftHalfCircle( const double& reference_deg, const double& heading_deg );
  
  /** \brief Check if a test heading it toward the right of a reference heading.
   * \param[in] refHdg The referece heading in degree.
   * \param[in] testHdg The heading to test relative to the referece in degree.
   * \return TRUE if testHdg is in (refHdg,refHdg+180]; FALSE otherwise.
   */
  bool isInRightHalfCircle( const double& reference_deg, const double& heading_deg );
  
  
  /** \brief Convert a heading to the closest point on the compass.
   * \param[in] heading The heading in degree.
   * \param[in] resolution (optional) 4: cardinal points, 8: + ordinal points
   * \return The corresponding compass point.
   */
  CompassPoint headingToCompassPoint( const double& heading_deg, int resolution = 8);
  
  
  /** \brief Convert a position to a Lat, Lon, Alt string .*/  
  std::string geodeticPositionString(AMG::Vector const & position);
  
  
  /** \brief Some data of the World Geodetic System 1984 reference ellipsoid
   * Data is from the web:
   * <a href="linkURL"> http://en.wikipedia.org/wiki/WGS84</a> 
   * <a href="linkURL"> http://earth-info.nga.mil/GandG/publications/tr8350.2/tr8350_2.html</a> 
   * <a href="linkURL"> http://earth-info.nga.mil/GandG/wgs84/gravitymod/egm2008/egm08_wgs84.html</a> 
   * <a href="linkURL"> http://www.nist.gov/pml/pubs/sp330/index.cfm </a>
   */
  struct WGS84{
    // defined
    constexpr static long double a     = 6378137.00L; ///< Equatorial (semi-major) radius in meter.
    constexpr static long double finv  = 298.257223563L; ///< Inverse of the ellipsoid's flatteing, \c 1/f in an ellipsoid.
    constexpr static long double omega = 7292115.0e-11; ///< Earth's angular velocity in radians/seconds. 
    constexpr static long double GM    = 3.986004418e14; ///< Earth's Gravitational constant (mass of atmosphere included) in m^3/s^2
    // computed (from http://earth-info.nga.mil/GandG/publications/tr8350.2/wgs84fin.pdf, table 3.3)
    constexpr static long double f     = 1.0L/298.257223563L; ///< Ellipsoidal latteing \c f, computed from \c 1.0/finv.
    constexpr static long double b     = 6356752.314245L; ///< Polar (semi-minor) radius in meter.
    constexpr static long double e     = 8.1819190842622e-2; ///< First eccentricity.
    constexpr static long double e2    = 6.69437999014e-3; ///< Square of the first eccentricity.
    constexpr static long double ep    = 8.2094437949696e-2; ///< Second eccentricity.
    constexpr static long double ep2   = 6.73949674228e-3; ///< Square of the second eccentricity.
    // other sources
    constexpr static long double g0    = 9.80665L; ///< Standart Acceleration due to gravity in [m/s^2] from NIST SP330
  }; 

  
  /** \brief Geodetic latitude corresponding to the Gudermannian latitudal coordinate.
   * 
   * This function is useful when converting Gudermannian coordinates to
   * geocentric coordinates.
   * 
   * @param[in] yGudermann The Gudermann latitudal coordinate
   * @param[out] phiGeocentric The corresponding geocentric latitude in radians
   * 
   * \see http://en.wikipedia.org/wiki/Mercator_projection
   * \see http://en.wikipedia.org/wiki/Gudermannian_function
   */
  void GudermannFunction(const double& yGudermann, double& phiGeocentric);
  
  
  /** \brief Gudermannian latitudinal coordinate for the given geodetic latitude.
   * 
   * This function is useful when converting geocentric coordinates into 
   * Gudermannian coordinates.
   * 
   * \pre The input geocentric latitude is within the open interval (-pi/2, pi/2)
   * 
   * @param[in] phiGeocentric Geocentric latitude in radians
   * @param[out] yGudermann The corresponding Gudermannian latitudinal coordinate
   * 
   * \see http://en.wikipedia.org/wiki/Mercator_projection
   * \see http://en.wikipedia.org/wiki/Gudermannian_function
   */
  void InverseGudermannFunction(const double& phiGeocentric, double& yGudermann);
  
  
  /** \brief Converts ECEF coordinates to Geodetic coordinates.
   *
   * @param[in] x ECEF X-coordinate in meter.
   * @param[in] y ECEF Y-coordinate in meter.
   * @param[in] z ECEF Y-coordinate in meter.
   * @param[out] lambda Geodetic longitude in radians.
   * @param[out] phi Geodetic latitude in radians.
   * @param[out] h Geodetic elevation above WGS84 in meter.
   * 
   * \see WGS84
   */
  void ecef2geodetic(const double& x      ,
                     const double& y      ,
                     const double& z      ,
                           double& lambda ,
                           double& phi    ,
                           double& h      );
  
  /** \brief Converts Geodetic coordinates to ECEF coordinates
   * 
   * @param[in] lambda Geodetic longitude in radians.
   * @param[in] phi Geodetic latitude in radians.
   * @param[in] h Geodetic elevation above WGS84 in meter.
   * @param[out] x ECEF X-coordinate in meter.
   * @param[out] y ECEF Y-coordinate in meter.
   * @param[out] z ECEF Y-coordinate in meter.
   * 
   * \see WGS84
   */
  void geodetic2ecef(const double& lambda ,
                     const double& phi    ,
                     const double& h      ,
                           double& x      ,
                           double& y      ,
                           double& z      );
    
  /** \brief Converts geocentric coordinates to Gudermannian/Mercator coordinates.
   * 
   * Gudermannian coordinates are used when dealing with Mercator map 
   * projections.
   * 
   * \note This conversion is independent from the underlying reference sphere.
   * 
   * @param[in] lambdaGeocentric Geocentric longitude in radians.
   * @param[in] phiGeocentric Geocentric latitude in radians.
   * @param[in] hGeocentric Geocentric elevation (above reference) in meter.
   * @param[out] xGudermann Gudermannian longitudinal coordinate.
   * @param[out] yGudermann Gudermannian latitudinal coordinate.
   * @param[out] zGudermann Gudermannian elevation (above map) in meter.
   */
  void geocentric2mercator(const double& lambdaGeocentric ,
                            const double& phiGeocentric   ,
                            const double& hGeocentric     ,
                            double& xGudermann            ,
                            double& yGudermann            ,
                            double& zGudermann            );
  
  /** \brief Converts Gudermannian/Mercator coordinates to geocentric coordinates.
   * 
   * Gudermannian coordinates are used when dealing with Mercator map 
   * projections.
   * 
   * \note This conversion is independent from the underlying reference sphere.
   * 
   * @param[in] xGudermann Gudermannian longitudinal coordinate.
   * @param[in] yGudermann Gudermannian latitudinal coordinate.
   * @param[in] zGudermann Gudermannian elevation (above map) in meter.
   * @param[out] lambdaGeocentric Geocentric longitude in radians.
   * @param[out] phiGeocentric Geocentric latitude in radians.
   * @param[out] hGeocentric Geocentric elevation (above reference) in meter.
   */
  void mercator2geocentric(const double& xGudermann  ,
                            const double& yGudermann ,
                            const double& zGudermann ,
                            double& lambdaGeocentric ,
                            double& phiGeocentric    ,
                            double& hGeocentric      );
  
  /** \brief Convert cartesian coordinates to spherical coordinates.
   * 
   * \pre -M_PIl   <= azimuth   <= M_PIl
   * \pre -M_PI_2l <= elevation <= M_PI_2l
   * \pre  0.0     <= distance
   * 
   * @param[in] azimuth The azimuth angle in radians.
   *  The azimuth is measured in the xy-plane in the +z direction. The 
   *  reference is the x-axis. 
   * @param[in] elevation The elevation angle in radians.
   *  Measured from the xy-plane, positive angles in the +z direction.
   * @param[in] distance The radial distance from the origin in meters.
   * @param[out] x The x coordinate in meters.
   * @param[out] y The y coordinate in meters.
   * @param[out] z The z coordinate in meters.
   */  
  void spherical2cartesian(const double& azimuth   ,
                           const double& elevation ,
                           const double& distance  ,
                                 double& x         ,
                                 double& y         ,
                                 double& z         );
  
  /** \brief Convert cartesian coordinates to spherical coordinates.
   * 
   * @param[in] x The x coordinate in meters.
   * @param[in] y The y coordinate in meters.
   * @param[in] z The z coordinate in meters.
   * @param[out] azimuth The azimuth angle in radians.
   *  The azimuth is measured in the xy-plane in the +z direction. The 
   *  reference is the x-axis. 
   * @param[out] elevation The elevation angle in radians.
   *  Measured from the xy-plane, positive angles in the +z direction.
   * @param[out] distance The radial distance from the origin in meters.
   */
  void cartesian2spherical(const double& x         ,
                           const double& y         ,
                           const double& z         ,
                                 double& azimuth   ,
                                 double& elevation ,
                                 double& distance  );
  
  /** \brief Converts ECEF coordinates to geocentric coordinates.
   * 
   * @param[in] x ECEF X-coordinate in meter.
   * @param[in] y ECEF Y-coordinate in meter.
   * @param[in] z ECEF Y-coordinate in meter.
   * @param[out] lambdaGeocentric Geocentric longitude in radians.
   * @param[out] phiGeocentric Geocentric latitude in radians.
   * @param[out] hGeocentric Elevation above reference sphere in meter.
   * 
   * \sa WGS84
   */  
  void ecef2geocentric(const double& x          ,
                       const double& y          ,
                       const double& z          ,
                       double& lambdaGeocentric ,
                       double& phiGeocentric    ,
                       double& hGeocentric      );
  
  
  /** \brief Converts geocentric coordinates to ECEF coordinates
   * 
   * @param[in] lambdaGeocentric Geodetic longitude in radians.
   * @param[in] phiGeocentric Geodetic latitude in radians.
   * @param[in] hGeocentric Geodetic elevation above WGS84 in meter.
   * @param[out] x ECEF X-coordinate in meter.
   * @param[out] y ECEF Y-coordinate in meter.
   * @param[out] z ECEF Y-coordinate in meter.
   * 
   * \sa WGS84
   */
  void geocentric2ecef(const double& lambdaGeocentric ,
                       const double& phiGeocentric    ,
                       const double& hGeocentric      ,
                       double& x                ,
                       double& y                ,
                       double& z                );
    
  
  /** \brief Convert geodetic coordinates to geocentric coordinates
   * The internal conversion is done via the ECEF coordinate system, the used 
   * ellipsoid is AMG::GIS::WGS84. 
   * The reference sphere for the geocentric system has a radius of \c WGS84.a,
   * i.e.the equatorial radius of the WGS84 ellipsoid.
   *    
   * @param[in] lambda Geodetic longitude in radians.
   * @param[in] phi Geodetic latitude in radians.
   * @param[in] h Geodetic elevation above WGS84 in meter.
   * @param[out] lambdaGeocentric Geocentric longitude in radians.
   * @param[out] phiGeocentric Geocentric latitude in radians.
   * @param[out] hGeocentric Geocentric elevation (above reference) in meter.
   */
  void geodetic2geocentric(const double& lambda   ,
                           const double& phi      ,
                           const double& h        ,
                                 double& lambdaGeocentric ,
                                 double& phiGeocentric    ,
                                 double& hGeocentric      );
  
  /** \brief Convert geocentric coordinates to geodetic coordinates 
   * The internal conversion is done via the ECEF coordinate system.
   * The used ellipsoid is AMG::GIS::WGS84.
   * 
   * @param[in] lambdaGeocentric Geocentric longitude in radians.
   * @param[in] phiGeocentric Geocentric latitude in radians.
   * @param[in] hGeocentric Geocentric elevation (above reference) in meter.
   * @param[out] lambda Geodetic longitude in radians.
   * @param[out] phi Geodetic latitude in radians.
   * @param[out] h Geodetic elevation above WGS84 in meter.
   */
  void geocentric2geodetic(const double& lambdaGeocentric ,
                           const double& phiGeocentric    ,
                           const double& hGeocentric      ,
                                 double& lambda   ,
                                 double& phi     ,
                                 double& h        );
  

  /** \brief Convert ECEF coordinates into Mercator (Gudermannian) coordinates.
   * 
   * This function internally does a ECEF -> Geocentric -> Mercator conversion
   * using the appropriated functions also defined in this unit.
   * 
   * @param[in] x ECEF X-coordinate in meter.
   * @param[in] y ECEF Y-coordinate in meter.
   * @param[in] z ECEF Y-coordinate in meter.
   * @param[out] xGudermann Gudermannian longitudinal coordinate.
   * @param[out] yGudermann Gudermannian latitudinal coordinate.
   * @param[out] zGudermann Gudermannian elevation (above map) in meter.
   */
  void ecef2mercator(const double& x          ,
                     const double& y          ,
                     const double& z          ,
                           double& xGudermann ,
                           double& yGudermann ,
                           double& zGudermann );
  
  
  
  /** \brief Convert Mercator (Gudermannian) coordinates into ECEF coordinates.
   * 
   * This function internally does a Mercator -> Geocentric -> ECEF conversion
   * using the appropriated functions also defined in this unit.
   * 
   * @param[in] xGudermann Gudermannian longitudinal coordinate.
   * @param[in] yGudermann Gudermannian latitudinal coordinate.
   * @param[in] zGudermann Gudermannian elevation (above map) in meter.
   * @param[out] x ECEF X-coordinate in meter.
   * @param[out] y ECEF Y-coordinate in meter.
   * @param[out] z ECEF Y-coordinate in meter.
   */
  void mercator2ecef(const double& xGudermann ,
                     const double& yGudermann ,
                     const double& zGudermann ,
                           double& x          ,
                           double& y          ,
                           double& z          );
  
  /** \brief Convert Mercator (Gudermannian) coordinates into Geodetic coordinates
   *  
   * This function internally does a Mercator -> Geocentric -> Geodetic 
   * conversion using the appropriated functions also defined in this unit.
   * 
   * @param[in] xGudermann Gudermannian longitudinal coordinate.
   * @param[in] yGudermann Gudermannian latitudinal coordinate.
   * @param[in] zGudermann Gudermannian elevation (above map) in \b meter.
   * @param[out] lambda Geodetic longitude in \b radians.
   * @param[out] phi Geodetic latitude in \b radians.
   * @param[out] h Geodetic elevation above WGS84 in \b meter.
   */
  void mercator2geodetic(const double& xGudermann ,
                         const double& yGudermann ,
                         const double& zGudermann ,
                               double& lambda     ,
                               double& phi        ,
                               double& h          );
  
  
  /** \brief Convert Geodetic coordinates into Mercator (Gudermannian) coordinates
   * 
   * This function internally does a Geodetic -> Geocentric -> Mercator 
   * conversion using the appropriated functions also defined in this unit.
   * 
   * @param[in] lambda Geodetic longitude in \b radians.
   * @param[in] phi Geodetic latitude in \b radians.
   * @param[in] h Geodetic elevation above WGS84 in \b meter.
   * @param[out] xGudermann Gudermannian longitudinal coordinate.
   * @param[out] yGudermann Gudermannian latitudinal coordinate.
   * @param[out] zGudermann Gudermannian elevation (above map) in \b meter.
   */
  void geodetic2mercator(const double& lambda     ,
                         const double& phi        ,
                         const double& h          ,
                               double& xGudermann ,
                               double& yGudermann ,
                               double& zGudermann );
   
  

} // namespace GIS
} // namespace AMG


std::ostream& operator<<(std::ostream& out, AMG::GIS::CompassPoint const & cp);


#endif // GIS_H
