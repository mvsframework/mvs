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

#ifndef FLIGHTPLAN_H
#define FLIGHTPLAN_H

#include <QtCore/QList>

#include "GNC/waypoint.h"

class Flightplan {
public:
  /** \brief Compute the hash used to identify a flight plan.
   * \details This is done on the IDL::Data::WaypointSeq as that ensures that
   * the waypoints are stored sequentially. (The QList used in Flightplan
   * does not guarantee this.)
   * \param[in] wpSequence The (contiguous) sequence of Waypoints to be hashed.
   * \return A 128 bit (16 byte) MD5 hash. */
  static IDL::Data::Md5Hash hashWaypoints(IDL::Data::WaypointSeq const& wpSequence);
  
  /** \brief Convert a GNC type Flightplan to an IDL::Data::Flightplan. */
  static IDL::Data::Flightplan convertToIdlFlightplan(Flightplan const & gncPlan);
  
  /** \brief Eliminate double-backing at the start and end of a Flightplan.
   * \details Due to the way the flighplan is build, there are sometimes 
   * cases of odd beginnings and endings in a flight plan as the paths doubles 
   * back. An example:
\code
     (1)                       (n)
      |                         |
(3)--(2)---(4)--...---(n-3)---(n-1)--(n-2)
\endcode

   *
   * This function tries to eliminate those things through a computation of the
   * area spanned by the triangels (2,3,4) and (n-1,n-2,n-3).
   * If those are deemed "small", then waypoints (3) and (n-2) are deleted, 
   * respectively.
   * 
   * \note This function most works on Flightplans generated through
   * VehiclePrivate::plan_viaRouteGraph(). Its effect on generic Flighplans 
   * might be different!
   *  
   * \param[in,out] fp The flightplan to operate on.
   * \param[in] areaThreshold Threshold below which waypoints get deleted from 
   * the flightplan.
   * \return TRUE if the flighplan has been altered; FALSE otherwise.
   */
  static bool eliminateDoubleBacks(Flightplan & fp, double const& areaThreshold = 0.5);
  
public:
  /** \brief Default (empty) constructor. */
  Flightplan() = default; 
  
  /** \brief Destructor. */
  ~Flightplan() = default; 
  
  /** \brief Copy-constructor. */
  Flightplan(IDL::Data::Flightplan const & idlPlan);
  
  /** \brief Get this Flightplan as an IDL::Data::Flightplan.
   * \return The corresponding IDL type flightplan. */
  IDL::Data::Flightplan toIdl() const
  { return convertToIdlFlightplan(*this); };
  
  /** \brief Conversion operator to IDL::Data::Flightplan.
   * \return The corresponding IDL type flightplan. */
  operator IDL::Data::Flightplan() const
  { return convertToIdlFlightplan(*this); };
   
  /** \brief Append a Waypoint to this Flightplan. */
  void append( Waypoint const & waypoint);
  /** \overload */
  void append( QList<Waypoint> const & waypoints);
  /** \overload */
  void append( Flightplan const & plan)
  { append(plan.waypoints); };
  
  /** \brief The ordered Waypoint list making up this Flightplan. */
  QList<Waypoint> waypoints;
  
  /** \brief A MD5 hash over the waypoints. */
  QByteArray hash;
};

#endif // FLIGHTPLAN_H
