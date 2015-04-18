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

#ifndef GUSTUAVPRIVATE_H
#define GUSTUAVPRIVATE_H

#include "Nodes/vehicle_p.h"

#include "GustInterface/gustinterface.h"

class GustUav;

class GustUavPrivate : public VehiclePrivate {  
private:
  Q_DECLARE_PUBLIC(GustUav);
  GustUav* const q_ptr;
  
public:
  GustUavPrivate ( GustUav* q
                  ,GustUav::Settings* const settings
                  /*,Vehicle* parent*/   );
  virtual ~GustUavPrivate();

  GustUav::Settings* const xmlSettings;
  
  void unInitializeGustUav();
  
  /** \brief The actual interface to communicate with GUST. */
  GustInterface* gust = nullptr;
 
  void updateCurrentWaypointIndex(unsigned int const & index);
  
  /** \brief Check whether the current flightplan is finished.
   *  
   * When the last waypoint of the current flightplan is within 
   * threshold_distance_m distance from the current 3D position of the vehicle
   * when this function is called, then the flightplan is decelared finished
   * and GustUav::flightplanFinished is emited.
   * 
   * \param[in] threshold_distance_m (optional) The threshold distance in meter.
   * \return TRUE if the flightplan is declared finished; FALSE otherwise.
   */
  bool detectFinishedFlightPlan(const double& threshold_distance_m = 2.0); //FIXME: Magic Numbers

  

};

#endif // GUSTUAVPRIVATE_H
