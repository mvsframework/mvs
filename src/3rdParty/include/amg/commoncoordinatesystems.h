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


#ifndef COMMONCOORDINATESYSTEMS_H
#define COMMONCOORDINATESYSTEMS_H

#include "frameofreference.h"
#include "units.hpp"

namespace AMG  {
  
/** \brief The Coordinate Systems related namespace */  
namespace CoSy {
  
  /** \brief Set a pointer to the Earth-centered, Earth-fixed frame of reference
   * Some functions in AMG::Cosy need a referece to the frame that represents
   * the ECEF coordinate system. 
   * (This information is stored in a static variable.)
   *
   * @param[in] frame The frame of reference used as the ECEF frame.
   * 
   * \see ecef_frame
   */
  void setECEF(FrameOfReference * frame);
  
  
  /** \brief Get a pointer to the Earth-centered, Earth-fixed frame of reference
   * 
   * The counterpart to setECEF().
   * 
   * @return frame The frame of reference used as the ECEF frame.
   */
  FrameOfReference* getECEF();
  
  
  /** \brief Set a pointer to the frame used as the Datum
   * (This information is stored in a static variable.)
   * 
   * @param[in] frame The frame of reference used as the Datum frame.
   * 
   * \see datum_frame
   */
  void setDatumFrame(FrameOfReference* frame);
  
  
  /** \brief Get the currently set Datum frame.
   * The counterpart to setDatumFrame().
   *
   * \pre datum_frame != NULL
   * 
   * @return A pointer to the frame of reference used as the Datum frame.
   */
  FrameOfReference* const getDatumFrame();
   
  
  /** \brief Create a NED-frame at the given location.
   * 
   * Create a new North-East-Down (NED) frame at the given location (at an 
   * implied altitude of \c 0m, i.e. on the surfarce of the WGS84 referece
   * ellipsoid.
   * 
   * \pre The static \c ecef_frame is not \c NULL
   * \pre -M_PIl   <= longitude <= M_PIl
   * \pre -M_PI_2l <= latitude  <= M_PI_2l
   * 
   * @param[in] longitude The longitude of the datum of the NED frame in <units>.
   * @param[in] latitude The latitude of the datum of the NED frame in <units>.
   * @return A copy by value of the newly created NED frame.
   */
  template<typename units=AMG::Units::degree>
  FrameOfReference createNorthEastDown(const double& longitude  ,
                                         const double& latitude ); 
  
  /** \brief Create a NED-frame at the given location.
   * 
   * Create a new North-East-Down (NED) frame at the given location (at an 
   * implied altitude of \c 0m, i.e. on the surfarce of the WGS84 referece
   * ellipsoid.
   * 
   * \pre The static \c ecef_frame is not \c NULL
   * \pre -M_PIl   <= longitude <= M_PIl
   * \pre -M_PI_2l <= latitude  <= M_PI_2l
   * 
   * @param[in] longitude The longitude of the datum of the NED frame in <units>.
   * @param[in] latitude The latitude of the datum of the NED frame in <units>.
   * @return A pointer to the the newly created NED frame.
   * 
   * \note This function calls \b new FrameOfReference, so watch out for memory
   *  leaks. It's your resonsibiliyt to clean up after yourself!
   */
  template<typename units=AMG::Units::degree>
  FrameOfReference* newNorthEastDown(const double& longitude  ,
                                       const double& latitude );
  
  
} // end namespace CoSy
} // end namespace AMG




#endif // COMMONCOORDINATESYSTEMS_H
