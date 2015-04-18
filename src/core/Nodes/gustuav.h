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

#ifndef GUSTUAV_H
#define GUSTUAV_H

#include "Nodes/vehicle.h"
#include <GustInterface/datalinkmessages.h>

class GustUavPrivate;

/** \brief A Vehicle utilizing an externally running GUST instance to implement the motion.
 */
class GustUav : public Vehicle {
  Q_OBJECT
  
public Q_SLOTS:
  /** \brief Synchronize the ownship information to the most recent GUST data.
   * 
   * This slot, normally connected to 
   * GustInterface::stateVectorUpdateAvailable(), updates the ownship's state 
   * data with data from GUST (received via the GustInterface).
   * 
   * \internal 
   * This function effectively updates:
   * - stateMessage->position
   * - stateMessage->velocity
   * - stateMessage->attitude
   * and then triggers a synchronization via 
   * processReceivedMessage(ownshipId, *stateMessage).
   */
  void synchronizeOwnshipState();
  
    
  virtual void stopMotion();
  virtual void continueMotion();
  
  /** \brief Send the <tt>trajGo</tt> command to GUST. 
   * 
   * \deprecated
   * \note This is mainly a debug functionality.
   * 
   * <tt>trajGo</tt> starts the execution or continuation of the currently 
   * active (GUST) flight plan, which might be different from a/the flight plan
   * in \ref MVS.
   */
  void gust_trajGo();
  
  /** \brief Send the <tt>trajStop</tt> command to GUST.
   * 
   * \deprecated
   * \note This is mainly a debug functionality.
   * 
   * <tt>trajStop</tt> stops the execution the currently active (GUST) flight 
   * plan. Rotorcraft will transition to a hover with the currently set limits
   * on acceleration and jerk.
   */
  void gust_trajStop();
  
  /** \brief Send the panic stop command to GUST.
   * 
   * \deprecated
   * \note This is mainly a debug functionality.
   * 
   * A panic stop is a full stop (for rotorcraft to hover) in the shortest
   * possible horizontal distance. As a result, when executing this command,
   * the vehicle will most likely climb.
   */
  void gust_trajPanicStop();
  
  
public:
  /** \brief The XSD complex type containing settings elements for a GustUav. */
  using Settings = XML::SAI::GustUavNode_t;
  
  /** \brief Constructor.
   * \note Vehicles don't have parents, hence GustUavs don't have them either.
   * \param[in] settings The defining settings for this GustUav.
   */
  explicit GustUav( Settings* const settings /*,Vehicle* parent*/ );
  
  /** \brief Destructor. */
  virtual ~GustUav();
  
  virtual void timerEvent ( QTimerEvent* event );
  
  
protected:
  explicit GustUav( GustUavPrivate& dd /*, Vehicle* parent = nullptr*/ );
  
  virtual void initializeItem();
  virtual void startItem();
  virtual void stopItem();
  virtual void unInitializeItem();
  
  virtual void executeFlightplan ( const Flightplan& plan );
  virtual void processReceivedMessage ( const NodeId& sender, const IDL::Messages::JoystickCommand& message );

private:
  Q_DISABLE_COPY(GustUav)
  Q_DECLARE_PRIVATE(GustUav);
};

#endif // GUSTUAV_H
