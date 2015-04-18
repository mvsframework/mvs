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

#ifndef GUSTINTERFACE_H
#define GUSTINTERFACE_H

#include "GustInterface/datalinkmessages.h"
#include "Nodes/gustuav.h"

class GustInterfacePrivate; // forward declaration

Q_DECLARE_METATYPE(GUST::M1_Safemode_Status)

/** \brief An interface to an instance of GUST, the Georgia Tech UAV Simulation Tool.
 * The GustInterface, implemented as a SimItem and configured via XML, provides
 * an interface to an oboard instance of GUST by mimicking the messages send 
 * from a GUST controls station instance. By means of monitoring the messages 
 * send back down from the onboard instance, the interface can keep track of the 
 * onboard instance's status and activities.
 * 
 * As GUST internally uses a datum that is independently settable, the reported
 * vector values, such as position() and velocity(), are given wrt. that datum
 * frame. 
 */
class GustInterface : public SimItem{
Q_OBJECT
Q_ENUMS(TrajectoryCommand)
Q_ENUMS(GuidanceMode)
Q_PROPERTY(bool liveliness READ isAlive NOTIFY livelinessChanged)

public:
  using TrajectoryCommand = GUST::TrajectoryCommand;
  using GuidanceMode = GUST::M1_Safemode_Status;

Q_SIGNALS:
  /** \brief Indicates the receipt of a datalink message from the GUST instance. */
  void messageReceived(int /*dataLinkMessageId*/); 
 
  /** \brief Indicates a change in liveliness of the connection to the GUST instance. */
  void livelinessChanged(bool /*isAlive*/);
  
  /** \brief Indicates a change in the current guidance mode of the GUST instance. */
  void guidanceModeChanged(GuidanceMode /*currentSafeMode*/);
  
  /** \brief A flight plan message from GUST/ESIM has been received.
   * This signal indicates that the DatalinkInterface has received a flight plan 
   * message from its attached GUST instance. 
   * 
   * \note If this signal is received some time other than afer sending a flight
   *  plan, this indicates that the GUST A/C instance has received a flight plan
   *  from somewhere else, overwriting whatever plan was active or stored before.
   */
  void flightPlanReceived();

  void stateVectorUpdateAvailable(); ///< \brief New state vector data (position, velocity, attitude) is available.
  void datumUpdateAvailable(); ///< \brief GUST has altered the datum frame of reference.
  
  void currentWaypointChanged(unsigned int /*waypointIndex*/);
  
public Q_SLOTS:
  void sendCommand(TrajectoryCommand command);
  
  void sendJoystick(IDL::Data::JoystickData data);
  
public:
  using Settings = XML::SAI::GustInterface_t; //NOTE: Change to something like XML::SAI:GustInterface_t

  static void  generateSetupInputFiles( GustUav::Settings const * settingsXml );
  
  explicit GustInterface( Settings* const settings
                         ,Node* parent);
  virtual ~GustInterface();
  
  /** \brief Get the last known position of the attached GUST instance. 
   * The position is given wrt. the GUST Datum frame (which is a NED frame).
   */
  AMG::Vector position() const;
    
  /** \brief Get the last known velocity of the attached GUST instance.
   * The velocity is given wrt. the GUST Datum frame (which is a NED frame).
   */
  AMG::Vector velocity() const;
  
  /** \brief Get the last known attitude of the attached GUST instance.
   * The attitude is given wrt. the GUST Datum frame (which is a NED frame).
   */
  AMG::QuaternionTupel attitude() const;
  
  bool uploadFlightplan( const Flightplan& flightplan );

  /** \brief Checka the communication with the GUST instance for liveliness.
   * \return TRUE if the connection is alive; FALSE otherwise.
   */
  bool isAlive();



protected Q_SLOTS:
  void processConnectionTimeOut(){};
  void readPendingDatagramms();
  
protected:
  explicit GustInterface ( GustInterfacePrivate& dd, Node* parent = nullptr );
  
  virtual void initializeItem();
  virtual void startItem();
  virtual void stopItem();
  virtual void unInitializeItem();
  

  
private:
  Q_DISABLE_COPY(GustInterface)
  Q_DECLARE_PRIVATE(GustInterface);

};

#endif // GUSTINTERFACE_H
