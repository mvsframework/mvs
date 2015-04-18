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

#ifndef VEHICLE_PRIVATE_H
#define VEHICLE_PRIVATE_H

#include <QtCore/QTimer>

#include "Nodes/node_p.h"
#include "DDS/addressedtopic.h"
#include "DDS/reliabletopic.h"

#include "GNC/waypoint.h"
#include "GNC/flightplan.h"
#include "GNC/multihopguidance.h"


class Vehicle;

class VehiclePrivate : public NodePrivate {
private:
  Q_DECLARE_PUBLIC(Vehicle);
  Vehicle* const q_ptr;
  
public:
  VehiclePrivate( Vehicle* q
                 ,Vehicle::Settings* const settings
                 /*,Node* parent*/  );
  virtual ~VehiclePrivate();
     
  Vehicle::Settings* const xmlSettings;
  
  /** \brief A set of all the detected ControlStation Nodes in the network.
   * 
   * This set is kept current through two Q_SIGNALS: Node::blipAdded and
   * Node::aboutToDeleteBlip. The connections are established in 
   * Vehicle::initializeItem.
   * 
   */
  QSet<NodeId> detectedControlStations;
  
  
  /** \brief The ControlStation this Vehicle is currently associated with
   * \details This ControlStation can command this vehicle. As such, the 
   * affiliation to a control station creates a subgroup of Nodes in the Network.
   * \sa processControlStationModeChanges  */
  Blip* commandingControlStation = nullptr;
  
  /** \brief Qt Connection to keep track of the COM link status of the commanding
   * ControlStation
   *\sa setCommandingControlStation */
  QMetaObject::Connection qtCon_trackLinkToCommandingControlStation;
    
  /** \brief Timeout period after which to start the lost link procedure. [s] */
  const int lostLinkTimeOut = 60; //FIXME: Magic Number: Timeout after which to start the lost link procedure
  
  /** \brief Lost link timer that triggers the lost link procedure when timing out. */
  QTimer* lostLinkTimer = nullptr;
  
  /** \brief Update the connectednessToCommandingControlStation property of Blips.*/
  virtual void updateRfConnectedArea();

  
  virtual void initializeBlip( const NodeId& id, const Node::Type& type
    , QString const & name, QStringList const & propertyList, Blip* blip = nullptr );
  
  /** \brief Delete the blip for the given id. */
  virtual void deleteBlip(NodeId const &id);

  
  /** \brief Keep track of the commandingControlStation throu Status message monitoring.
   *
   * This function is called whenever a (ControlStation) Blip sends a new 
   * IDL::Data::Status message. The function inspects the mode status of the 
   * Blip and updates the commandingControlStation pointer accordingly:
   * 
   * - if commandingControlStation was not set and a ControlStation Blip became 
   *   operational, that Blip is set as the commandingControlStation.
   * - if the Blip identified through commandingControlStation exits its operational
   *   state, commandingControlStation is reset to nullptr.
   * 
   * \todo Obviously this logic to determine the currently commanding 
   * control station is somewhat flawed, but for now - as there will only be one
   * - it'll do just fine.
   * 
   * \param[in] id The NodeId of a ControlStation Blip that just updated its 
   *  mode.
   */
  void processControlStationModeChanges(NodeId const & id);

  
  /** \brief The Vehicle currently in the "primary" role.
   * \sa keepTrackOfPrimaryVehicle */
  Blip* primaryVehicleBlip = nullptr;
  
  /** \brief Keep track of the current primary vehicle by snooping on ModeChange messages.
   */
  void processVehicleModeChanges(NodeId const & id);
  
  void processModeChanges(NodeId const & id);
  
  
  /** \brief Update the target destination for multi-hop operations. */
  void updateMultiHopGuidance();
  
//   /** \brief Get the Waypoint closest to the R/F intersection centroid of two Blips.
//    * 
//    * \param[in] head One of the Blips to connect
//    * \param[in] tail The other Blip
//    * \param[in] moveToRouteGraph (optional) Moves the resulting waypoint to the 
//    * closest point on the route graph if TRUE; doesn't change the computed location
//    * if FALSE.
//    * \return The waypoint closest to the centroid of the intersection or the 
//    * middle between head and tail of no intersection exists. */
//   Waypoint rfIntersectionCentroid(Blip* head, Blip* tail, bool const & moveToRouteGraph = false) const;
  
  
  /** \brief The data needed for operating Dual-Hop guidance.
   * \details The State dualHopGuidance takes care of the memory management, 
   *  calling \c new and \c delete in DualHopGuidance::onEntry and in 
   *  DualHopGuidance::onExit, respectively.
   */
  DualHopGuidanceData* dualHopGuidanceData = nullptr;
  
  
  //
  // The Vehicle-specific states for the SimItem's state machine.
  //

  /** \brief A template-substitute State providing access to VehiclePrivate. */
  class State : public ::State {
  public:
    explicit State(StateId const stateId, ::State * parent, VehiclePrivate* const d_ptr)
      : ::State(stateId,parent)
      ,vehicle_private_ptr(d_ptr){};
    explicit State(StateId const stateId, QState::ChildMode childMode, ::State* parent, VehiclePrivate* const d_ptr)
      : ::State(stateId,childMode,parent)
      ,vehicle_private_ptr(d_ptr){};
    virtual ~State() = default;
    
  protected:
    ///< \brief A pointer to the associated VehiclePrivate.
    VehiclePrivate* const vehicle_private_ptr; 
  };
  
  /** \brief A template-substitute Activity providing access to VehiclePrivate. */
  class Activity : public ::Activity {
  public:
    explicit Activity(StateId const stateId, ::State* parent, VehiclePrivate* const d_ptr)
      : ::Activity(stateId, parent)
      ,vehicle_private_ptr(d_ptr)
      {};
    virtual ~Activity() = default;
  protected:
    ///< \brief A pointer to the associated VehiclePrivate.
    VehiclePrivate* const vehicle_private_ptr; 
  };
  
  
  State* autonomous = nullptr;
  Activity* executingExternalCommands = nullptr;
  
  
  /** \brief Lost Link Activity.
   * \details The lost link activity is enterend when a vehicle decides that it
   * has lost to its commanding control station. (Which currently is triggered 
   * through the QTimer::timeout() signal from lostLinkTimer, set at 
   * lostLinkTimeOut.)
   * 
   * As such, the state/activity is "done" whenever the communication is 
   * restablished. To capture that, a QSignalTransition is used to catch the 
   * commandingControlStation::connectednessToHostChanged() signal, which, when
   * transporting the TRUE state, indicates a reestablishment of the COM link to
   * the commanding ControlStation.
   * 
   * This signal-based transition has to be reset everytime the commanding 
   * ControlStation changes as the underlying Qt signal sender changes, too.
   */
  class LostLinkActivity : public Activity
  {
  public:
    explicit LostLinkActivity(::State* parent, VehiclePrivate* const d_ptr)
      :Activity(IDL::Data::STATE_ID_V_LOST_LINK, parent,d_ptr){};
    virtual ~LostLinkActivity() = default;
    /** \brief Setup the transition to the done substate.
     * \details The transition is tied to a signal of the commanding 
     * ControlsStation, which can change. As such, a transition tying the current
     * commanding ControlsStation's connectednessToHostChanged signal to the
     * done substate is setup each time the LostLinkActivity is entered. */
    virtual void onEntry( QEvent* event );
    /** \brief Teardown the transition to the done substate.
     * \details The transition to the done substate is/was tailored to the current
     * commandingControlStation. As this could change, the associated transition
     * is removed and scheduled for deletion. */
    virtual void onExit( QEvent* event );
  protected:
    QSignalTransition* reestablishedLink = nullptr;
  };
  LostLinkActivity* lostLinkProcedure = nullptr;

  State* joystickControl = nullptr;
  
  class Piloted : public State {
  public:
    Piloted(::State* parent, VehiclePrivate* const d_ptr)
      :State(IDL::Data::STATE_ID_V_PILOTED,parent,d_ptr){};
    virtual ~Piloted() = default;
    virtual void onEntry ( QEvent* event );
  };
  Piloted* piloted = nullptr;
  
  class ExecutingExternalPlan : public State {
  public:
    ExecutingExternalPlan(::State* parent, VehiclePrivate* const d_ptr)
      :State(IDL::Data::STATE_ID_V_EXTERNAL_PLAN,parent,d_ptr){};
    virtual ~ExecutingExternalPlan() = default;
    virtual void onEntry ( QEvent* event );
  };
  ExecutingExternalPlan* executingExternalPlan = nullptr;
  
  /** \brief A "Work" state that moves the Vehicle to the RouteGraph. */
  class MovingToRouteGraph : public State {
  public:
    MovingToRouteGraph(::State* parent, VehiclePrivate* const d_ptr)
      :State(IDL::Data::STATE_ID_V_MOVING_TO_ROUTE,parent,d_ptr){};
    virtual ~MovingToRouteGraph() = default;
    virtual void onEntry ( QEvent* event );
  };
  MovingToRouteGraph* movingToRouteGraph = nullptr;
  
  /** \brief A "Work" state that moves the Vehicle to its startup location. */
  class MovingToHome : public State {
  public:
    MovingToHome(::State* parent, VehiclePrivate* const d_ptr)
      :State(IDL::Data::STATE_ID_V_MOVING_TO_HOME,parent,d_ptr){};
    virtual ~MovingToHome() = default;
    virtual void onEntry ( QEvent* event );
  };
  MovingToHome* movingToHome = nullptr;
  
  class Loiter : public State {
  public:
    Loiter(::State* parent, VehiclePrivate* const d_ptr)
      :State(IDL::Data::STATE_ID_V_LOITER,parent,d_ptr){};
      virtual ~Loiter() = default;
      virtual void onEntry( QEvent* event );
  };
  Loiter* pilotedLoiter = nullptr;
  Loiter* autonomousLoiter = nullptr;
  
  /** \brief A "Work" state that moves the Vehicle to the location of the (last
   * known) commanding ControlStation. */
  class MovingToControlStation : public State {
  public:
    MovingToControlStation(::State* parent, VehiclePrivate* const d_ptr)
      :State(IDL::Data::STATE_ID_V_MOVING_TO_CONTROLSTATION,parent,d_ptr){};
      virtual ~MovingToControlStation() = default;
      virtual void onEntry( QEvent* event );
  };
  MovingToControlStation* movingToControlStation = nullptr;
  
  class DualHopGuidance : public State {
  public:
    DualHopGuidance(::State* parent, VehiclePrivate* const d_ptr)
      :State(IDL::Data::STATE_ID_V_DUAL_HOP_GUIDANCE,parent,d_ptr){};
    virtual ~DualHopGuidance() = default;
    virtual void onEntry( QEvent* event );
    virtual void onExit( QEvent* event );
  };
  DualHopGuidance* dualHopGuidance = nullptr;
  
  class PerimeterGuidance : public State {
  public:
    PerimeterGuidance(::State* parent, VehiclePrivate* const d_ptr)
      :State(IDL::Data::STATE_ID_V_PERIMETER_GUIDANCE,parent,d_ptr){};
    virtual ~PerimeterGuidance() = default;
//     virtual void onEntry( QEvent* event ){};
  };
  PerimeterGuidance* perimeterGuidance = nullptr;
  
  class DynamiceGuidance : public State {
  public:
    DynamiceGuidance(::State* parent, VehiclePrivate* const d_ptr)
      :State(IDL::Data::STATE_ID_V_DYNAMIC_GUIDANCE,parent,d_ptr){};
    virtual ~DynamiceGuidance() = default;
//     virtual void onEntry( QEvent* event ){};
  };
  DynamiceGuidance* dynamiceGuidance = nullptr;
  
  void initializeStates();
  
  void unInitializeVehicle();
  
  /** \brief Go directly to the given destination. */
  void goTo_direct(Waypoint const& destination);
 
  /** \brief Go to the point on the routegraph closest to the destinaton.
   * \note The waypoint on the route graph is set at routeGraphAltitude.
   * \sa Blip::routeAltitude()  
   * 
   * \param[in] destination The point to get close to.
   * \param[in] interpolate (optional) TRUE if the actual target can be on 
   *  edges; FALSE if only actual graph vertices can be used.
   */
  void goTo_closestOnRouteGraph( const Waypoint& destination , bool interpolate = true);
  
  /** \brief Construct a Flightplan to a destination via the route graph.
   * 
   * \note <b> The resulting plan is \u NOT guranteed to be colision free! </b>
   * 
   * \details Construct a flight paln that goes to the routegraph, find the closest point
   * on the graph to the destination, goes there, an then leaves the graph for 
   * the destination.
   * 
   * \note The motion on the route graph happens at routeGraphAltitude, but the
   *  destination altitude is not altered.  
   * \sa Blip::routeAltitude()  
   * 
   * \param[in] destination The point to reach (via the route graph).
   * \param[in] interpolate (optional) TRUE if the actual target can be on 
   *  edges; FALSE if only actual graph vertices can be used.
   * \return The corresponding Flightplan.
   */
  Flightplan plan_viaRouteGraph(Waypoint const & destination, bool interpolate = true);
  
  /** \brief Use the routegraph to go to the destination.
   * 
   * A convenience function that constructs and immedaitely executes a 
   * Flightplan to the destination, using a route along the route graph.
   * \param[in] destination The point to reach (via the route graph).
   */
  void goTo_viaRouteGraph(Waypoint const& destination)
  { processFlightplan(plan_viaRouteGraph(destination)); };
  
  /** \brief Convert a Flightplan into the corresponding IDL version. */
//   IDL::Data::Flightplan convertFlightplan(Flightplan const & gncPlan);

  /** \brief Process a Flighplan before it's execution. */
  void processFlightplan( const Flightplan& flightplan );
  /** \overload */
//   void processFlightplan( IDL::Data::Flightplan const & flightplan);
  
  Waypoint randomWaypointInArena( const double& altitude = 100 );
  
  
  /** \brief A flightplan that came from an external source. */
  Flightplan externalFlightplan;
  
  // 
  // Topic : Joystick
  //
  
  /** \brief The topic for exchanging Joystick messages.
   * \note This is a receive-only topic.
   */
  AddressedTopic<IDL::Messages::JoystickCommand,Vehicle,TopicOptions::Receive>* joystickTopic = nullptr;

  //
  // Topic: Mode Change Command
  //
  
  /** \brief The topic for exchanging Command messages. */
  AddressedTopic<IDL::Messages::ModeChangeCommand, Vehicle, TopicOptions::Receive>* commandTopic = nullptr; 

  
  //
  // Topic: External Plan Command
  //
  
  /** \brief The topic for exchanging externally provided Flightplan command messages. */
  ReliableTopic<IDL::Messages::ExternalPlanCommand, Vehicle, TopicOptions::Receive>* planCommandTopic = nullptr; 
//   AddressedTopic<IDL::Messages::ExternalPlanCommand, Vehicle, TopicOptions::Receive>* planCommandTopic = nullptr; 

  //
  // Topic: Dual Hop Guidance
  //
  
  /** \brief The topic for exchanging DualHopGuidance messages.  */
  BroadcastTopic<IDL::Messages::DualHopGuidance,Vehicle,TopicOptions::SendReceive>* dualHopGuidanceTopic = nullptr;
  
  IDL::Messages::DualHopGuidance* dualHopGuidanceMessage = nullptr;
  
  
  void initializeDatalink();

};

#endif // VEHICLE_PRIVATE_H
