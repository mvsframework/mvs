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

#ifndef VEHICLE_H
#define VEHICLE_H

#include "Nodes/node.h"

#include "GNC/waypoint.h"
#include "GNC/flightplan.h"

class VehiclePrivate; // forward declaration

/** \brief An abstract base class defining the motion interface of a (mobile) Node.
 * 
 * Vehicles can move and as such this class provides the interface to get
 * that done.
 */
class Vehicle : public Node {
Q_OBJECT
  
//  Q_PROPERTY(NodeId primaryVehicle READ primaryVehicleId NOTIFY primaryVehicleChanged)
//  Q_PROPERTY(NodeId commandingControlStation READ commandingControlStationId NOTIFY commandingControlStationChanged)

  Q_PROPERTY( bool connectedToCommandingControlStation READ isConnectedToCommandingControlStation )

  template<typename Message, typename ParentNodeType, TopicOptions options>
  friend class Topic; // forward declaration 
  

Q_SIGNALS:
  /** \brief Indicates the (successfull) completion of a flightplan. */
  void flightplanFinished();
  
  /** \brief Indicates a detected change of the primary vehicle. */
  void primaryVehicleChanged(NodeId /*id*/);

//FIXME: Without a NULL_NODE_ID this signal is fairly useless ...
//   /** \brief Indicates a detected change of the affiliated control station. */
//   void commandingControlStationChanged(NodeId /*id*/);
  
  
public Q_SLOTS:
  
  /** \brief Pick a random point in the arena and go there.
   * 
   * \deprecated
   * \note This is a mainly a debug functionality.
   * 
   * This function picks a random point in the arena (to be precise, in the 
   * bounding box of the arena) and moves the Vehicle to that point utilizing
   * the route graph.
   */
  void goTo_randomPointInArena();
  
  //
  // State Transition Command Slots
  // ==============================
  // These slots cause the emission of events which could trigger a transition.
  //
  
  /** \brief A Command Slot, posting the IdlCommand Event IDL::Commands::V_ENABLE_JOYSTICK_CONTROL. */
  virtual void goJoystickControl(); 
  
  /** \brief A Command Slot, posting the IdlCommand Event IDL::Commands::V_EXECUTE_EXTERNAL_PLAN. */
  virtual void executeExternalPlan(); 
 
  /** \brief A Command Slot, posting the IdlCommand Event IDL::Commands::V_LOITER. */
  virtual void stopAndLoiter();
  
  
public:
  
  /** \brief The XSD complex type containing settings elements for a Vehicle. */
  using Settings = Node::Settings; 
  
  /** \brief Constructor.
   * \note  A Vehicle is a standalone SimItem and as such doesn't have a 
   * (SimItem) parent.
   * 
   * \param[in] settings The defining settings for this Vehicle.
   */
  explicit Vehicle( Settings* const settings /*,Node* parent*/);
  
  /** \brief Destructor. */
  virtual ~Vehicle();
  
  
  /** \brief Get the Type of this Node. */
  virtual Type type() const;

//NOTE: As there is no NULL NodeID, what would this return in the case of d->primaryVehicleBlip == nullptr?  
//   /** \brief Get the ID of the current primary vehicle. */
//   virtual NodeId primaryVehicleId() const;
  
  /** \brief Get the Blip representing the currently supported primary Vehicle.
   * \note This can return a \c nullptr !
   * \return The corresponding Blip, or \c nullptr if no primary Vehicle is set.
   */
  Blip const* primaryVehicle() const;
  
//NOTE: As there is no NULL NodeID, what would this return in the case of d->commandingControlStation == nullptr?  
//   /** \brief Get the ID of the control station currently commanding this vehicle. */
//   virtual NodeId commandingControlStationId() const;
  
  /** \brief Get the Blip representing the ControlStation currently commanding this vehicle.
   * \note This can return a \c nullptr !
   * \return The corresponding Blip, or \c nullptr if no ControlStation is set.
   */
  Blip const* commandingControlStation() const;
  
  /** \brief Set the commanding ControlStation to the one identified.
   * \details If another ControlStation was set as the commanding one,
   *  unsetCommandingControlStation() is called before the change.
   * \sa commandingControlStation()
   * \param[in] id The ID of the ControlStation which should be set as the 
   *  commanding one. 
   * \return TRUE if successfull; FALSE otherwise.
   */
  bool setCommandingControlStation(NodeId const& id);
  
  /** \brief Unset the commanding ControlStation.
   */
  bool unsetCommandingControlStation();
  
  /** \brief The status of a/the COM link connection to the commanding ControlStation.
   * \details This is a convenience function to check the COM connectedness of
   * the current commanding ControlStation.
   * \return TRUE if the commandingControlStation is COM-connected to this
   * Vehicle; FALSE otherwise. */
  bool isConnectedToCommandingControlStation() const;
  
  /** \brief The list of possible routing methods for path computations. */
  enum class Routing : uint {
     /** \brief Direct to the destination.
      * \details The route will be a single waypoint at the given destination, 
      * i.e. a straight line segment from the current location to the destination.
      * 
      * \note There is \e no feasibility check performed, i.e. the path could 
      * clipp obstacles.
      */
     directTo,
     
     /** \brief Using the route graph for the majority of the path.
      * \details The computed path will consist of three segements:
      * - the (shortest) directTo path from the current location to a
      *   vertex of the route graph
      * - the (shortest) directTo path from the destination to a vertex
      *   of the route graph
      * - the shortest path on the route graph combining the two indentified
      *   vertices above.
      *
      * \note There is \e no feasibility check performed, i.e. the path could 
      * clipp obstacles.
      */ 
     viaRouteGraph
  };
  
  /** \brief Distance threshold in meters below which Routing::directTo is used. */
  constexpr static double ROUTING_DIRECT_TO_THRSHLD = 10.0; // FIXME: Magic Numbers
  
  /** \brief Command the Vehicle to a destination.
   * 
   * \param[in] destination The target destination.
   * \param[in] routing (optional) The routing method to use to compute the path
   *  to the destination.
   */
  virtual void goTo(Waypoint const & destination, Routing routing = Routing::viaRouteGraph);


  
  /** \brief Set the target element via its ElementId.
   * \details Sets the target ID stored in the ownshipBlip. This is the entry 
   * point for processing after a target selection through a GCS operator. */
  virtual void setTargetId(ElementId const& id);
  
protected:
  
  /** \brief Constructor optimized for d-pointer inheritance.
   * See http://qt-project.org/wiki/Dpointer for more explanations.
   */
  explicit Vehicle ( VehiclePrivate& dd/*, Node* parent = nullptr*/ );
  
  virtual void initializeItem();
  virtual void unInitializeItem();
  
  /** \brief Execute an already constructed flightplan.
   * 
   * This function is the last step in dealing with flight plans. It needs to 
   * be implemented by all derived vehicles and should result in the provided
   * plan beeing immediately executed.
   * 
   * On successfull completion of the flight plan, flightplanFinished should be
   * emitted.
   */
  virtual void executeFlightplan(Flightplan const & plan)=0;
  
  
  /** \brief Command the Vehicle to stop at it's current location.
   * \details This slot is triggered upon entry of the loiter state.
   */
  virtual void stopMotion()=0; 
  
  /** \brief Process incomming Joystick commands.
   * \note When implementing this function, make sure to check whether the
   * Vehicle is in joystickControl.
   */
  virtual Q_INVOKABLE void processReceivedMessage(NodeId const & sender, IDL::Messages::JoystickCommand const & message)=0;

  /** \brief Process incomming commands.
   * 
   * This funciton handles incomming self-contained commands for the Vehicle 
   * level, i.e.
   * - IDL::Commands::CommandTypes::V_ENABLE_JOYSTICK_CONTROL
   * 
   * Being self-contained, these commands do not require any additional 
   * supplementing data to be processed as their meaning is unambigeous and 
   * known a priory by all parties involved. 
   * 
   */
  virtual Q_INVOKABLE void processReceivedMessage(NodeId const & sender, IDL::Messages::ModeChangeCommand const & message);
  
  /** \brief Process incomming external plan commands.
   * 
   * Unlike a ModeChangeCommand, an external plan command requires some more 
   * context, mainly the actual plan to be executed. As such, there is a 
   * dedicated message for this command type which provides this supplementing
   * data.
   * This function handles these commands.
   * 
   */
  virtual Q_INVOKABLE void processReceivedMessage(NodeId const & sender, IDL::Messages::ExternalPlanCommand const & message);
  
  /** \brief Process incomming Status messages.
   * \details The function takes care of vehicle specific actions associated 
   * with Status messages from other Nodes. Primarily this means duplicating the
   * target settings from the relevant ControlStation.
   */
  virtual Q_INVOKABLE void processReceivedMessage(NodeId const & sender, IDL::Messages::Status const & message);

  /** \brief Process incomming DualHopGuidance messages.
   * \details TBD
   */
  virtual Q_INVOKABLE void processReceivedMessage(NodeId const & sender, IDL::Messages::DualHopGuidance const & message);
  
private:
  Q_DISABLE_COPY(Vehicle)
  Q_DECLARE_PRIVATE(Vehicle);

};

#endif // VEHICLE_H
