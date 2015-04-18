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

#ifndef CONTROLSTATION_PRIVATE_H
#define CONTROLSTATION_PRIVATE_H

#include "node_p.h"

#include <boost/graph/breadth_first_search.hpp>

#include "HMI/humanmachineinterface.h"
#include "DDS/addressedtopic.h"
#include "DDS/reliabletopic.h"

class ControlStation;

class ControlStationPrivate : public NodePrivate {
private:
  Q_DECLARE_PUBLIC(ControlStation);
  ControlStation* const q_ptr;  
  
public:
  ControlStationPrivate( ControlStation* q
                        ,ControlStation::Settings* const settings
                        /*,Node* parent*/  );
  virtual ~ControlStationPrivate();
     
  ControlStation::Settings* const xmlSettings;
  
  void initializeDatalink();  
  
  void unInitializeControlStation();

  /** \brief Compute the list of blips to exclude and update the R/F coverage map.
   */ 
  virtual void updateRfConnectedArea();
  
  /** \brief Update the R/F coverage map for the primary Vehicle.
   * 
   * \sa VehiclePrivate::updateRfConnectedArea()
   * \param[in] excludedNodes A list of nodes who are considered to be 
   * disconnected, no matter their actual R/F state.
   * */
  virtual void updateRfConnectedArea(QList<NodeId> const & exludedNodes);
  
  
  

  
  
  /** \brief Qt Connection to keep track of the R/F connected area for the 
   * primaryVehicleBlip.
   *\sa updateRfConnectedArea */
  QMetaObject::Connection qtCon_trackRfConnectedAreaOfPrimaryVehicle;
  
  
  /** \brief Create and add a new blip from the given data.
   * \details Reimplements a function from NodePrivate */
  virtual void initializeBlip( const NodeId& id, const Node::Type& type
  , QString const & name, QStringList const & propertyList, Blip* blip = nullptr );
  
  
  /** \brief Delete the blip for the given id. */
  virtual void deleteBlip(NodeId const &id);
  
#ifdef WITH_GRAPHICS   
  /** \brief React to a Blip beeing dropped in the Arena. */
  void processDroppedBlip(NodeId const & id, QPointF scenePosition);
  
  /** \brief React to a Blip being moved in the Arena. */
  void processDraggedBlip(QPointF scenePosition);
  void processDraggedBlip(NodeId const & id, QPointF scenePosition);
     
  /** \brief Update the R/F polygon for the indicated node at the last 
   * Waypoint of the current Flightplan.
   * 
   * This function computes the R/F polygon at the last Waypoint of the current
   * Flightplan for the indicated Node based on the corresponding blipData. If 
   * the R/F polygon has successfully been updated,the function returns TRUE.
   * 
   * \param[in] blip The blip for which to compute the R/F polygon.
   * \param[in] minDistance (optional) The minimum distance the blip needs to
   *  have moved before a new R/F polygon is computed.
   * \return TRUE, if a new R/F polygon has been computed; FALSE otherwise.
   */
  bool updateLastWpRfPoly(Blip* const blip, double const & minDistance = 1.0 ); 
    
  /** \brief The "database" holding all the Blip related data. */
  std::unordered_map<NodeId,QGraphicsItem*> lastWpRfPolygonData;
  
  /** \brief The Blip currently being dragged by the mouse. */
  Blip* draggedBlip = nullptr;

#endif // WITH_GRAPHICS
  
  
  /** \brief Command available Blips to positions in a Formation. */
  void centrallyAssingFormationPositions();
  
  template< typename Vertex
          , typename Container >
  class DiscoverySequenceRecorder : public boost::default_bfs_visitor
  {
  public:
    DiscoverySequenceRecorder(Container& container)
    :sequence(container){};
    
    template < /*typename Vertex,*/ typename Graph >
    void discover_vertex(Vertex u, const Graph & g)
    { sequence.push_back(u); };
    
    //NOTE: Visitors are passed by value, so no internal state will be 
    // Using pointers can obvioulsy cicumvent this. Also, see the note at 
    // http://www.boost.org/doc/libs/1_55_0/libs/graph/doc/breadth_first_search.html#1
    Container& sequence;
  };
  
  
  /** \brief The Human-Machine interface of the control station. 
   *
   * This interface funcitons as the main physical I/O to the human operator
   * using the control station.
   */
  HumanMachineInterface* hmi = nullptr;
  
  /** \brief The blip that is currently under primary (manual) control. */
  Blip* primaryNodeBlip = nullptr;
  
  
  //
  // Topic: Joystick Command
  //
  
  /** \brief The topic for exchanging State messages. */
  AddressedTopic<IDL::Messages::JoystickCommand, ControlStation, TopicOptions::SendReceive>* joystickTopic = nullptr;

  /** \brief A pointer to the struct used to send State messages. */
  IDL::Messages::JoystickCommand* joystickMessage = nullptr;
  
  /** \brief Send the joystickData to the currently controlled blip. */
  void sendJoystickCommand(IDL::Data::JoystickData const & joystickData);
  
  //
  // Topic: Mode Change Command
  //
  
  /** \brief The topic for exchanging Command messages. */
  AddressedTopic<IDL::Messages::ModeChangeCommand, ControlStation, TopicOptions::SendReceive>* commandTopic = nullptr; 
  
  /** \brief A pointer to the struct used to send Command messages. */
  IDL::Messages::ModeChangeCommand* commandMessage = nullptr;
  
  /** \brief Send a command to a Node.
   * \param[in] command The command to send.
   * \param[in] id The NodeId of the node to be commanded.
   */
  void sendCommand(IDL::Commands::CommandTypes const & command, NodeId const & id);
  
  
  
  //
  // Topic: External Plan Command
  //
  
  /** \brief The topic for exchanging externally provided Flightplan command messages. */
  ReliableTopic<IDL::Messages::ExternalPlanCommand, ControlStation, TopicOptions::SendReceive>* planCommandTopic = nullptr; 
//   AddressedTopic<IDL::Messages::ExternalPlanCommand, ControlStation, TopicOptions::SendReceive>* planCommandTopic = nullptr; 

  /** \brief A pointer to the struct used to send External Plan Command messages. */
  IDL::Messages::ExternalPlanCommand* planCommandMessage = nullptr;
  
  
  
  //
  // Topic: Dual Hop Guidance
  //
  
  /** \brief The topic for exchanging DualHopGuidance messages.  */
  BroadcastTopic<IDL::Messages::DualHopGuidance,ControlStation,TopicOptions::Receive>* dualHopGuidanceTopic = nullptr;
  
 
  
  

protected:
    void d();
};

#endif // CONTROLSTATION_PRIVATE_H
