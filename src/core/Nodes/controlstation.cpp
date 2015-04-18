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

#include "controlstation.h"
#include "controlstation_p.h"

#include <boost/graph/graph_utility.hpp> // get print_graph
#include <boost/property_map/compose_property_map.hpp>
#include <boost/graph/breadth_first_search.hpp>
#include <QtCore/qtimer.h>

#ifdef WITH_GRAPHICS
  #include <QtWidgets/QMenu>
#endif // WITH_GRAPHICS

//############################################################################//
//                                                                            //
//  ControlStationPrivate                                                     //
//                                                                            //
//############################################################################//


ControlStationPrivate::ControlStationPrivate ( ControlStation* q
                                        ,ControlStation::Settings* const settings
                                        /*,Node* parent*/)
  :NodePrivate(q,settings)
  ,q_ptr ( q )
  ,xmlSettings(settings)
{ //NOTE: Do _NOT_ use q_ptr or q in here, it _WILL_ break things!
}

ControlStationPrivate::~ControlStationPrivate()
{
}

void ControlStationPrivate::unInitializeControlStation()
{
  // undo whatever ControlStation::initializeItem() did...
  Q_Q(ControlStation); Q_ASSERT(q);
  
  { /** - Uninitialize the HumanMachineInterface: a parent/child relationship 
     * deletes hmi */
//     delete hmi;
//     hmi = nullptr;
  }
  
  
  {/// - Uninitialize Joystick topic
    joystickMessage = nullptr;
    
    delete joystickTopic;
    joystickTopic = nullptr;
  }
}

void ControlStationPrivate::updateRfConnectedArea()
{
  Q_Q(ControlStation); Q_ASSERT(q);
  
  QList<NodeId> excludes;
  
  if( primaryNodeBlip != nullptr )
  { excludes << primaryNodeBlip->id(); }

#ifdef WITH_GRAPHICS
  if( draggedBlip != nullptr )
  { excludes << draggedBlip->id(); }
#endif // WITH_GRAPHICS  
   
  updateRfConnectedArea(excludes);
}

void ControlStationPrivate::updateRfConnectedArea(QList<NodeId> const & exludedNodes)
{
  Q_Q(ControlStation); Q_ASSERT(q);
  
  QList<NodeId> relayBlips = comRelayNodes( q->id(), exludedNodes );  
  
  for( std::pair<const NodeId,Blip*> pair : blipData )
  { 
    
    /** \note The logic check on mode().contains(IDL::Data::STATE_ID_N_OPERATIONAL)
     * removes joining or leaving Nodes from the list of Blips that are 
     * considered as providing a link to the commanding ControlsStation.
     * However, this results in either the need to call updateRfConnectedArea 
     * not only on changes in the COM graph, but also when other Blips receive 
     * the IDL::Commands::N_LEAVE_NETWORK command, or the acceptance of the fact
     * that the connectednessToCommandingControlStation could be slightly 
     * outdated. */
    
    bool isInRfArea = relayBlips.contains(pair.first);
//       and !pair.second->mode().contains(IDL::Data::STATE_ID_N_STANDBY)
//       and !pair.second->mode().contains(IDL::Data::STATE_ID_N_LEAVING)
    
    pair.second->setRfRelayFlag(isInRfArea);
  }   
}





void ControlStationPrivate::initializeBlip( const NodeId& id, const Node::Type& type
  , QString const & name, QStringList const & propertyList, Blip* blip)
{
  NodePrivate::initializeBlip(id,type,name,propertyList,blip);
 
  // 'blip' could be a nullptr and since it is passed by value it still could be
  // even though NodePrivate::initializeBlip has created a new Blip.
  
  if( blip == nullptr )
  { blip = blipData.at(id); } 

#ifdef WITH_GRAPHICS
  
  ControlStation::connect(blip, &Blip::flightplanChanged,
    [=](){ updateLastWpRfPoly(blip); } );

#endif // WITH_GRAPHICS
  
}

void ControlStationPrivate::deleteBlip(NodeId const &id)
{
  Q_Q(ControlStation); Q_ASSERT(q);
  if( primaryNodeBlip != nullptr and id == primaryNodeBlip->id() )
  { q->unsetPrimaryNode(); }
  NodePrivate::deleteBlip(id);
}

void ControlStationPrivate::sendJoystickCommand(IDL::Data::JoystickData const & joystickData)
{
  if(nullptr != primaryNodeBlip)
  { // the control station is indeed controlling a blip
  
    joystickMessage->addressee = primaryNodeBlip->id();
    joystickMessage->joystick = joystickData;
    
//     Q_Q(ControlStation); Q_ASSERT(q);
//     qDebug() << q->qualifiedName() << ": sending Joystick data.";
    
    joystickTopic->send();
  }
}

void ControlStationPrivate::sendCommand( IDL::Commands::CommandTypes const & command
                                       , NodeId const & id)
{
  Q_Q(ControlStation); Q_ASSERT(q);
  
//   qDebug() << q->qualifiedName() << ": Sending" << command << "command to" << id;
  
  if( id != ownshipId and ownshipBlip->mode().contains(IDL::Data::STATE_ID_N_STANDBY) )
  {
    qWarning() << q->qualifiedName() << ": Cannot send commands while in STANDBY."
      << "(Command"<< command << "to"<< q->blip(id).name() << ")";
    return;
  }
  
  
  
  switch( command )
  {
    case IDL::Commands::V_ENABLE_JOYSTICK_CONTROL:
    {    
      q->setPrimaryNode(id);
      break;
    }
    case IDL::Commands::N_JOIN_NETWORK : //NOTE: Fall-through intended
    case IDL::Commands::V_EXECUTE_EXTERNAL_PLAN: //NOTE: Fall-through intended
    case IDL::Commands::V_LOITER:
    {
     if( primaryNodeBlip and id == primaryNodeBlip->id() )
     { // we are sending a JOIN command to the currentlyControlledBlip
       q->unsetPrimaryNode();
     }
     break;
    }
    default: break;
  }

  commandMessage->addressee = id;
  commandMessage->command = command;
    
  if( id != q->id() )
  { // trying to command somebody else => we can send a message
    commandTopic->send();
  }
  else
  { // trying to command self => we cannot send a message
    //NOTE: sending a message to self won't have a result as those messages are
    // ignored by the Datalink. So simply bypass it and trigger the processing
    // directly:
    q->processReceivedMessage(q->id(), *commandMessage);  
  }
}

#ifdef WITH_GRAPHICS
void ControlStationPrivate::processDroppedBlip(NodeId const & id, QPointF scenePosition)
{
  Q_Q(ControlStation); Q_ASSERT(q);
  if( blipExists(id) and q->blip(id).isConnectedToHost())
  {
    
    
    Waypoint destinaton;
    destinaton.setPosition(scenePosition.x()
                          ,scenePosition.y()
                          ,q->blip(id).routeAltitude());

    Flightplan fp;
    fp.append(destinaton);
    
    q->sendExternalPlanCommand(fp,id); 
  }
}

void ControlStationPrivate::processDraggedBlip( QPointF scenePosition)
{
  Q_Q( ControlStation ); Q_ASSERT(q);
//   qDebug() << q->qualifiedName() << ": moving Blip to" << scenePosition;
  
  if(draggedBlip != nullptr)
  {
    // compute an R/F polygon
    Geometry::Polygon_2D::ring_type ring;
    if( computeRfPolygon(Geometry::Point_2D(scenePosition.x(),scenePosition.y()), ring) )
    { draggedBlip->setLastWpRfPolygon(ring); } 
  }
}

void ControlStationPrivate::processDraggedBlip(NodeId const & id, QPointF scenePosition)
{
  Q_Q(ControlStation); Q_ASSERT(q);
  if( draggedBlip == nullptr )
  { q->setDraggedBlip(id); }
  
  processDraggedBlip(scenePosition);  
}


bool ControlStationPrivate::updateLastWpRfPoly(Blip* const blip, double const & minDistance)
{
  Q_ASSERT( blip );
  Q_Q(Node); Q_ASSERT(q);
  
  if( blip->flightplan().waypoints.length() == 0 )
  { 
    /** \internal If a vehicle has rached the last Waypoint of a Flightplan, it
     * sends out a Flightplan message with an empty Flightplan, i.e. one with
     * length zero. */
    qDebug() << q->qualifiedName() << ": No last Waypoing in Flightplan of"
       << blip->name() << ". Skipping R/F polygon computation.";
    
    blip->lastWpRfPolygonItem()->hide();   
    return false; 
  }
  
  // check whether the node has moved more than the required minDistance 
  Waypoint lastWp = blip->flightplan().waypoints[blip->flightplan().waypoints.length()-1];
  Geometry::Point_2D lastWpPos2D(lastWp.position().x,lastWp.position().y);

  Geometry::Polygon_2D::ring_type rfRing;
  
  if( computeRfPolygon(lastWpPos2D,rfRing) )
  {
    qDebug() << q->qualifiedName() << ": Updated R/F polygon at the last Waypoint of"
      << blip->name();
    blip->setLastWpRfPolygon(rfRing);  
    blip->lastWpRfPolygonItem()->show();
    return true;
  }
  else 
  { 
    qCritical() << q->qualifiedName() << ": Couldn't update the R/F polygon for the last Waypoint of"
      << blip->name();
    blip->lastWpRfPolygonItem()->hide();
    return false; 
  }
}



#endif // WITH_GRAPHICS

void ControlStationPrivate::centrallyAssingFormationPositions()
{
  Q_Q( ControlStation ); Q_ASSERT(q);
  
  qDebug() << q->qualifiedName() 
   << ": Centrally assigning Blips to vertex positions of the Fomration Graph.";
  
  using namespace boost;
  
  // The number of positions that need to be assigned to a Blip
  int numPositions = num_vertices(formationGraph); 
  
  //FIXME: STATE_ID_V_LOITER describes two states: autonomousLoiter and pilotedLoiter 
  std::list<Blip*> availableBlips = getBlipsByMode(IDL::Data::STATE_ID_V_LOITER); //FIXME: Magic Numbers
  
  // The number of Blips available to this GCS.
  int numBlips = availableBlips.size();
  
  qDebug() 
  << q->qualifiedName() << ": Formation Positions (including self):" << numPositions 
  << ", Available Blips (not including self) :" << numBlips;
  
  Formation formation;
  
  //
  // Translate the formationGraph into a Formation object.
  //
  /** \internal ... which might again be a graph in a future a long long time 
   * away... */
  {
    auto nodeIdMap = get(vertex_id, formationGraph);
    auto locationIdMap = get(vertex_locationId, formationGraph);
    auto locationMap = boost::make_compose_property_map(location_pMap, locationIdMap);
      
      
    // breadth first search through the formation graph to determine the order 
    // of vertex assignment.
    /** \internal
    * Breadth first should ensure that in cases with more formation positions
    * than available blips the 'best that can be done' is achieved.
    * The benefit of breadth first vs. depth first is IMHO as follows:
    * If both methods start at the GCS, both result in the same order along the
    * ingress path (as there is only one choice). Reaching the obstacle, 
    * breadth first should populate both paths (CW and CCW) at the same time.
    * I consider this beneficial over a depth first (which would only populate 
    * the CW or CCW path) as 
    * 1) depth first is 'slower' (as presmable the Blips have to travel further -
    * if they start out from the GCS)
    * 2) In cases of insufficient Blips, the operator has the choice of initially 
    * going CW or CCW and then needs to repositing Blips. In a depth first 
    * approach he either goes with whatever the algorithm chose (CW or CCW) and
    * in cases where that doesn't match the operators prefered path, relocations 
    * of Blips have to happen immediately. */
  
    using namespace boost;
    using Vertex = FormationGraph::Vertex_Descriptor;
    
    // HACK: This is by no means a 'safe' piece of code, but it should work:
    // The formation graph is essentially formed when the ingress and the 
    // loop sections of it are combined. This (currently) is done while starting
    // with the ingress path, with in turn starts at the GCS... as such, the 
    // first vertex in the graph should (!) be the GCS vertex...
    Vertex gcsVertex = vertex(0,formationGraph); 
    
    // .. but just to be sure, let's check that.
    Q_ASSERT( get(vertex_id, formationGraph, gcsVertex) == q->id());
    
    std::list<Vertex> discoverySequence;
    
    DiscoverySequenceRecorder<Vertex, std::list<Vertex>> vis(discoverySequence);
      
    breadth_first_search(formationGraph, gcsVertex ,visitor(vis) );
    
    for(Vertex v : discoverySequence )
    {
      Geometry::Point_2D position;
      auto nodeId = get(nodeIdMap, v);
      
      auto locationId = get(locationIdMap, v);
      if( locationId != NULL_LOCATION_ID )
      { position = get(locationMap,v); }
      else
      { // this is a "dynamic" formation position, tied to a Node. As such this 
        // is already somewhat assigned to that Node...
        Q_ASSERT(blipExists(nodeId));
        position = blipData.at(nodeId)->position2D();
      }
      formation.push_back(FormationPairing(nodeId,position));
    }
    
  }
  
  
  //
  // Do the assignment of Nodes to Positions
  //
  
  for( FormationPairing& pairing : formation )
  {
   if(availableBlips.size() == 0 )
   { break; }
   
   if( pairing.nodeId() == NULL_NODE_ID )
   {
     std::list<Blip*>::iterator closestBlip_it = availableBlips.end();
     Blip*  closestBlip = nullptr;
     double minDistance = std::numeric_limits< double >::max();
     
     for( std::list<Blip*>::iterator b_it = availableBlips.begin(); b_it!=availableBlips.end(); ++b_it )
     {
      double distance =  boost::geometry::distance(pairing.position(), (*b_it)->position2D() );
      if( distance < minDistance ) 
      {
        minDistance = distance;
        closestBlip_it = b_it;
        closestBlip = *b_it;
      }
     }
   
     qDebug() << "Assigning"<< closestBlip->name() << "to" << pairing.position();
   
     pairing.setNodeId((*closestBlip_it)->id());
     availableBlips.erase(closestBlip_it);  
   }
  }
  
  {//DEBUG print the formation
    qDebug() << q->qualifiedName() << ": Formation assigment:";
    int pos = 0;
    for( FormationPairing pairing : formation )
    {     
      qDebug() << "Pos." << pos++ << ":" 
      << ( pairing.nodeId() == NULL_NODE_ID ? "(none)" : blipData.at(pairing.nodeId())->name() )
      << "@" << pairing.position() ;
    }
  }
    
  //
  // Command the Blips to their positions
  //
  
  for( FormationPairing pairing : formation )
  {
    if( pairing.nodeId() == q->id() or pairing.nodeId() == NULL_NODE_ID )
    { continue; }
    
    Waypoint destinaton;
    destinaton.setPosition(pairing.position().x(),
                           pairing.position().y(),
                           ownshipBlip->routeAltitude() );

    Flightplan fp;
    fp.append(destinaton);    
    q->sendExternalPlanCommand(fp,pairing.nodeId()); 
  }
    
  
}


void ControlStationPrivate::initializeDatalink()
{
  Q_Q( ControlStation ); Q_ASSERT(q);
    
  /// - initialize the Joystick Topic
  joystickTopic = new AddressedTopic<IDL::Messages::JoystickCommand, ControlStation,TopicOptions::SendReceive>
    ("JoystickCommand",dds,q);
  joystickTopic->initialize();  
  joystickMessage = joystickTopic->message();  
  
  /// - initialize the Mode Change Command Topic
  commandTopic = new AddressedTopic<IDL::Messages::ModeChangeCommand, ControlStation, TopicOptions::SendReceive>
    ("Command", dds, q);
  commandTopic->initialize();
  commandMessage = commandTopic->message();
  
  /// - initialize the External Plan Command Topic
//     planCommandTopic = new AddressedTopic<IDL::Messages::ExternalPlanCommand, ControlStation, TopicOptions::SendReceive>
  planCommandTopic = new ReliableTopic<IDL::Messages::ExternalPlanCommand, ControlStation, TopicOptions::SendReceive>
    ("PlanCommand", dds, q);
  planCommandTopic->initialize();
  planCommandMessage = planCommandTopic->message();
  
  /// - initialize the Dual-Hop Guidance Topic
  dualHopGuidanceTopic = new BroadcastTopic<IDL::Messages::DualHopGuidance,ControlStation,TopicOptions::Receive>
    ("Dual-Hop Guidance", dds, q);
  dualHopGuidanceTopic->initialize();
   
}



//############################################################################//
//                                                                            //
//  ControlStation                                                            //
//                                                                            //
//############################################################################//

/** \note This constructor does NOT do any work!
 * All ControlStation level construction needs to happen in 
 * ControlStation(ControlStationPrivate & dd, ...)!
 */
ControlStation::ControlStation( Settings* const settings
                              /*,Node* parent*/)
  :ControlStation(*new ControlStationPrivate(this,settings/*,parent*/)/*,parent*/)
{}

ControlStation::ControlStation( ControlStationPrivate& dd
                                /* ,Node* parent*/ )
  : Node(dd/*, parent*/) // ControlStations are Nodes and hence don't have parents
{
  Q_D(ControlStation); Q_ASSERT(d);
  d->hmi = new HumanMachineInterface(&(d->xmlSettings->hmi()),this);
  
  
  /** \internal 
   * This will only indicate the status of the ControlStation, any derived classes
   * will not be constructed at this point!
   */
  d->setStatus(Status::Constructed); 
}


ControlStation::~ControlStation()
{
  Q_D(ControlStation); Q_ASSERT(d);
  d->unInitializeControlStation();
}


Node::Type ControlStation::type() const
{ return IDL::Data::NODE_TYPE_GCS; }


void ControlStation::initializeItem()
{
  Node::initializeItem();
  // do local work below ...
  
  Q_D(ControlStation); Q_ASSERT(d);

  
  
  d->hmi->initialize();
//   d->initializeJoystick();
  
  d->initializeDatalink();
  
  connect(this, &ControlStation::blipAdded,
    [d](NodeId const & id) { d->joystickTopic->registerAddressee(id);});
  connect(this, &ControlStation::aboutToDeleteBlip,
    [d](NodeId const & id) { d->joystickTopic->unregisterAddressee(id);});
  
  //NOTE: as this "connection" uses anonymous lambdas, it might be tricky
  // to disconnect. Hence some more work might be necessary in sendJoystickCommand
  // in order to only send stuff when one actually wants to.
  connect(d->hmi, &HumanMachineInterface::joystickChanged,
    [d](IDL::Data::JoystickData const data){d->sendJoystickCommand(data);} );

       
  connect(this, &ControlStation::blipAdded,
    [d](NodeId const & id) { d->commandTopic->registerAddressee(id);});
  connect(this, &ControlStation::aboutToDeleteBlip,
    [d](NodeId const & id) { d->commandTopic->unregisterAddressee(id);});
  

#ifdef WITH_GRAPHICS  
  {/// - modify the paletts from it's Node-defaults
    Q_D(ControlStation); Q_ASSERT(d);
    
    using namespace Graphics_2D;   
       
    d->palette.blip.rfPoly_ownship[IDL::Data::STATE_ID_N_OPERATIONAL].brush  = Brushes::FloodFill(Qt::darkGreen,30);
//     d->palette.blip.rfPoly_ownship[IDL::Data::STATE_ID_V_AUTONOMOUS].brush  = Brushes::FloodFill(Qt::darkGreen,30);
    d->palette.blip.rfPoly_ownship[IDL::Data::STATE_ID_V_PILOTED].brush  = Brushes::FloodFill(Qt::darkGreen,30);
      
    d->palette.blip.rfPoly_ownship[IDL::Data::STATE_ID_N_STANDBY].brush = Brushes::FloodFill(Qt::lightGray,30);
    
    d->palette.blip.rfPoly_ownship[IDL::Data::STATE_ID_N_JOINING].brush = Brushes::FloodFill(Qt::green,30);
    d->palette.blip.rfPoly_ownship[IDL::Data::STATE_ID_N_LEAVING].brush = Brushes::FloodFill(Qt::green,30);
    
    //NOTE: "Joystick" additionally has FOV enabled and uses fovPoly_joystick.
    d->palette.blip.rfPoly_ownship[IDL::Data::STATE_ID_V_JOYSTICK].brush = Brushes::FloodFill(Qt::darkGreen,30);
    d->palette.blip.rfPoly_ownship[IDL::Data::STATE_ID_V_PILOTED].brush = Brushes::FloodFill(Qt::darkGreen,30);
    
//     d->palette.blip.rfPoly_ownship[IDL::Data::STATE_ID_D_NO_LINK]
  }
  { // Drag and Drop support
    Q_D(ControlStation); Q_ASSERT(d);
    connect(d->environmentItem, &Graphics_2D::Arena::blipDropped,
            [=](NodeId id, QPointF scenePos){ d->processDroppedBlip(id,scenePos); });
    
    connect(d->environmentItem, &Graphics_2D::Arena::blipMoved,
            [=](QPointF position){ d->processDraggedBlip(position);});
    
    connect(d->environmentItem, &Graphics_2D::Arena::blipEntered,
            [=](NodeId id, QPointF position){d->processDraggedBlip(id, position);});
  }
#endif // WITH_GRAPHICS  
 
//   {/// - connect the formation realization
//     connect(this, &ControlStation::comFormationProposed,
//             this, &ControlStation::realizeFormation);
//   }
 
}

void ControlStation::startItem()
{
  Node::startItem();
  // do local work below ...
  
  Q_D(ControlStation); Q_ASSERT(d);
  d->hmi->run();
  
}

void ControlStation::stopItem()
{
  Q_D(ControlStation); Q_ASSERT(d);
  d->hmi->halt();
  
  // do local work above...
  Node::stopItem();
}

void ControlStation::unInitializeItem()
{
  Q_D(ControlStation); Q_ASSERT(d);
  d->unInitializeControlStation();
  
  // do local work above...
  Node::unInitializeItem();
}


// void ControlStation::setJoystickConfiguration ( const XML::SAI::Configuration::JoystickSequence& joysticks )
// {
//   Q_D(ControlStation); Q_ASSERT(d);
//   d->xmlJoysticks = joysticks;
// }


NodeId ControlStation::primaryNode() const
{
  Q_D(const ControlStation); Q_ASSERT(d);
  if( nullptr != d->primaryNodeBlip )
  { // this control station is currently controlling a blip
    return d->primaryNodeBlip->id();
  } 
  // else
  return id();  // no blip is currently controlled
}

bool ControlStation::setPrimaryNode ( const NodeId& id )
{
  Q_D(ControlStation); Q_ASSERT(d);
  
  if( d->blipExists(id) )
  {
  
    d->primaryNodeBlip = &(blip(id)); 
    d->updateRfConnectedArea();
      
    qDebug() << qualifiedName() << ": Changed the primarily controlled node to" << blip(primaryNode()).name();
    emit primaryNodeChanged(id);
    return true;
    
    /** \todo this doesn't seem to work... though it would be neat if it would */
//     connect(d->primaryNodeBlip, &Blip::modeChanged,
//       [d, this](){
//         if( d->primaryNodeBlip->mode().contains(IDL::Data::STATE_ID_V_JOYSTICK) )
//         { this->unsetPrimaryNode(); }
//       }
//     );
    
    
  }
  else
  {
    qCritical() << qualifiedName() << ": Did not change the primarily controlled Node.";
    return false;
  }
}


void ControlStation::unsetPrimaryNode()
{
  Q_D(ControlStation); Q_ASSERT(d);
  d->primaryNodeBlip = nullptr;
  
//   disconnect( d->qtCon_trackRfConnectedAreaOfPrimaryVehicle );
  d->updateRfConnectedArea();
  
  qDebug() << qualifiedName() << ": Unset the primarily controlled node.";
  emit primaryNodeChanged(id());
}

#ifdef WITH_GRAPHICS
void ControlStation::setDraggedBlip(const NodeId& id)
{
  Q_D(ControlStation); Q_ASSERT(d);
  
  if( blipExists(id) )
  {
    d->draggedBlip = &blip(id);
    d->updateRfConnectedArea();
    
    // prepare the draggedBlipRfRangeItem for placement/motion during the 
    // (most likely) following Blip movement.
    
    d->draggedBlip->lastWpRfPolygonItem()->setGeometry(d->draggedBlip->rfPolygon());
    d->draggedBlip->lastWpRfPolygonItem()->show();
  }
  else
  { d->draggedBlip = nullptr; }
}

void ControlStation::unsetDraggedBlip()
{
  Q_D(ControlStation); Q_ASSERT(d);
  
  d->draggedBlip->lastWpRfPolygonItem()->hide();
  d->draggedBlip = nullptr;
  d->updateRfConnectedArea();
}




void ControlStation::buildElementContextMenu(QMenu* parentMenu, const ElementId& id)
{
  Q_D( ControlStation ); Q_ASSERT(d);
  
  if( !obstacleExists(id) )
  { return; }
  
  Node::buildElementContextMenu(parentMenu, id);
  
    
  
  if( parentMenu->actions().size() > 0 )
  { parentMenu->addSeparator(); }
  
  // 
  // persistent menu actions
  //
  
  
  //
  // temporary menu actions
  //
  
  QAction* action = nullptr;
  
  
  if( id == targetId() )
  {
            
    action = new QAction(parentMenu);
    action->setText(Node::tr("&Propose Formation"));
    
    action->setEnabled(!d->computingComFormationProposal and !d->isFormationGraphValid);
    
    parentMenu->addAction(action);
    
    connect(action, &QAction::triggered,
      [=](){ d->proposeComFormation(id, ownship().id()); } );
    
    buildFormationContextMenu(parentMenu);    
    
  }
}



#endif // WITH_GRAPHICS


void ControlStation::setTargetId(const ElementId& id)
{
  Node::setTargetId(id);  
  Q_D( ControlStation ); Q_ASSERT(d);
  
  ElementId target = id;
  NodeId    source = this->id();
  d->createComCorridor(target,source);
//   d->proposeComFormation(target, source);
}


void ControlStation::unsetTarget()
{
  Q_D( ControlStation ); Q_ASSERT(d);
 
  d->unsetComFormation();
  d->unsetComCorridor();
  
  Node::unsetTarget();
}

/** \internal This function realizes the NodePrivate::formationGraph if it is 
 * valid. */
void ControlStation::realizeFormation()
{
  Q_D(ControlStation); Q_ASSERT(d);
  
  if( ownship().mode().contains(IDL::Data::STATE_ID_N_STANDBY) )
  {
    qDebug() << qualifiedName() << ": Connot realize a formation from standby.";
    return;
  }
  
  if( !(d->isFormationGraphValid) )
  { 
    qDebug() << qualifiedName() << ": Cannot realize an invalid formation graph.";
    return;
  }
  
  d->centrallyAssingFormationPositions();
}


void ControlStation::sendCommand ( const IDL::Commands::CommandTypes& command, const NodeId& id )
{
  Q_D(ControlStation); Q_ASSERT(d);
  d->sendCommand(command,id);
}



void ControlStation::sendExternalPlanCommand(const IDL::Data::Flightplan& plan, const NodeId& id)
{

  Q_D(ControlStation); Q_ASSERT(d);
  
  d->planCommandMessage->addressee = id;
  d->planCommandMessage->plan = plan;
  
  if( id != this->id() and id != NULL_NODE_ID)
  { // trying to command somebody else => we can send a message
    
    qDebug() << qualifiedName() << ": Sending an external plan command to" 
      << (blipExists(id) ? blip(id).name() : "(--)");
    
    if( d->primaryNodeBlip and id == d->primaryNodeBlip->id() )
    { unsetPrimaryNode(); }   
      
      
    if( d->planCommandTopic->send() == false )
    { 
      qWarning() << qualifiedName() << ": Could not send an external plan command to"
      << (blipExists(id) ? blip(id).name() : "(--)");
    }
  }
  else
  { // trying to command self => we cannot send a message
    //NOTE: sending a message to self won't have a result as those messages are
    // ignored by the Datalink. So simply bypass it and trigger the processing
    // directly:
    processReceivedMessage(this->id(), *(d->planCommandMessage));  
  }
  
  
  
}

#ifdef WITH_GRAPHICS
void ControlStation::buildBlipContextMenu(QMenu* parentMenu, const NodeId& id)
{
  Node::buildBlipContextMenu(parentMenu, id);
    
  // 
  // persistent menu actions
  //
  
  
  //
  // temporary menu actions
  //
  
  if( parentMenu->actions().size() > 0 )
  { parentMenu->addSeparator(); }
  
  /** \todo It would be nice if we could make use of 
   * enumToQString(IDL::Commands::CommandTypes), but that would kill the 
   * keyboard shortcuts, indicated by the \c &, e.g. "&Join" makes "J" the 
   * shortcut for that action. */
   
  QAction* action = nullptr;
  
//   qDebug() << qualifiedName() << ": Creating a Context Menu for" << id;
  
  //
  // Node-level commands
  // ===================
  // Every Blip is a Node, so there is no need to check for the Blip type.
  //
  if(true)  
  {
    action = new QAction(parentMenu);
    action->setText( tr("&Join"));
    connect( action, &QAction::triggered,
            [=](){ sendCommand(IDL::Commands::N_JOIN_NETWORK,id);} );
    parentMenu->addAction(action);
    
    action = new QAction(parentMenu);
    action->setText( tr("&Leave"));
    connect( action, &QAction::triggered,
            [=](){sendCommand(IDL::Commands::N_LEAVE_NETWORK,id);} );
    parentMenu->addAction(action);
  }
  
  //
  // Vehicle-level commands
  // ======================
  // These commands should only be shown if the Blip this context menu is for
  // is of the "Vehicle" type.
  //
  if(blip(id).type()==IDL::Data::NODE_TYPE_VEHICLE) 
  {

    //
    // The plain (state transition) commands.
    //
    
    parentMenu->addSeparator();
    
    action = new QAction(parentMenu);
    action->setText(tr("&Stop Motion"));
    connect(action, &QAction::triggered,
            [=](){ sendCommand(IDL::Commands::V_LOITER,id); } );
    parentMenu->addAction(action);
    
    action = new QAction(parentMenu);
    action->setText(tr("Execute &Plan"));
    connect(action, &QAction::triggered,
            [=](){ sendCommand(IDL::Commands::V_EXECUTE_EXTERNAL_PLAN,id); } );
    parentMenu->addAction(action);
    
    action = new QAction(parentMenu);
    action->setText( tr("Enable Jo&ystick Control"));
    connect( action, &QAction::triggered,
            [=]() { sendCommand(IDL::Commands::V_ENABLE_JOYSTICK_CONTROL,id); } );
    parentMenu->addAction(action);
    
    //
    // Some more elaborate commands, potentially involving data transmission.
    //
    
    parentMenu->addSeparator();
    
    action = new QAction(parentMenu);
    action->setText( tr("&Return To GCS"));
    connect( action, &QAction::triggered,
            [=]()
            {   
                /** \todo This is actually not really smart in real life:
                 * If a vehicle is in trouble, should it really return to the 
                 * Control Station? I mean, really to where the human operators
                 * are? (There was an icident involving a Schiebel S-100 where a
                 * UAV crashed into the GCS van.) */
              
                Waypoint gcsLocation;
                gcsLocation.setPosition(ownship().position3D());
          
                Flightplan fp;
                fp.append(gcsLocation);
        
                sendExternalPlanCommand(fp,id); 
            });
    parentMenu->addAction(action);

    
    //
    // Debug Commands
    //
#ifndef NDEBUG //NOTE: double negative
    {
      parentMenu->addSeparator();
      
      action = new QAction(parentMenu);
      action->setText(tr("Enter Operational Loiter"));
      connect( action, &QAction::triggered,
                [=](){ sendCommand(IDL::Commands::D_V_ENTER_OPERTAIONAL_LOITER,id); });
      parentMenu->addAction(action);
      
      action = new QAction(parentMenu);
      action->setText(tr("Enter Dual Hop Guidance"));
      connect( action, &QAction::triggered,
                [=](){ sendCommand(IDL::Commands::D_V_ENTER_DUAL_HOP,id); });
      parentMenu->addAction(action);
    }
#endif //DEBUG
    
  }
  
}


void ControlStation::buildFormationContextMenu(QMenu* parentMenu)
{
  Node::buildFormationContextMenu(parentMenu);
    
  Q_D(ControlStation); Q_ASSERT(d);   
  
  if( parentMenu->actions().size() > 0 )
  { parentMenu->addSeparator(); }

  // 
  // persistent menu actions
  //
  
  
  //
  // temporary menu actions
  //
 
  QAction* action = nullptr;
  
  action = new QAction(parentMenu);
  action->setText(ControlStation::tr("&Realize Formation"));
  
  action->setEnabled( d->isFormationGraphValid );
  
  parentMenu->addAction(action);
  
  ControlStation::connect(action, &QAction::triggered,
    [=](bool checked)
    { 
      qDebug() << qualifiedName() << ": Realizing formation."; 
      realizeFormation();
    } );
    
}



#endif // WITH_GRAPHICS

// void ControlStation::registerCommandMessageInstances ( const NodeId& id )
// {
//   Q_D(ControlStation); Q_ASSERT(d);
//   d->addCommandMessageInstances(id);
// }
// 
// void ControlStation::unregisterCommandMessageInstances ( const NodeId& id )
// {
//   Q_D(ControlStation); Q_ASSERT(d);
//   d->removeCommandMessageInstances(id);
// }

// void ControlStation::processJoystickInterfaceMessage ( const IDL::Messages::JoystickCommand& message )
// {
//   Q_D(ControlStation); Q_ASSERT(d);
//   
//   Q_ASSERT(d->joystickMessage);
//   // copy the "payload" portion from the command message into the instance that
//   // actually will get send, i.e. the current default one
//   d->joystickMessage->axes = message.axes;
//   d->joystickMessage->actions = message.actions;
//   
//   // send the (default) message.
//   d->joystickTopic->send();
//   
// }

void ControlStation::processReceivedMessage ( const NodeId& sender, const IDL::Messages::JoystickCommand& message )
{
  QString name = ( blipExists(message.addressee) ?  blip(message.addressee).name() : "(--)" );
  
  qDebug() << qualifiedName() << ": received a 'JoystickCommand' message for" << name;
}

void ControlStation::processReceivedMessage ( const NodeId& sender, const IDL::Messages::ModeChangeCommand& message )
{
  QString name = ( blipExists(message.addressee) ?  blip(message.addressee).name() : "(--)" );
  qDebug() << qualifiedName() << ": Received a" << message.command << "command for" << name;


  if( NodeId(message.addressee) == id())
  {
    using namespace IDL::Commands;
    switch( message.command )
    {
      case N_JOIN_NETWORK           : joinSwarm(); break;
      case N_LEAVE_NETWORK          : leaveSwarm(); break;
      case NULL_COMMAND: //NOTE: fall-through intended
      default:
        qDebug() << qualifiedName() << ": Ignoring" << message.command << "command for self"; 
        break;
    }
  }
  
}

void ControlStation::processReceivedMessage(const NodeId& sender, const IDL::Messages::ExternalPlanCommand& message)
{
  QString name = ( blipExists(message.addressee) ?  blip(message.addressee).name() : "(--)" );
  qDebug() << qualifiedName() << ": received an 'ExternalPlanCommmand' message for" << name ;
}


void ControlStation::processReceivedMessage(const NodeId& sender, const IDL::Messages::DualHopGuidance& message)
{
  QString name = ( blipExists(sender) ?  blip(sender).name() : "(--)" );
  qDebug() << qualifiedName() << ": received a 'Dual-Hop Guidance' message from" 
    << name ;
  
}








#include "controlstation.moc"
