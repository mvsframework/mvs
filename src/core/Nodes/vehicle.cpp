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

#include "Nodes/vehicle.h"
#include "Nodes/vehicle_p.h"
#include "IDL/idlsupport.h"

#include <boost/graph/graph_utility.hpp>
#include <boost/property_map/compose_property_map.hpp>
#include <string>

#include <QtCore/QTimer>
#include <QtCore/QList>

#include "Graph/graphsupport.h"
#include "StateMachine/transitions.h"

//############################################################################//
//                                                                            //
//  VehiclePrivate                                                            //
//                                                                            //
//############################################################################//


VehiclePrivate::VehiclePrivate ( Vehicle* q
                                ,Vehicle::Settings* const settings
                                /*,Node* parent*/)
  :NodePrivate(q,settings)
  ,q_ptr ( q )
  ,xmlSettings(settings)
{ //NOTE: Do _NOT_ use q_ptr or q in here, it _WILL_ break things!

  { // add to the list of reported states.
    using namespace IDL::Data;
    reportedStates 
      << STATE_ID_V_AUTONOMOUS
      << STATE_ID_V_PILOTED
      << STATE_ID_V_JOYSTICK
      << STATE_ID_V_LOITER
      << STATE_ID_V_EXECUTING
      << STATE_ID_V_DUAL_HOP_GUIDANCE
      << STATE_ID_V_PERIMETER_GUIDANCE
      << STATE_ID_V_DYNAMIC_GUIDANCE;
  }

}

VehiclePrivate::~VehiclePrivate()
{
}

void VehiclePrivate::initializeStates()
{
  //
  // create the states
  // =================
  // Some of the created states would normally not be necessary when drawing
  // a statechart, but are required because of how the Qt QStateMachine works.
  // Those states are marked as "A Qt necessity" and most often represent 
  // orthogonal regions.
  //
  {
    autonomous = new State(IDL::Data::STATE_ID_V_AUTONOMOUS, active_nodeRegion, this);
    piloted    = new Piloted(active_nodeRegion, this);
    
    executingExternalCommands = new Activity(IDL::Data::STATE_ID_V_EXECUTING,piloted, this);
    pilotedLoiter = new Loiter(piloted, this);
    joystickControl = new State(IDL::Data::STATE_ID_V_JOYSTICK, piloted, this);
    
    lostLinkProcedure = new LostLinkActivity(operational, this); ///NOTE: this is an Activity, i.e. it has a "done" substate
    
    executingExternalPlan = new ExecutingExternalPlan(executingExternalCommands, this);
    
    movingToRouteGraph = new MovingToRouteGraph(joining, this);
    movingToHome       = new MovingToHome(leaving, this);
    movingToControlStation = new MovingToControlStation(lostLinkProcedure, this);
    
    autonomousLoiter = new Loiter(operational, this);
    dualHopGuidance = new DualHopGuidance(operational,this);
    perimeterGuidance = new PerimeterGuidance(operational, this);
    dynamiceGuidance = new DynamiceGuidance(operational, this);
    
  }
  
  //
  // set the initial (sub-)states
  // ==============================
  // Any "Region" state is "just" a Qt construct to allow parallelization at 
  // the level directly above. As such regions don't need to be set themselves 
  // as initial states of anything as their use implies that they are children 
  // of a QState::ParallelStates compound state and Qt automatically enters all
  // parallel children of compound states upon entry.
  //
  // NOTE: Activities automatically set the "done" substate as their initial 
  // state.
  //
  {
    
    //
    // Reparent existing states
    //
    {
      joining->setParent(autonomous);
      leaving->setParent(autonomous);
      operational->setParent(autonomous);
    }
    
    autonomous->setInitialState(joining);
    
    executingExternalCommands->setInitialState(executingExternalPlan);
    
    lostLinkProcedure->setInitialState(movingToControlStation);
    
    joining->setInitialState(movingToRouteGraph);
    leaving->setInitialState(movingToHome);
    
    piloted->setInitialState(pilotedLoiter);
    
    operational->setInitialState(autonomousLoiter);
    
  }
  
  //
  // add the transitions
  // ===================
  // NOTE: When creating custom transitions (i.e. subclassing QAbstractTransition)
  //   do remember that a state to which a transition is added assumes possession
  //   of that transition pointer. As a result, a transition pointer cannot be
  //   reused!!
  //
  { 
    //
    // Activity Transitions
    // ====================
    // Every Activity has a final state "done" that indicates the end of the
    // activity. As such, the "done"-state has/needs a transition to whatever 
    // comes after the activity. This section sets up these transitions.
    //
    
    /*auto doneExecuting =*/
    executingExternalCommands->addTransition(executingExternalCommands, SIGNAL(finished()), pilotedLoiter);
    
    /* auto doneLostLinkeProcedure =*/
    lostLinkProcedure->addTransition(lostLinkProcedure,SIGNAL(finished()), operational);
    
    //
    // Signal and Command Transitions
    // ==============================
    //
    
    Q_Q(Vehicle); Q_ASSERT(q);
    
    auto reachedGraph = new QSignalTransition(q,SIGNAL(flightplanFinished()));
    reachedGraph->setTargetState(joining->done);
    movingToRouteGraph->addTransition(reachedGraph);
    reachedGraph = nullptr; // prevent reuse 
    
    auto reachedHome = new QSignalTransition(q, SIGNAL(flightplanFinished()));
    reachedHome->setTargetState(leaving->done);
    movingToHome->addTransition(reachedHome);
    reachedHome = nullptr; // prevent reuse 

    
// This is made obsolete by the transitions added and removed in the LostLinkActivity::onEntry()
// and LostLinkActivity::onExit() methods.
//     auto reachedControlStation = new QSignalTransition(q, SIGNAL(flightplanFinished()));
//     reachedControlStation->setTargetState(lostLinkProcedure->done);
//     movingToControlStation->addTransition(reachedControlStation);
//     reachedControlStation = nullptr; // prevent reuse
    
    using namespace Transitions;
    
    auto goJoystick_frmAuto = new CommandTransition(IDL::Commands::V_ENABLE_JOYSTICK_CONTROL);
    goJoystick_frmAuto->setTargetState(joystickControl);
    autonomous->addTransition( goJoystick_frmAuto );
    goJoystick_frmAuto = nullptr; // prevent reuse 

    auto goJoystick_frmStby = new CommandTransition(IDL::Commands::V_ENABLE_JOYSTICK_CONTROL);
    goJoystick_frmStby->setTargetState(joystickControl);
    standby->addTransition( goJoystick_frmStby );
    goJoystick_frmStby = nullptr; // prevent reuse 
    
    auto goJoystick_frmPltd = new CommandTransition(IDL::Commands::V_ENABLE_JOYSTICK_CONTROL);
    goJoystick_frmPltd->setTargetState(joystickControl);
    piloted->addTransition( goJoystick_frmPltd );
    goJoystick_frmPltd = nullptr; // prevent reuse 
    
    
    
    auto executePlan_frmAuto = new CommandTransition(IDL::Commands::V_EXECUTE_EXTERNAL_PLAN);
    executePlan_frmAuto->setTargetState(executingExternalPlan);
    autonomous->addTransition(executePlan_frmAuto);
    executePlan_frmAuto = nullptr; // prevent reuse
    
    auto executePlan_frmStby = new CommandTransition(IDL::Commands::V_EXECUTE_EXTERNAL_PLAN);
    executePlan_frmStby->setTargetState(executingExternalPlan);
    standby->addTransition(executePlan_frmStby);
    executePlan_frmStby = nullptr; // prevent reuse
    
    auto executePlan_frmPltd = new CommandTransition(IDL::Commands::V_EXECUTE_EXTERNAL_PLAN);
    executePlan_frmPltd->setTargetState(executingExternalPlan);
    piloted->addTransition(executePlan_frmPltd);
    executePlan_frmPltd = nullptr; // prevent reuse
    
    
    auto goLoiter_frmAuto = new CommandTransition(IDL::Commands::V_LOITER);
    goLoiter_frmAuto->setTargetState(pilotedLoiter);
    autonomous->addTransition(goLoiter_frmAuto);
    goLoiter_frmAuto = nullptr; // prevent reuse
    
    auto goLoiter_frmStby = new CommandTransition(IDL::Commands::V_LOITER);
    goLoiter_frmStby->setTargetState(pilotedLoiter);
    standby->addTransition(goLoiter_frmStby);
    goLoiter_frmStby = nullptr; // prevent reuse
    
    auto goLoiter_frmPlt = new CommandTransition(IDL::Commands::V_LOITER);
    goLoiter_frmPlt->setTargetState(pilotedLoiter);
    piloted->addTransition(goLoiter_frmPlt);
    goLoiter_frmPlt = nullptr; // prevent reuse
    
        
    auto finishedExternalPlan = new QSignalTransition(q, SIGNAL(flightplanFinished()));
    finishedExternalPlan->setTargetState(executingExternalCommands->done);
    executingExternalPlan->addTransition(finishedExternalPlan);
    finishedExternalPlan = nullptr; // prevent reuse 
    
    auto joinNetwork_frmPltd = new CommandTransition(IDL::Commands::N_JOIN_NETWORK);
    joinNetwork_frmPltd->setTargetState(joining);
    piloted->addTransition( joinNetwork_frmPltd );
    joinNetwork_frmPltd = nullptr; // prevent reuse  
    
    auto leaveNetwork_frmPltd = new CommandTransition(IDL::Commands::N_LEAVE_NETWORK);
    leaveNetwork_frmPltd->setTargetState(leaving);
    piloted->addTransition( leaveNetwork_frmPltd );
    leaveNetwork_frmPltd = nullptr; // prevent reuse
    
    
    Q_ASSERT(lostLinkTimer);
    auto goLostLink_fromActive = new QSignalTransition(lostLinkTimer, SIGNAL(timeout()));
    goLostLink_fromActive->setTargetState(lostLinkProcedure);
    active->addTransition( goLostLink_fromActive );
    goLostLink_fromActive = nullptr; // prevent reuse
    
    {//Debug Transitions
      auto goOperationalLoiter_frmOprtnl = new CommandTransition(IDL::Commands::D_V_ENTER_OPERTAIONAL_LOITER);
      goOperationalLoiter_frmOprtnl->setTargetState(autonomousLoiter);
      operational->addTransition(goOperationalLoiter_frmOprtnl);
      goOperationalLoiter_frmOprtnl = nullptr; // prevent reuse
      
      auto goDualHop_frmOprtnl = new CommandTransition(IDL::Commands::D_V_ENTER_DUAL_HOP);
      goDualHop_frmOprtnl->setTargetState(dualHopGuidance);
      operational->addTransition(goDualHop_frmOprtnl);
      goDualHop_frmOprtnl = nullptr; // prevent reuse
    }
    
  }

  
  //
  // Additional Signal-triggered Events
  // ==================================
  //
    
  Vehicle::connect( standby, &State::entered,
      ownshipBlip, &OwnshipBlip::clearFlightplan );
    
}

void VehiclePrivate::initializeDatalink()
{
  Q_Q(Vehicle); Q_ASSERT(q);
  {/// initialize JoystickCommand topic
    joystickTopic = new AddressedTopic<IDL::Messages::JoystickCommand,Vehicle,TopicOptions::Receive>
      ("JoystickCommand",dds,q);
    joystickTopic->initialize();
  } 
  {/// initialize ModeChangeCommand topic
    commandTopic = new AddressedTopic<IDL::Messages::ModeChangeCommand, Vehicle, TopicOptions::Receive>
      ("Command",dds,q);
    commandTopic->initialize();
  }
  {/// initialize PlanCommand topic
    planCommandTopic = new ReliableTopic<IDL::Messages::ExternalPlanCommand, Vehicle, TopicOptions::Receive>
      ("PlanCommand",dds,q);
    planCommandTopic->initialize();
      
  }
  {/// initialize DualHopGuidance topic
    dualHopGuidanceTopic = new BroadcastTopic<IDL::Messages::DualHopGuidance,Vehicle,TopicOptions::SendReceive>
      ("Dual-Hop Guidance",dds,q);
    dualHopGuidanceTopic->initialize();
    dualHopGuidanceMessage = dualHopGuidanceTopic->message();
  }

}

void VehiclePrivate::unInitializeVehicle()
{
  { // unInitialize the lost link timer
    lostLinkTimer->stop();
    delete lostLinkTimer;
    lostLinkTimer = nullptr;
  }
  
  {/// unInitialize the joystick topic
    delete joystickTopic;
    joystickTopic = nullptr;
  }
  {/// unInitialize ModeChangeCommand topic
    delete commandTopic;
    commandTopic = nullptr;
  }
  {/// unInitialize PlanCommand topic
    delete planCommandTopic;
    planCommandTopic = nullptr;
  }
  {/// unInitialize DualHopGuidance topic
    dualHopGuidanceMessage = nullptr;
    delete dualHopGuidanceTopic;
    dualHopGuidanceTopic = nullptr;
  }
}

void VehiclePrivate::updateRfConnectedArea()
{
  Q_Q(Vehicle); Q_ASSERT(q);
  if( commandingControlStation == nullptr )
  { 
//NOTE: This actually is "expected" behavior, so maybe this message can be 
// skipped (especially as it is printed rather frequently...)
//    qDebug() << q->qualifiedName() << ": Cannot update 'updateRfConnectedArea' without a commanding ControlStation.";
   return;
  }
  
  QList<NodeId> connectedBlips = comRelayNodes(commandingControlStation->id()); 
  
  
//   std::cout << q->qualifiedName().toStdString() << " : R/f connected Blips: ";
  
  for( std::pair<const NodeId,Blip*> pair : blipData )
  { 
    /** \internal The logic check on !mode().contains(IDL::Data::STATE_ID_N_STANDBY)
     * removes standy nodes from the R/F area. 
     * However, this results in either the need to call updateRfConnectedArea 
     * not only on changes in the COM graph, but also when other Blips receive 
     * the IDL::Commands::N_LEAVE_NETWORK command, or the acceptance of the fact
     * that the updateRfConnectedArea could be slightly 
     * outdated. */
    
    /** \todo A better interaction is needed wrt. the "reportedStates" */
    bool isInRfArea = connectedBlips.contains(pair.first);
//       and !pair.second->mode().contains(IDL::Data::STATE_ID_N_STANDBY)
//       and !pair.second->mode().contains(IDL::Data::STATE_ID_N_LEAVING)
      
      
      
//     if( isInRfArea )
//     { std::cout << pair.second->name().toStdString() << " "; }
                      
    pair.second->setRfRelayFlag(isInRfArea);
           
  }  
  
//   std::cout << std::endl;
}

void VehiclePrivate::initializeBlip(const NodeId& id, const Node::Type& type
  ,QString const & name, QStringList const & propertyList, Blip* blip)
{
  NodePrivate::initializeBlip(id,type,name,propertyList,blip); 
  
  Q_Q(Vehicle); Q_ASSERT(q);
  
  // 'blip' is a 'by-value' argument into NodePrivate::initializeBlip and hence
  // is most likely a nullptr;
  if( blip == nullptr )
  { blip = &(q->blip(id)); }
  
  if( blip == ownshipBlip )
  { return; }
  
  switch( blip->type() )
  {
    case IDL::Data::NODE_TYPE_GCS :
    {
      qDebug() << q->qualifiedName() << ": detected ControlStation:" << blip->name();
      detectedControlStations.insert(id);
      break;
    }
    default : /* do nothing */ break;
  }

  
  Vehicle::connect(blip,&Blip::modeChanged,
    [=](){ processModeChanges(id); }); /// \todo Disconnect upon reinitialization.
}

void VehiclePrivate::deleteBlip(NodeId const &id)
{
  Q_Q(Vehicle); Q_ASSERT(q);
  
  Blip* blip = &(q->blip(id));
  
  switch( blip->type() )
  {
    case IDL::Data::NODE_TYPE_GCS :
    {
      detectedControlStations.remove(id); 
      qDebug() << q->qualifiedName() << ": removed ControlStation:" << blip->name();
      
      if( commandingControlStation != nullptr and id == commandingControlStation->id() )
      { q->unsetCommandingControlStation(); }
      
      break;
    }
    default : /* do nothing */ break;
  }
  
  /** \internal It is not necessary to disconnect the connections made in 
   * initializeBlip as those will be disconnected upon destruction of the 
   * respective Blip -- which is imminent. */
  
  NodePrivate::deleteBlip(id);
}


void VehiclePrivate::goTo_direct(Waypoint const & destination)
{
 Flightplan fp;
 fp.append( destination );
 
 processFlightplan(fp);
}

void VehiclePrivate::goTo_closestOnRouteGraph(const Waypoint& destination, bool interpolate)
{
  using namespace boost;
  
  /// - find the route graph vertex closest to the given destination
  
  Geometry::Point_2D dest_2D(destination.position().x,
                             destination.position().y );

  Geometry::Point_2D entryPoint2D;
  if( interpolate == false )
  {
    auto locationIdMap = get(vertex_locationId, routeGraph);
    auto locationMap = make_compose_property_map(location_pMap, locationIdMap);
    
    auto entryVertex = Graph::getNearestVertex(routeGraph,locationMap,dest_2D);   
    entryPoint2D = locationMap[entryVertex];
  }
  else
  {
    entryPoint2D = closestPointOnRouteGraph(dest_2D);
  }
  
  /// build the flight plan
  double const altitude = ownshipBlip->routeAltitude();
  Flightplan fp;
  fp.append(Waypoint( entryPoint2D.x(),entryPoint2D.y(), altitude  ));
  
  processFlightplan(fp);
}

Flightplan VehiclePrivate::plan_viaRouteGraph( const Waypoint& destination, bool interpolate )
{
  using namespace boost;
  
  using RouteGraphType=RouteGraph::Graph;
  
  { // Concept checking for Dijkstra
    { // graph: VertexListGraph and IncidenceGraph
      BOOST_CONCEPT_ASSERT(( VertexListGraphConcept<RouteGraphType> ));
      BOOST_CONCEPT_ASSERT(( IncidenceGraphConcept<RouteGraphType> ));
    }
  }

  /// - find the route graph vertex closest to the current position
  
  
  Geometry::Point_2D entryPoint2D = closestPointOnRouteGraph(ownshipBlip);
  
  auto locationIdMap = get(vertex_locationId, routeGraph);
  auto locationMap = make_compose_property_map(location_pMap, locationIdMap);
  
  auto entryVertex = Graph::getNearestVertex(routeGraph,locationMap,entryPoint2D);   

  
  
  /// - find the route graph vertex closest to the destination
  
  Geometry::Point_2D destination2D = Geometry::Point_2D( destination.position().x,
                                                         destination.position().y);  
  
  Geometry::Point_2D exitPoint2D = closestPointOnRouteGraph(destination2D);
  
  auto exitVertex = Graph::getNearestVertex(routeGraph,locationMap,exitPoint2D);
  
  /// - find the shortest path on the route graph connecting those two vertices  

  // the predecessor map allows backtracking through the graph
  std::map<RouteGraph::Vertex_Descriptor,RouteGraph::Vertex_Descriptor> predecessorData;
  auto pMap = make_assoc_property_map(predecessorData);
  
  // see http://www.boost.org/doc/libs/1_53_0/libs/graph/doc/dijkstra_shortest_paths.html
  // and http://www.boost.org/doc/libs/1_53_0/libs/graph/example/dijkstra-example-listS.cpp
  // NOTE: boost::predecessor_map is a _name_parameter_function_, see 
  //   http://www.boost.org/doc/libs/1_53_0/libs/graph/doc/bgl_named_params.html
  dijkstra_shortest_paths(routeGraph,exitVertex, predecessor_map(pMap)  );
  

  
  
  
  /// build the flight plan
  double const altitude = ownshipBlip->routeAltitude();
  Flightplan fp;
  fp.append(Waypoint( entryPoint2D.x(),entryPoint2D.y(),altitude ));
  
  for(RouteGraph::Vertex_Descriptor wpVertex = entryVertex ;
      wpVertex != exitVertex /*and pMap[wpVertex] != exitVertex*/;
      wpVertex = pMap[wpVertex] /*advance one vertex*/ )
  {
    Geometry::Point_2D wpPoint = locationMap[wpVertex];
    fp.append(Waypoint(wpPoint.x(),wpPoint.y(),altitude));    
  }
  Geometry::Point_2D exitVertex2D = locationMap[exitVertex];
  fp.append(Waypoint( exitVertex2D.x(),exitVertex2D.y(),altitude ));
  
  fp.append(Waypoint( exitPoint2D.x(),exitPoint2D.y(),altitude ));
  
  fp.append( destination );
  
  Flightplan::eliminateDoubleBacks(fp);
    
  return fp;
}

void VehiclePrivate::processFlightplan(Flightplan const & flightplan)
{
  ownshipBlip->setFlightplan(flightplan.toIdl());
  
  Q_Q(Vehicle); Q_ASSERT(q);
  q->executeFlightplan(flightplan);
}

Waypoint VehiclePrivate::randomWaypointInArena( const double& altitude )
{
  
  
  auto boundingBox = Geometry::boundingBox(environment2D);
  
  qreal x_min = boundingBox.min_corner().x();
  qreal y_min = boundingBox.min_corner().y();
  
  qreal width = boundingBox.max_corner().x() - x_min;
  qreal height = boundingBox.max_corner().y() - y_min;
  
  double x = x_min + rZeroOne() * width;
  double y = y_min + rZeroOne() * height;
  
  return Waypoint(x,y,-altitude);
}


void VehiclePrivate::processModeChanges(NodeId const & id)
{
  Q_Q(Vehicle); Q_ASSERT(q);
  
  using namespace IDL::Data;
  
  switch( q->blip(id).type() )
  {
    case NODE_TYPE_GCS :     processControlStationModeChanges(id); break; 
    case NODE_TYPE_VEHICLE : processVehicleModeChanges(id); break; 
    default: /* do nothing */ break;
  }
}

void VehiclePrivate::processControlStationModeChanges(NodeId const & id)
{  
  Q_Q(Vehicle);Q_ASSERT(q);
  qDebug() << q->qualifiedName() << ": Processing ControlStation mode changes.";

//   Blip* csBlip = &q->blip(id);
//   Q_ASSERT(csBlip);
//   Q_ASSERT(csBlip->type() == IDL::Data::NODE_TYPE_GCS );                 
//                                                           
//   if( commandingControlStation==nullptr )
//   { 
//     if( csBlip->mode().contains( IDL::Data::STATE_ID_N_OPERATIONAL ) )
//     {
//       commandingControlStation = csBlip;        
//       qDebug() << q->qualifiedName() << ": Set the commanding ControlStation to" 
//         << csBlip->name();
//       emit q->commandingControlStationChanged(id);
//     }
//   }
//   else
//   { // commandingControlStation != nullptr
//     if( commandingControlStation->id() == id 
//         and !csBlip->mode().contains( IDL::Data::STATE_ID_N_OPERATIONAL ))
//     {
//       commandingControlStation = nullptr;
//       qDebug() << q->qualifiedName() << ": Unset the commanding ControlStation.";
// //       emit q->commandingControlStationChanged( NULL_NODE_ID); //TODO: create a NULL_NODE_ID
//     } 
//   }
}

void VehiclePrivate::processVehicleModeChanges(NodeId const & id)
{
  Q_Q(Vehicle);Q_ASSERT(q);
  
  Blip* vehicleBlip = &q->blip(id);
  Q_ASSERT(vehicleBlip);
  Q_ASSERT(vehicleBlip->type() == IDL::Data::NODE_TYPE_VEHICLE );
  
  qDebug() << q->qualifiedName() << ": Processing Vehicle mode changes of" << vehicleBlip->name();
  
  if( primaryVehicleBlip==nullptr )
  { // this vehicle does not recognize a blip as being the primary vehicle...
    if( vehicleBlip->mode().contains(IDL::Data::STATE_ID_V_JOYSTICK) )
    { // the blip that just changed modes included joystick control...
      
      if(ownshipBlip->mode().contains(IDL::Data::STATE_ID_V_JOYSTICK) )
      { 
        // this vehicle no longer is under joystick control...
        q->stopAndLoiter();
      }
      
      // ...=> make it the primary vehicle.
      primaryVehicleBlip = vehicleBlip;

      qDebug() << q->qualifiedName() << ": Set the primary Vehicle to"
        << vehicleBlip->name();
      
      emit q->primaryVehicleChanged(id);
      
//       QObject::connect(primaryVehicleBlip, &Blip::rfPolygonChanged,
//         [&](){this->updateDualHopGuidance();}); //NOTE: capture by copy [=] is different than capture by reference [&]!
    }
  }
  else
  { // primaryVehicleBlip != nullptr
    // this vehicle has one blip marked as being the primary vehicle...
    
    
    if( primaryVehicleBlip->id() == id 
        and !vehicleBlip->mode().contains(IDL::Data::STATE_ID_V_JOYSTICK))
    { /// ... and that is not the primary vehicle any more
      primaryVehicleBlip=nullptr;
      qDebug() << q->qualifiedName() << ": Unset the primary Vehicle.";
    }
  }
}



void VehiclePrivate::updateMultiHopGuidance()
{
 
  Blip* head = primaryVehicleBlip;
  Blip* tail = commandingControlStation;
  
  Q_Q(Vehicle);Q_ASSERT(q);
  
  // sanity checking the fast and slow Blip.
  if( head == nullptr )
  { 
    qCritical() << q->qualifiedName() <<": CRITICAL:"
      << "Cannot update Multi-Hop Guidance, HEAD == NULLPTR.";
    return;
  }
  if( tail == nullptr )
  { 
    qCritical() << q->qualifiedName() <<": CRITICAL:"
      << "Cannot update Multi-Hop Guidance, Tail == NULLPTR.";
    return;
  }
  
  qDebug() << q->qualifiedName() << ": Updateing Dual-Hop Guidance to connect"
    << head->name() << "at" << head->position2D() << "with" 
    << tail->name() << "at" << tail->position2D();
     
  auto rfIntersection = [=](Blip* a, Blip* b)->std::deque<Geometry::Polygon_2D>
  {
    std::deque<Geometry::Polygon_2D> rfint;
    
    boost::geometry::intersection(a->rfPolygon(),b->rfPolygon(), rfint); 
        
    if( rfint.size() == 0 )
    { 
      qWarning() << q->qualifiedName() 
        << ": R/F polygons of" << a->name() << "and" << b->name() << "don't intersect.";
    }
    
    return rfint;
  };
  

  Q_ASSERT(dualHopGuidanceData);  
  auto data = dualHopGuidanceData;  

  //  
  // intersect the R/F polygons from the two Blips and generate the target area
  // FIXME: Currently this step ignores the possibility of multiple 
  // intersections.
  // HACK: If such a scenario occurs, the _first_area_found_ is used. Although
  // easy, this is most certainly a bad choice.   
  //
  #ifdef WITH_GRAPHICS
    data->targetAreaItem->setGeometry(rfIntersection(head,tail).front());
//     scene->update();
  #else // WITH_GRAPHICS
    data->targetArea = rfIntersection(head,tail).front(); 
  #endif // WITH_GRAPHICS
 
  //
  // Select a target waypoint which is within the target area and on the route
  // graph.
  //
  
  bool moveOnRouteGraph = true;
  
  Geometry::Point_2D centroid;
  boost::geometry::centroid(data->targetArea,centroid);
  
#ifdef WITH_GRAPHICS
    data->centroidItem->setGeometry(centroid);
#else
    data->centroid = centroid;
#endif // WITH_GRAPHICS  
  
  
  Waypoint target;
  
  if( moveOnRouteGraph )
  {
    Geometry::Point_2D onRoute = closestPointOnRouteGraph(centroid);
    target.setPosition(onRoute.x(),onRoute.y(), ownshipBlip->routeAltitude());

#ifdef WITH_GRAPHICS
    data->targetItem->setGeometry(onRoute);
#else
    data->target = onRoute;
#endif // WITH_GRAPHICS
    
    goTo_viaRouteGraph(target);
  }
  else
  { // simply set the centroid of the target area as the target
    target.setPosition(centroid.x(),centroid.y(),ownshipBlip->routeAltitude());
    
#ifdef WITH_GRAPHICS
    data->targetItem->setGeometry(centroid);
#else
    data->target = centroid;
#endif // WITH_GRAPHICS
    
    goTo_direct(target);
  }
    
}

//############################################################################//
//                                                                            //
//  VehiclePrivate::State                                                     //
//                                                                            //
//############################################################################//


void VehiclePrivate::Piloted::onEntry( QEvent* event )
{
  State::onEntry(event);
  vehicle_private_ptr->q_func()->stopMotion();
  vehicle_private_ptr->ownshipBlip->clearFlightplan();
}


void VehiclePrivate::LostLinkActivity::onEntry( QEvent* event )
{ 
  Q_UNUSED(event);
  
  Q_ASSERT(reestablishedLink == nullptr);
  reestablishedLink = new QSignalTransition(vehicle_private_ptr->commandingControlStation, SIGNAL(connectednessToHostChanged(bool)));
  reestablishedLink->setTargetState(this->done);
  this->addTransition(reestablishedLink);
}

void VehiclePrivate::LostLinkActivity::onExit( QEvent* event )
{
  Q_UNUSED(event);
  
  this->removeTransition(reestablishedLink);
  
  reestablishedLink->deleteLater();
  reestablishedLink = nullptr;
}

void VehiclePrivate::MovingToRouteGraph::onEntry( QEvent* event )
{
  State::onEntry(event);
  
  Geometry::Point_2D pos = vehicle_private_ptr->closestPointOnRouteGraph(vehicle_private_ptr->ownshipBlip);
  
  Waypoint wp;
  wp.setPosition(pos.x(),pos.y(),vehicle_private_ptr->ownshipBlip->routeAltitude());
  
  vehicle_private_ptr->goTo_direct(wp);
}

void VehiclePrivate::MovingToHome::onEntry( QEvent* event )
{
  State::onEntry(event);
  auto home = vehicle_private_ptr->getXmlInitialPosition().coordsIn(AMG::CoSy::getDatumFrame());
  constexpr double offset = -10; //FIXME HACK In order to avoid crashing, pad "home" with 10 m of altitude!
  vehicle_private_ptr->goTo_viaRouteGraph(Waypoint(home[0],home[1],home[2]+offset));
}

void VehiclePrivate::MovingToControlStation::onEntry( QEvent* event )
{
  State::onEntry(event);

  auto csPos = vehicle_private_ptr->commandingControlStation->position2D();
  double const altitude = vehicle_private_ptr->ownshipBlip->routeAltitude();
  vehicle_private_ptr->goTo_viaRouteGraph(Waypoint(csPos.get<0>(),csPos.get<1>(),altitude));

}


void VehiclePrivate::ExecutingExternalPlan::onEntry( QEvent* event )
{
  State::onEntry(event);
  vehicle_private_ptr->q_func()->stopMotion();
  vehicle_private_ptr->ownshipBlip->clearFlightplan();
  vehicle_private_ptr->processFlightplan(vehicle_private_ptr->externalFlightplan);
}

void VehiclePrivate::Loiter::onEntry( QEvent* event )
{
  State::onEntry(event);
  vehicle_private_ptr->q_func()->stopMotion();
  vehicle_private_ptr->ownshipBlip->clearFlightplan();
}


void VehiclePrivate::DualHopGuidance::onEntry( QEvent* event )
{
  State::onEntry(event);
  
  /** \todo I actually would like to only start publishing on the DualHopGuidance 
   * topic once inside the appropriate mode... but I ran into odd threading issues...
   */
  
  VehiclePrivate * const d = vehicle_private_ptr;
  Vehicle* const q = d->q_func(); Q_ASSERT(q);
  
  /// 1. Determine the primary vehicle to be correctly detected
  if( d->primaryVehicleBlip == nullptr )
  { 
     qCritical("\"%s\" : CRITICAL : Cannot set up Dual-Hop Guidance without a primary vehicle.", 
                q->qualifiedName().toStdString().c_str() );
     return;
  }
  d->dualHopGuidanceMessage->primaryVehicle = d->primaryVehicleBlip->id();
    
  /// 2. Determine the presence of a (matching?) GCS
  if( d->commandingControlStation == nullptr )
  {
    qCritical("\"%s\" : CRITICAL : Cannot set up Dual-Hop Guidance without a control station.", 
                q->qualifiedName().toStdString().c_str() );
    return;
  }
  d->dualHopGuidanceMessage->controlStation = d->commandingControlStation->id();
  
  
  // as this is "setting up" the dual hop guidance, let's indicate that through
  // a non-sensical part in the assoc. message: totalBeadCount = 0. This doesn't
  // make sense as in that case--no intermediary necessary between primary and 
  // control station--the dual hop scenario wouldn't make sense.
  d->dualHopGuidanceMessage->totalBeadCount = 0;
  
  
  // initialize the bead index with a number that 'should' be the highest
  // (but might very well not be!!!)
  unsigned int count = vehicle_private_ptr->blipCount(IDL::Data::STATE_ID_V_DUAL_HOP_GUIDANCE);
  d->dualHopGuidanceMessage->beadIndex = count; //TODO: improve this inital guess by checking the index of the blips in this state
  
  
  {
    
    d->dualHopGuidanceData = new DualHopGuidanceData;
#ifdef WITH_GRAPHICS
    d->dualHopGuidanceData->targetAreaItem->setParentItem(d->datumItem);
    d->dualHopGuidanceData->targetItem->setParentItem(d->datumItem);
    d->dualHopGuidanceData->centroidItem->setParentItem(d->datumItem);
#endif // WITH_GRAPHICS
    
    Blip*& head = d->primaryVehicleBlip;
    Blip*& tail = d->commandingControlStation; 
    
//     d->dualHopGuidanceData->tracking_head = 
      Vehicle::connect(head, &Blip::rfPolygonChanged ,
                       [=](){d->updateMultiHopGuidance();} );

//       d->dualHopGuidanceData->tracking_tail = 
      Vehicle::connect(tail, &Blip::rfPolygonChanged ,
                       [=](){d->updateMultiHopGuidance();} );
  }
  
}

void VehiclePrivate::DualHopGuidance::onExit( QEvent* event )
{
  VehiclePrivate * const d = vehicle_private_ptr;
  
//   Vehicle::disconnect(d->dualHopGuidanceData->tracking_head);
//   Vehicle::disconnect(d->dualHopGuidanceData->tracking_tail);
  
  delete d->dualHopGuidanceData;
  d->dualHopGuidanceData = nullptr;
  
  State::onExit( event );
}


//############################################################################//
//                                                                            //
//  Vehicle                                                                   //
//                                                                            //
//############################################################################//

/** \note This constructor does NOT do any work!
 * All Vehicle level construction needs to happen in 
 * Vehicle(VehiclePrivate & dd, ...)!
 */
Vehicle::Vehicle( Settings* const settings
                 /*,Node* parent*/)
  :Vehicle(*new VehiclePrivate(this,settings/*,parent*/)/*,parent*/)
{}

Vehicle::Vehicle( VehiclePrivate& dd
                 /*,Node* parent*/ )
  : Node(dd/*, parent*/) // Vehicles are Nodes and hence don't have parents
{
  Q_D(Vehicle); Q_ASSERT(d);
  /** \internal 
   * This will only indicate the status of the Vehicle, any derived classes
   * will not be constructed at this point!
   */
  d->setStatus(Status::Constructed); 
  
  

  
           
    
}


Vehicle::~Vehicle()
{
  Q_D(Vehicle); Q_ASSERT(d);
  d->unInitializeVehicle();
}


Node::Type Vehicle::type() const
{ return IDL::Data::NODE_TYPE_VEHICLE; }


void Vehicle::initializeItem()
{
  Node::initializeItem();
  // do local work below ...

  Q_D(Vehicle); Q_ASSERT(d);

  { // initialize lost link timer
    //NOTE: this has to happen before the initialization of the States as there
    // is a QSignalTransition depending on this timer.
    d->lostLinkTimer = new QTimer(this);
    d->lostLinkTimer->setSingleShot(true);
  }

  d->initializeDatalink();
  d->initializeStates();  
  
}

// void Vehicle::startItem()
// {
//   Node::startItem();
//   // do local work below ...
// }

// void Vehicle::stopItem()
// {
//   // do local work above...
//   Node::stopItem();
// }

void Vehicle::unInitializeItem()
{
  Q_D(Vehicle); Q_ASSERT(d);
  d->unInitializeVehicle();
  
  // do local work above...
  Node::unInitializeItem();
}

void Vehicle::goTo ( const Waypoint& destination, Vehicle::Routing routing )
{
  Q_D(Vehicle); Q_ASSERT(d);
  switch( routing )
  {
    case Routing::directTo : d->goTo_direct(destination); break;
    case Routing::viaRouteGraph: //NOTE: Fall-through intended
    default:d->goTo_viaRouteGraph(destination); break;
  }
//   d->goTo_closestOnRouteGraph(destination);
}


void Vehicle::setTargetId(const ElementId& id)
{
  if(id == ownship().targetId())
  { return; }
  
  Node::setTargetId(id);
  
  Q_D( Vehicle ); Q_ASSERT(d);
  if( d->commandingControlStation != nullptr )
  { 
    d->createComCorridor(id, d->commandingControlStation->id());
    //NOTE: As vehicles currently don't make any use of this information, I decided
    // to skip the computation to maintain a better responsiveness of the 
    // (ControlStation) GUI for cases where several nodes are executed on the some
    // host as the GUI.    
    //FIXME: Well, I guess I proposed that all nodes (vehicles) involved would
    // compute solutions for a formation and then agree on one... :/
//     d->proposeComFormation(id, d->commandingControlStation->id());
  }
  else
  { 
//NOTE: As vehicles currently don't make any use of this information, I decided
// to skip the computation to maintain a better responsiveness of the 
// (ControlStation) GUI for cases where several nodes are executed on the some
// host as the GUI.    
//     qDebug() << qualifiedName() << ": Cannot propose a COM formation without a commanding ControlStation.";
//     d->proposeComFormation(id, ownship().id()); 
  }
}


void Vehicle::goTo_randomPointInArena()
{
  Q_D(Vehicle); Q_ASSERT(d);
  d->goTo_viaRouteGraph(d->randomWaypointInArena());
}

void Vehicle::goJoystickControl()
{
  Q_D(Vehicle); Q_ASSERT(d);
  d->postCommand(IDL::Commands::V_ENABLE_JOYSTICK_CONTROL);
}

void Vehicle::executeExternalPlan()
{
  Q_D(Vehicle); Q_ASSERT(d);
  d->postCommand(IDL::Commands::V_EXECUTE_EXTERNAL_PLAN);
}


void Vehicle::stopAndLoiter()
{
  Q_D( Vehicle ); Q_ASSERT(d);
  d->postCommand(IDL::Commands::V_LOITER);
  
}


const Blip* Vehicle::primaryVehicle() const
{
  Q_D(const Vehicle); Q_ASSERT(d);
  return d->primaryVehicleBlip;
}

const Blip* Vehicle::commandingControlStation() const
{
  Q_D(const Vehicle); Q_ASSERT(d);
  return d->commandingControlStation;
}


bool Vehicle::setCommandingControlStation(const NodeId& id)
{
  Q_D(Vehicle); Q_ASSERT(d);
  if( !blipExists(id) )
  { 
    qDebug() << qualifiedName() 
    << ": Cannot set the commanding ControlStation to the unknown Blip" << id;
    return false;
  }
  else if( blip(id).type() != IDL::Data::NODE_TYPE_GCS )
  {
    qDebug() << qualifiedName() 
    << ": Cannot set the commanding ControlStation to the non-GCS Blip" << id;    
    return false;
  }
  else
  {
    if( d->commandingControlStation != nullptr )
    { unsetCommandingControlStation(); }
    
    d->commandingControlStation = &blip(id);
    
    d->qtCon_trackLinkToCommandingControlStation =
    connect(d->commandingControlStation, &Blip::connectednessToHostChanged
           ,[=](bool commandingControlStationIsConnected)
            {
              if( commandingControlStationIsConnected )
              { 
                qDebug() << qualifiedName() << ": Resetting Lost Link Timer.";
                d->lostLinkTimer->stop();
              }
              else
              {
                qDebug() << qualifiedName() << ": Starting Lost Link Timer (Time Out ="
                  << d->lostLinkTimeOut<< "s).";
                Q_ASSERT(d->lostLinkTimeOut > 0);
                d->lostLinkTimer->start(d->lostLinkTimeOut*1000); 
              }
            });
    
    return true;
  }
  //else
  qDebug() << qualifiedName() << ": Cannot set the commanding ControlStation to Blip" << id;
  return false;
}

bool Vehicle::unsetCommandingControlStation()
{
  Q_D(Vehicle); Q_ASSERT(d);
  if( d->commandingControlStation != nullptr )
  {
    disconnect(d->qtCon_trackLinkToCommandingControlStation);
    
    d->commandingControlStation = nullptr;
    
    qDebug() << qualifiedName() << ": The commanding ControlStation has been unset. Leaving...";
    d->postCommand(IDL::Commands::N_LEAVE_NETWORK);
    
    return true;
  }
  // else
  return false;
}

bool Vehicle::isConnectedToCommandingControlStation() const
{ 
  if( commandingControlStation() == nullptr )
  { return false; }
  else
  { return commandingControlStation()->isConnectedToHost(); }
}


void Vehicle::processReceivedMessage(const NodeId& sender, const IDL::Messages::Status& message)
{
  Node::processReceivedMessage(sender,message);

  Q_D(Vehicle); Q_ASSERT(d);
  
  { // deal with the setting/unsetting of the target through the commanding ControlStation
    if( d->commandingControlStation and d->commandingControlStation->id() == sender ) // only process updates from the current control station
    {
      if( message.target == NULL_ELEMENT_ID )
      { unsetTarget(); }
      else
      { 
        // copy the target from the ControlStation currently controling this Vehicle.
        setTargetId(message.target); 
      }
    }
  }

}

void Vehicle::processReceivedMessage ( const NodeId& sender, const IDL::Messages::ModeChangeCommand& message )
{
  // only actually react to messages that are intended for this node.
  if( NodeId(message.addressee) != id() )
  { return; }
  
/** \internal Normally I would simply call the Node level method to process these
 * commands, but vehicles shouldn't take all commands from all Nodes. Hence I 
 * opted to reimplement the Node level stuff here, but put it in the default in 
 * case there is an uncovered command left.
 */  
//   // deal with Node-level N_ commands...
//   Node::processReceivedMessage(sender,message);
  
  using namespace IDL::Commands;
  Q_D(Vehicle); Q_ASSERT(d);
  if( d->commandingControlStation )
  { // This vehicle already is assigned a commanding control station ...
    
    if( d->commandingControlStation->id() == sender ) // only process commands from the current control station
    { // .. which is sending this message to this vehicle.
      switch( message.command )
      {
        case NULL_COMMAND    : /* has no effect */  break;
        case N_JOIN_NETWORK  : joinSwarm();         break;
        case N_LEAVE_NETWORK :
        {
          unsetCommandingControlStation();
          leaveSwarm();        
          break;
        }
        
        case V_LOITER:                  stopAndLoiter(); break;
        case V_ENABLE_JOYSTICK_CONTROL: goJoystickControl(); break;
        case V_EXECUTE_EXTERNAL_PLAN :  executeExternalPlan(); break;
        
        case D_V_ENTER_DUAL_HOP :  //NOTE: fall-through intended
        case D_V_ENTER_OPERTAIONAL_LOITER:
        { /* post the command */
          d->postCommand(message.command);
          break;
        };
        
        default:
        { /* call the Node level version for any other cases. */
          Node::processReceivedMessage(sender,message);
          break;
        }
      }
    }  
    else
    { // this is a message either not from the correct control station or not intended 
      // for this vehicle
      /* do nothing so far */
    }
  }
  else 
  { // This Vehicle is not associated with any control station and as such
    // can/should/will only process Node level "join" commands.
    
    Blip* csBlip = &blip(sender);
    Q_ASSERT(csBlip);
    Q_ASSERT(csBlip->type() == IDL::Data::NODE_TYPE_GCS );   

    switch( message.command )
    {
      case N_JOIN_NETWORK :
      { 
        if( csBlip->mode().contains( IDL::Data::STATE_ID_N_OPERATIONAL ) 
            and setCommandingControlStation(sender) )
        {     
          qDebug() << qualifiedName() << ": Set the commanding ControlStation to" 
            << csBlip->name();
          joinSwarm();
        }
        break;
      }
      default:
      { /* call the Node level version for any other cases. */
        Node::processReceivedMessage(sender,message);
        break;
      }
    }
  }
  
}


void Vehicle::processReceivedMessage ( const NodeId& sender, const IDL::Messages::ExternalPlanCommand& message )
{
  Q_D(Vehicle); Q_ASSERT(d);
  if( d->commandingControlStation and d->commandingControlStation->id() == sender // only process commands from the current control station
      and NodeId(message.addressee) == id())
  {    
    
    qDebug() << qualifiedName() << ": Received an ExternalPlanCommand."
      << "Plan Hash:" << IDL::toQByteArray(message.plan.hash).toHex();
    
    
    Waypoint entry = message.plan.waypoints[0];
    
    // 
    // Determine the routing choice to the entry point of the received plan
    //
    
    Routing routing;
        
    Geometry::Point_2D entry2D( entry.position().x,entry.position().y );
    Geometry::Point_2D ownshipPos2d = ownship().position2D();
    
    if( boost::geometry::distance(ownshipPos2d,entry2D) < ROUTING_DIRECT_TO_THRSHLD )//TODO: AND (directTo path is collision free)   
    { routing = Routing::directTo; }
    else
    { routing = Routing::viaRouteGraph; }
    
    //
    // prepare the plan for execution
    //
    
    switch( routing )
    {
      case Routing::directTo:
      {
        d->externalFlightplan = message.plan;
        break;
      }  
      case Routing::viaRouteGraph:
      {
        Q_D(Vehicle); Q_ASSERT(d);
        
        Flightplan external = message.plan;       
        Flightplan plan = d->plan_viaRouteGraph(entry);
        
        if( external.waypoints.length() > 1 )
        { plan.append(external); }
        
        d->externalFlightplan = plan;
        
        break; 
      }
          
    };
    
    qDebug() << qualifiedName() << ": Received an external plan command from"
    << blip(sender).name() << "to go to" << d->externalFlightplan.waypoints.back().position();
       
    executeExternalPlan();
  }
}

void Vehicle::processReceivedMessage(const NodeId& sender, const IDL::Messages::DualHopGuidance& message)
{
//   if( QThread::currentThread() != this->thread() )
//   {
//     QMetaObject::invokeMethod(this,"processReceivedMessage",Qt::BlockingQueuedConnection,
//           Q_ARG(NodeId,   sender),
//           Q_ARG(IDL::Messages::DualHopGuidance, message )               
//         );
//     return;
//   }
//   
//   Q_D(Node); Q_ASSERT(d);
//   QMutexLocker locker(&(d->mutex));
  
  qDebug() << qualifiedName() << ": received a 'DualHopGuidance' message from" << blip(sender).name();
}



#include "vehicle.moc"
