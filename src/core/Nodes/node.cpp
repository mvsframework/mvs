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

#include "Nodes/node.h"
#include "Nodes/node_p.h"

#include <QtCore/QtNumeric>
#include <QtCore/QObject>
#include <QtCore/QDebug>
#include <QtCore/QDateTime>
#include <QtCore/QThread>
#include <QtConcurrent/QtConcurrent>
#include <QtCore/QMutexLocker>
#include <QtCore/QSet>

#include <boost/tuple/tuple.hpp> // also provides "tie"
#include <boost/graph/graph_utility.hpp> // get print_graph
#include <boost/graph/copy.hpp>
#include <boost/property_map/property_map.hpp>
#include <boost/property_map/compose_property_map.hpp>
#include <boost/geometry/algorithms/overlaps.hpp> 

#include <random>

#include <Eigen/Dense>

#include <amg.hpp>
#include "config.h"

#include "DDS/datalink.h"
#include "IDL/idlsupport.h"
#include "visilibityinterface.h"
#include "Graph/graphsupport.h"
#include "Graph/obstaclegraph.h"
#include "Graph/directedrfgraph.h"
#include "Graph/facegeometry_visitor.h"
#include "Graph/misc/my_subgraph.hpp"
#include "Geometry/slot_2d.h"
#include "XSD/xmltools.h"
#include "StateMachine/transitions.h"
#include "StateMachine/events.h"

#ifdef WITH_GRAPHICS
  #include <QtWidgets/QApplication>
  #include <QtWidgets/QMenu>
  #include "Graphics/palette.h"
  #include "Graphics/geometryitem.h"
  #include "Graphics/graphgraphics.h"
  #include <math.h> // for round()
#include <sys/stat.h>
  
#else
  #include <QtCore/QCoreApplication>
#endif // WITH_GRAPHICS


//############################################################################//
//                                                                            //
//  NodePrivate                                                               //
//                                                                            //
//############################################################################//

NodePrivate::NodePrivate( Node* q,
                          Node::Settings*const settings
                          /* SimItem* parent*/)
  :SimItemPrivate( q, settings )
//   ,QObject()
  ,q_ptr(q)
  ,xmlSettings(settings)
{ //NOTE: Do _NOT_ use q_ptr or q in here, it _WILL_ break things!
  { // initialize the list of reported states.
    using namespace IDL::Data;
    reportedStates 
      << STATE_ID_N_STANDBY
      << STATE_ID_N_JOINING
      << STATE_ID_N_OPERATIONAL
      << STATE_ID_N_LEAVING;
  }
}

NodePrivate::~NodePrivate()
{
//   Q_Q(Node); Q_ASSERT(q);
//   qDebug() << q->qualifiedName() << ": ~NodePrivate()";

}

// Private Static Member
QList<IDL::Data::StateDescriptorId> NodePrivate::reportedStates;

void NodePrivate::initializeDatalink()
{
  Q_Q(Node); Q_ASSERT(q);
  
  dds->initialize();
  
  ownshipId = NodeId(dds->participant_guid());
  qDebug() << q->qualifiedName() << ": Id :" << ownshipId;
  
  /** \todo Maybe the topic initialization should be implemented as some sort of
   *  functional that is then given to the datalink so that the datalink could 
   *  "initialize" the topic on Datalink::run() (and consqently delete it at 
   *  Datalink::halt()) ?
   */
  { /// initialize broadcast topics
  
    /// initialize State topic
    stateTopic = new BroadcastTopic<IDL::Messages::State,Node,TopicOptions::SendReceive>("State",dds,q);
    stateTopic->initialize();
    stateMessage = stateTopic->message();
   
    /// initialize Status topic
    statusTopic = new BroadcastTopic<IDL::Messages::Status,Node,TopicOptions::SendReceive>("Status",dds,q);
    statusTopic->initialize();
    statusMessage = statusTopic->message();
//     statusMessage->closestObstacle = 0;
    
  
    /// initialize Datum topic
    datumTopic = new BroadcastTopic<IDL::Messages::Datum,Node,TopicOptions::SendReceive>("Datum",dds,q);
    datumTopic->initialize();
    datumMessage = datumTopic->message();
  
    /// initialize Flightplan topic
    flightplanTopic = new BroadcastTopic<IDL::Messages::Flightplan,Node,TopicOptions::SendReceive>("Flightplan",dds,q);
    flightplanTopic->initialize();
    flightplanMessage = flightplanTopic->message(); 
  }
}

#ifdef WITH_GRAPHICS
void NodePrivate::initializeGraphics()
{
  /** \internal 
   * scene /// external 
   * datumItem 
   * environmentItem
   * targetEnvironmentGraphics /// at this point the targetEnvironment polygon is not valid, hence the graphics wouldn't be valid either.
   * comCorridorGraphics /// at this point the targetEnvironment polygon is not valid, hence the graphics wouldn't be valid either.
   * obstacleCellGraphics
   * (obstacleDelaunayFaceGraphics)
   * 
   * comGraphGraphics
   * vroniGraphGraphics
   * routeGraphGraphics
   * obstacleGraphGraphics
   * bifurcationGraphGraphics
   */
  
  using namespace Graphics_2D;
  datumItem = new CoordinateSystem;
  datumItem->setZValue(GraphicItemZLevels::DatumItemZLevel);
 
  
  motionBoundaryItem = new Graphics_2D::Polygon(motionBoundary,datumItem);  
  motionBoundaryItem->setZValue(GraphicItemZLevels::EnvironmentGraphicZLevel);
  motionBoundaryItem->setBrush(Qt::NoBrush);
  motionBoundaryItem->setPen(QPen(Qt::red, 5, Qt::SolidLine, Qt::RoundCap,Qt::RoundJoin)); //FIXME: Magic Numbers : Pen width.
  motionBoundaryItem->setFlag(QGraphicsItem::ItemStacksBehindParent);
  
  
  environmentItem = new Arena(environment2D,datumItem); 
  environmentItem->setZValue(GraphicItemZLevels::EnvironmentGraphicZLevel);
  environmentItem->setBrush(Qt::white); //FIXME: Magic Numbers: Color of the environment polygon
  environmentItem->setFlag(QGraphicsItem::ItemStacksBehindParent);


  //
  // Initializing (at this point) empty QGraphicsItemGroups
  //
  /** \todo This would be a good place to figure out the z-ordering... */
  
  obstacleCellGraphics      = new QGraphicsItemGroup(datumItem);
  obstacleCellGraphics->setZValue(GraphicItemZLevels::ObstacleCellGraphicZLevel);
  
//   routeGraphGraphics        = new GraphItem(q,datumItem); //NOTE: Created/filled through generateQtGraphicsFromGraph
  obstacleGraphGraphics     = new QGraphicsItemGroup(datumItem); //NOTE: Created/filled through generateQtGraphicsFromGraph
  obstacleGraphGraphics->hide();
#ifdef DEBUG_GRAPHICS
  bifurcationGraphGraphics  = new QGraphicsItemGroup(datumItem); //NOTE: Created/filled through generateQtGraphicsFromGraph
  bifurcationGraphGraphics->hide();
#endif // DEBUG_GRAPHICS    
  for( auto& pair : obstacleData )
  { 
    pair.second.setParentItem(datumItem); 
    pair.second.setZValue(NodePrivate::ObstacleGraphicZLevel);
  }
  
  /** \todo Store all connections to lambdas somewhere so that they can be
   * deleted when a Node is reinitialized. (Lamda/Functor connections will 
   * automatically disconnect if the sender is destroyed.) */
  Q_Q(Node); Q_ASSERT(q);
  Node::connect(q,&Node::comGraphChanged,
                [=](){ updateComGraphGraphics();} );
  
}

void NodePrivate::initializeModels()
{
  Q_Q(Node); Q_ASSERT(q);
  
  
  blipModel = new BlipModel(&blipData,q);
  
  QObject::connect(q,&Node::blipAdded,
    [this](NodeId const & id){blipModel->addBlipToModel(id);});
  QObject::connect(q,&Node::aboutToDeleteBlip,
    [this](NodeId const& id){blipModel->removeBlipFromModel(id);});
  
//   //The ownship blip already exists, hence no signal will be emitted for it
//   blipModel->addBlipToModel(ownshipId);
//   
  
  
}
#endif // WITH_GRAPHICS

void NodePrivate::initializeComGraph()
{ 
#ifdef WITH_GRAPHICS
  {
    updateComGraphGraphics();

//     //FIXME: I am not sure what happens when a Node get's reinitializes as a
//     // potentially existing old connection doesn't get disconnected in that case
//     Q_Q(Node); Q_ASSERT(q);
//     QObject::connect(q, &Node::comGraphChanged,
//                      &updateComGraphGraphics );
    
    Q_Q(Node); Q_ASSERT(q);
    q->setComGraphVisibility(true); //FIXME: Magic Numbers: initial ComGraph visibility.
  }
#endif // WITH_GRAPHICS

}

void NodePrivate::initializeRouteGraph()
{
  updateRouteGraph();
#ifdef WITH_GRAPHICS
  {
    Q_Q(Node); Q_ASSERT(q);
    q->setRouteGraphVisibility(true); //FIXME: Magic Numbers: initial RouteGraph visibility.
  }
#endif // WITH_GRAPHICS
}


// void NodePrivate::initializeVisibilityGraph()
// {
//  #ifdef WITH_GRAPHICS
//   {
//     Q_Q(Node); Q_ASSERT(q); 
//     q->setVisibilityGraphVisibility(false); //FIXME: Magic Numbers: initial VisibilityGraph visibility.
//   }
// #endif // WITH_GRAPHICS
// }

void NodePrivate::initializeDerivedGraphs()
{
  updateDerivedGraphs();
#ifdef WITH_GRAPHICS
  {
    Q_Q(Node); Q_ASSERT(q);
    q->setObstacleGraphVisibility(false); //FIXME: Magic Numbers: initial ObstacleGraph visibility.
  }
#endif // WITH_GRAPHICS
}

void NodePrivate::initializeOwnshipBlip()
{
  Q_Q(Node); Q_ASSERT(q);
  
  ownshipBlip = new OwnshipBlip(this);

  initializeBlip(q->id(),q->type(), q->qualifiedName(), QStringList(), ownshipBlip);
  
  int hFov = 0; // horizontal Field-of-View of the camera
  if( xmlSettings->camera().present() )
  { hFov = xmlSettings->camera().get().hFov(); }
  ownshipBlip->setHorizontalFov(hFov);

  // recompute the closest obstacle whenever the position of the ownship changes
  Node::connect(ownshipBlip, &Blip::positionChanged,
    [=]()
    {
      ElementId id = getClosestObstacle(ownshipBlip->position2D(),
                                     ownshipBlip->closestObstacleId());
      ownshipBlip->setClosestObstacleId(id); 
      statusMessage->closestObstacle = id;
    }
  );
    
}

AMG::Vector NodePrivate::getXmlInitialPosition()
{
  auto positionXml = xmlSettings->coordinates();
  
  auto coordinateList = XML::splitXmlStringList( positionXml );
  
  if( coordinateList.size() != 1 )
  {
    Q_Q(Node); Q_ASSERT(q);
    //NOTE: qFatal() will _abort_ on a UNIX system!
    qFatal("%s: Cannot initialize ownship position from the given XML initialization settings."
      ,q->qualifiedName().toStdString().c_str());
  }


  auto posCoords = coordinateList.first();
  
  double longitude_deg  = posCoords.at(0).toDouble();
  double latitude_deg   = posCoords.at(1).toDouble();
  double altitude_m     = posCoords.at(2).toDouble();
  double xECEF,yECEF,zECEF; // prepare as return values

  AMG::GIS::geodetic2ecef(AMG::Units::degree2radian(longitude_deg),
                          AMG::Units::degree2radian(latitude_deg),
                          altitude_m,
                          xECEF,yECEF,zECEF);
  
  AMG::Vector position(xECEF,yECEF,zECEF,AMG::CoSy::getECEF());
  position.setFrameOfReference(AMG::CoSy::getDatumFrame(),true); 
  
  return position;
}

void NodePrivate::initializePosition()
{
  

  // put the position into the node database.
  auto pos = getXmlInitialPosition();
  ownshipBlip->setPosition3D(pos);
  
  updateTertiaryBlipData(ownshipBlip);
  
  Q_Q(Node); Q_ASSERT(q);
  qDebug() << q->qualifiedName() << ": Positioned at" << QString(AMG::GIS::geodeticPositionString(pos).c_str());
//   qDebug() << q->qualifiedName() << ": ECEF coords: (" << xECEF << "," << yECEF << "," << zECEF <<").";
//   qDebug() << q->qualifiedName() << ": Datum coords: (" << ownshipData().position3D.coords(0)
//     << "," << q->ownshipData().position3D.coords(1) << "," << ownshipData().position3D.coords(2) <<").";
}

void NodePrivate::updateTertiaryBlipData(Blip* const blip)
{

  {/** update the position of the blip in the comGraph
    *
    * \internal I would have liked to hide this away somewhere in the 
    * Blip::setPosition3D routines, but that would have required a connection
    * of the Blip to the Node. (Maybe by making the Blip a QObject and using
    * parent() ?) Putting it here though made all that not necessary and 
    * it might actually better reflect that there is (currently) no real 
    * connection between a Blip and the comGraph 
    * 
    * \todo I _really_ would like to make Blip the vertex type of the comGraph.
    */
  
    auto positionMap = boost::get(boost::vertex_position_2D, comGraph);
    positionMap[blip->comGraphVertex()] = blip->position2D();
  }
  
  /** \internal The FOV polygon is currently not used other than displaying it
   * for joystick controlled Blips. As such, I introduced a simple guard to
   * reduce the number of (currently unnecessary) update calls. */
#ifdef WITH_GRAPHICS
  if(blip->isFovPolygonVisible())
  { updateFovPoly(blip); }
#endif // WITH_GRAPHICS
  
  /** \internal There are two principal ways which could be used to keep the COM
   * graph current: listen to the Blip::rfPolygonChanged() signal and use the 
   * emitters ID to identify the dynamic blip or use the true/false return value
   * of the updateRfPoly() method.
   * I elected to the later to be sure about a proper "order" of things: the 
   * signal connections might end up being processed in a different thread which
   * means that it could happen that the COM graph has not yet updated when 
   * returning from this function. But that is needed in the overall scheme of 
   * the isMessageReceivable() call (which propagates down to here...). */
  if( updateRfPoly(blip) )
  { updateComGraph(blip->id()); }
  
  
  
  /** \note The closestObstacleId  is tertiary data for the ownshipBlip and 
   * primary data for all other blips. As such it is not updated here.
   * (The update is triggered from the OwnshipBlip::positionChanged signal.
   * The connection is set up in NodePrivate::initializeOwnshipBlip(). )
   */
}


ElementId NodePrivate::getClosestObstacle( const Geometry::Point_2D& location,
                                        const ElementId& hint)
{
  /** \todo change this from an vertex index based routine  to a vertex 
   * descriptor based one. */
    
  Q_Q(Node); Q_ASSERT(q);

  using namespace boost;
  
    
  bool search_all_obstacles = (hint == NULL_ELEMENT_ID)
                              or (obstacleDelaunayFaceData.count(hint)!=1);
  
  if( search_all_obstacles == false )
  { 
    // check if the blip has left the cell of the current obstacle
    if( !geometry::within(location, obstacleDelaunayFaceData[hint]) )
    {
      // ... and search amongst its neighbors to find the new one.
      auto id_map = get(vertex_id, obstacleGraph);
      
      graph_traits<ObstacleGraph::Graph>::vertex_iterator oi, oi_end;
      for( tie(oi,oi_end) = vertices(obstacleGraph);
            id_map[*oi] != hint and oi!=oi_end;
          ++oi )
      { /*emtpy*/ }
      
      if( oi==oi_end )
      {
//           qDebug() << q->qualifiedName() 
//             << ": Couldn't determine current obstacle association for"
//             << blip->name();
          
        // search was unsuccessfull, try again within all obstacles
        search_all_obstacles = true;
      }
      else
      {
        // get the neighboring obstacles and check them
        ObstacleGraph::Traits::adjacency_iterator  ni, ni_end;
        for(tie(ni,ni_end)=adjacent_vertices(*oi, obstacleGraph); ni!=ni_end; ++ni )
        {
          int id_neighbor = id_map[*ni];
          if( geometry::within( location, obstacleDelaunayFaceData[id_neighbor]) )
          {
//               blip->setClosestObstacle(i_neighbor);
//               qDebug() << q->qualifiedName() << ": Closest obstacle to"
//                 << blip->name() << "is" << obstacleData[i_neighbor].objectName();
//               break;    
            return id_neighbor;
          }
        }
        
        if( ni==ni_end )
        {
//             qDebug() << q->qualifiedName()
//             << ": Couldn't find" << blip->name() 
//             << "in any of the neighbor cells of obstacle" << i_obstacle
//             << "(" << obstacleData[i_obstacle].objectName() << ").";
          search_all_obstacles = true;
        }
          
      }
    }
    else
    {
      // the location is closests to the obstacle given as hint
      return hint;
    }
  }
  
  if( search_all_obstacles == true)
  {
    for( auto& pair : obstacleDelaunayFaceData )
    {
      Geometry::Polygon_2D const&  cell  = pair.second;
      
      if( geometry::covered_by(location,cell) )
      {
        return pair.first;
      }
    }
  }
  
  qCritical() << q->qualifiedName() 
    << ": Couldn't find the closest obstacle for the given location.";
  
  return NULL_ELEMENT_ID; // TODO: this is an error, treat it like that...
}

void NodePrivate::initializeStates()
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
    using namespace IDL::Data;
    
    standby = new State(IDL::Data::STATE_ID_N_STANDBY ,running_itemRegion);
    active  = new State(IDL::Data::STATE_ID_N_ACTIVE ,QState::ParallelStates, running_itemRegion);
    active_nodeRegion = new State(IDL::Data::STATE_ID_N_ACTIVE_REGION ,active);
    joining = new Activity(IDL::Data::STATE_ID_N_JOINING ,active_nodeRegion);
    operational = new State(IDL::Data::STATE_ID_N_OPERATIONAL ,active_nodeRegion);
    leaving = new Activity(IDL::Data::STATE_ID_N_LEAVING ,active_nodeRegion);
  
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
    running_itemRegion->setInitialState(standby);
    active_nodeRegion->setInitialState(joining);
  }
  
  //
  // add the transitions
  // ===================
  // NOTE: When creating custom transitions (i.e. subclassing QAbstractTransition)
  //   do remember that a state to which a transition is added assumes possession
  //   of that transition pointer. As a result, a transiotn pointer cannot be
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
    
    /*auto doneJoining =*/
    joining->addTransition(joining, SIGNAL(finished()),operational);
    
    /*auto doneLeaving =*/
    leaving->addTransition(leaving, SIGNAL(finished()),standby);
    
    //
    // Command Transitions
    //
    
    using namespace Transitions;
    
    auto joinNetwork_frmStby = new CommandTransition(IDL::Commands::N_JOIN_NETWORK);
    joinNetwork_frmStby->setTargetState(joining);
    standby->addTransition(joinNetwork_frmStby); // assumes possession of transition pointer
    joinNetwork_frmStby = nullptr; // prevent reuse 
    
    auto joinNetwork_frmLeaving = new CommandTransition(IDL::Commands::N_JOIN_NETWORK);
    joinNetwork_frmLeaving->setTargetState(joining);
    leaving->addTransition( joinNetwork_frmLeaving ); // assumes possession of transition pointer
    joinNetwork_frmLeaving = nullptr; // prevent reuse 

    auto leaveNetwork_frmJoining = new CommandTransition(IDL::Commands::N_LEAVE_NETWORK);
    leaveNetwork_frmJoining->setTargetState(leaving);
    joining->addTransition( leaveNetwork_frmJoining ); // assumes possession of transition pointer
    leaveNetwork_frmJoining = nullptr; // prevent reuse 

    auto leaveNetwork_frmCnctd = new CommandTransition(IDL::Commands::N_LEAVE_NETWORK);
    leaveNetwork_frmCnctd->setTargetState(leaving);
    operational->addTransition(leaveNetwork_frmCnctd); // assumes possession of transition pointer
    leaveNetwork_frmCnctd = nullptr; // prevent reuse 
    
    
  }
}

void NodePrivate::initializeBlip( const NodeId& id, const Node::Type& type, const QString& name, const QStringList& propertyList, Blip* blip )
{
  Q_Q(Node); Q_ASSERT(q);
  
  static uint BLIP_COLOR_INDEX = 0;
  
  if( blip == nullptr )
  { blip = new Blip(q); }// generate a generic Blip
  
  blip->setId(id);
  blip->setObjectName(name);
  blip->setType(type);

#ifdef WITH_GRAPHICS
  blip->setMainColor( Graphics_2D::Colors::colorBrewerSet(BLIP_COLOR_INDEX++) );
  blip->setDatumItem(datumItem);  
  blip->setPalette( &(palette.blip) );
#endif // WITH_GRAPHICS
  
  auto blipVertex = boost::add_vertex(comGraph);
  blip->setComGraphVertex( blipVertex );
  
  comGraphNameMap = boost::get(boost::vertex_name, comGraph); //TODO: Do I really need to get the map every time a vertex is added?
  comGraphNameMap[blipVertex] = blip->name().toStdString();
  
  comGraphIdMap = boost::get(boost::vertex_id, comGraph); //TODO: Do I really need to get the map every time a vertex is added?
  comGraphIdMap[blipVertex] = blip->id();
  
  comGraphPositionMap = boost::get(boost::vertex_position_2D, comGraph); //TODO: Do I really need to get the map every time a vertex is added?
  comGraphPositionMap[blipVertex] = Geometry::Point_2D(0.0,0.0);
  
  blipData[id] = blip;
    
  // set blip props from participant properties  
  //NOTE: the iterator advance is 2 as there are always key,value pair 
  // sequences and we are only interested in the keys.
  for( auto it = propertyList.begin(); it != propertyList.end() ; it +=2 )
  {
    if( QString("hFov") == *it)
    { blip->setHorizontalFov( (it+1)->toInt() ); }
  }
  
  
  /** \todo Maybe this would be a good point to set up some sort of rebroadcasting
   * for Blip signals? Like Bip::signal() to Node::blipSignal(blipId) ? */
  
  Node::connect(blip, &Blip::modeChanged,
    [=](){ updateRfConnectedArea(); } );
  
}

void NodePrivate::unInitializeNode()
{
  Q_Q(Node); Q_ASSERT(q);
//   qDebug() << q->qualifiedName() << ": NodePrivate::unInitializeNode()";

  {/// - Uninitialize State topic
    stateMessage = nullptr;
    
    delete stateTopic;
    stateTopic = nullptr;
  }
  
  {/// - Uninitialize State topic
    statusMessage = nullptr;
    
    delete statusTopic;
    statusTopic = nullptr;
  }
  
  {/// - Uninitialize Datum topic
    datumMessage = nullptr;
    
    delete datumTopic;
    datumTopic = nullptr;
  }
  
  { /// uninitialize Flightplan topic 
    flightplanMessage = nullptr;
    
    delete flightplanTopic;
    flightplanTopic = nullptr;
  }
  
  
  {/** - Uninitialize Datalink: a parent/child relationship deletes dds */
//     delete dds;
//     dds = nullptr;
  }
  
  {/// - Uninitialize blipData
    for( std::pair<const NodeId,Blip*>& pair : blipData )
    { 
      boost::remove_vertex(pair.second->comGraphVertex(), comGraph);
      
      delete pair.second;
      pair.second = nullptr;
    }
    
    //NOTE: as ownshipBlip is also in blipData, the memory has already been freed.
    ownshipBlip = nullptr;
  }

  {/// - Uninitialize obstacleData 
    obstacleData.clear();
  }
  
#ifdef WITH_GRAPHICS
  {/** - Uninitialize graphics
    * \todo Do I need to remove the datumItem from the scene before I can 
    * delete it?
    */
  
    datumItem->setParentItem(nullptr); // remove from any potential parent
    delete datumItem; // this also deletes all children
    datumItem = nullptr;

// NOTE: the arena is now a child of the datum as well    
//     environmentItem->setParentItem(nullptr);
//     delete environmentItem;
//     environmentItem = nullptr;    
      
    delete blipModel;
    blipModel = nullptr;
  }
#endif // WITH_GRAPHICS



}

void NodePrivate::setOwnshipStateVector( const AMG::Vector& position
                                        ,const AMG::Vector& velocity
                                        ,const AMG::EulerAngleTupel& attitude )
{

  /** as the ownshipBlip also stores its data in the stateMessage, the respective
   * methods of the ownshipBlip can be used to generaet a stateMessage that
   * reflect the wanted state data.
   * 
   * \internal Yes, this is a double conversion, but it reduces code redundancy.
   */  
    
  ownshipBlip->setPosition3D(position);
  ownshipBlip->setVelocity(velocity);
  ownshipBlip->setAttitude(attitude);
  
  Q_Q(Node); Q_ASSERT(q);
  /** although this will essentially call again all the ownshipBlip methods 
   * used above, the method is mainly used as it updates all the graphics, etc.
   */
  q->processReceivedMessage(ownshipId, *stateMessage); // this will
}

void NodePrivate::deleteBlip(NodeId const &id)
{
  QMutexLocker locker(&blipDeletionMutex);
  
  // check if that blip is indeed still in the database
  if( !blipExists(id) )
  { return; }  
  
  
  
  Blip* blip = blipData.at(id);
  
  Q_Q(Node); Q_ASSERT(q);
  QString blipName = blip->name();
  qDebug() << q->qualifiedName() << ": Deleting"<< blipName<<".";
  

  boost::clear_vertex(blip->comGraphVertex(),comGraph); 
  boost::remove_vertex(blip->comGraphVertex(),comGraph); //NOTE: call clear_vertex before using remove_vertex!
    
  /** \internal It is not necessary to call blipModel->removeBlipFromModel(id)
   *  as this is taken care of by a signal/slot connection. */
  
  blipData.erase(id);
  delete blip;
  blip = nullptr;

  updateComGraph(ownshipId);
  
  qDebug() << q->qualifiedName() << ": Deleting"<< blipName<<". DONE.";
}

Geometry::Point_2D NodePrivate::closestPointOnRouteGraph(Blip const * const blip)
{
  return closestPointOnRouteGraph(blip->position2D(), blip->closestObstacleId());
}

Geometry::Point_2D NodePrivate::closestPointOnRouteGraph(Geometry::Point_2D const& fromHere,
                                                         ElementId const& closestObstacleHint )

{
  
  ElementId hint = closestObstacleHint;
  
  if( hint == NULL_ELEMENT_ID )
  { hint = getClosestObstacle(fromHere); }
  
  
  Geometry::Polygon_2D cell = obstacleDelaunayFaceData[hint];
  
  if( hint == motionBoundaryElementId )
  { 
    // Check that there is only one hole in the polygon, the outer ring of
    // the route graph.
    Q_ASSERT(cell.inners().size() == 1); 
    
    Geometry::Point_2D closestPoint = Geometry::closestPointOnRing(fromHere, cell.inners().front()); 
    return closestPoint;
  }
  else
  {
   Geometry::Point_2D closestPoint = Geometry::closestPointOnRing(fromHere, cell.outer()); 
   return closestPoint;
  }
}



void NodePrivate::updateRouteGraph()
{
  
  Q_Q(Node); Q_ASSERT(q);
 
  /** \internal Step 1: Update the vroniGraph. */
  {  
    qDebug() << q->qualifiedName() << ": Updating route graph (running VRONI).";
    
  //     boost::geometry::correct(environment2D);
  //     vroni.setVroniSiteData(environment2D); // (re-)set the VRONI data to match the new arena
    
    /** \todo Deal with skipped obstacles when computing the VRONI environment.
     * Currently it is not guaranteed that all obstacles in obstacleData are 
     * actually part of the environment. Intersecting obstacles are skipped.
     * \sa updateArena() */
    
    vroni.setVroniSiteData(motionBoundary,obstacleData);
    vroni.computeVD(); // use the default settings.
    vroni.computeWMAT(0); //FIXME: Magic Numbers : minumum clearance distance between obstacles

#if defined WITH_GRAPHICS && defined DEBUG_GRAPHICS
if(false){ // DEBUG graphics for libvroni
        
        auto vroniSiteGraphics = vroni.getSiteGraphics();
        vroniSiteGraphics->setParentItem(datumItem);
        
        auto vroniNodeGraphics = vroni.getNodeGraphics();
        vroniNodeGraphics->setParentItem(datumItem);

        auto vroniEdgeGraphics = vroni.getEdgeGraphics();
        vroniEdgeGraphics->setParentItem(datumItem);

        if(true){ // site-to-shape association
          auto siteShapeGraph = vroni.createSiteShapeAssociationGraph();
          
          auto positionMap = boost::get(boost::vertex_position_2D, siteShapeGraph);
          delete vroniSiteShapeGraphics;
          vroniSiteShapeGraphics = Graphics_2D::generateQtGraphicsFromGraph(
            siteShapeGraph, positionMap
            ,Graphics_2D::Pens::Outline(Qt::cyan)
            ,Graphics_2D::Pens::Outline(Qt::cyan)
            ,/*drawEdges*/ true
            ,/*drawVertices*/ false);
          
          vroniSiteShapeGraphics->setParentItem(datumItem);
          vroniSiteShapeGraphics->setZValue(GraphicItemZLevels::VroniSiteShapeGraphicsZLevel);
//           vroniSiteShapeGraphics->hide();
        }
        if(true){ // edge-to-site association
          auto edgeSiteGraph = vroni.createEdgeSiteAssociationGraph();
          
          auto positionMap = boost::get(boost::vertex_position_2D, edgeSiteGraph);
          delete vroniEdgeSiteGraphics;
          vroniEdgeSiteGraphics = Graphics_2D::generateQtGraphicsFromGraph(
            edgeSiteGraph, positionMap
            ,Graphics_2D::Pens::Outline(Qt::magenta)
            ,Graphics_2D::Pens::Outline(Qt::magenta)
            ,/*drawEdges*/ true
            ,/*drawVertices*/ false);

          vroniEdgeSiteGraphics->setParentItem(datumItem);
          vroniEdgeSiteGraphics->setZValue(GraphicItemZLevels::VroniEdgeSiteGraphicsZLevel);
//           vroniEdgeSiteGraphics->hide();
        }
        if(true){ // edge-to-shape associaton 
          auto shapeToEdgeMap = vroni.determineVoronoiCells();
          auto edgeShapeGraph = vroni.createEdgeShapeAssociationGraph(shapeToEdgeMap);
          
          auto positionMap = boost::get(boost::vertex_position_2D, edgeShapeGraph);
          delete vroniEdgeShapeGraphics;
          vroniEdgeShapeGraphics = Graphics_2D::generateQtGraphicsFromGraph(
            edgeShapeGraph, positionMap
            ,Graphics_2D::Pens::Outline(Qt::gray)
            ,Graphics_2D::Pens::Outline(Qt::gray)
            ,/*drawEdges*/ true
            ,/*drawVertices*/ false);
          
          vroniEdgeShapeGraphics->setParentItem(datumItem);
          vroniEdgeShapeGraphics->setZValue(GraphicItemZLevels::VroniEdgeShapeGraphicsZLevel);
//           vroniEdgeShapeGraphics->hide();
        }  
        
}
#endif // WITH_GRAPHICS&& DEBUG_GRAPHICS
      
    vroniGraph = vroni.extractWmatGraph();
  
#if defined WITH_GRAPHICS && defined DEBUG_GRAPHICS
{
    delete vroniGraphGraphics;
    vroniGraphGraphics = Graphics_2D::generateQtGraphicsFromGraph(vroniGraph);
    vroniGraphGraphics->setParentItem(datumItem);
    vroniGraphGraphics->setZValue(GraphicItemZLevels::VroniGraphGraphicZLevel);
    vroniGraphGraphics->hide();
}
#endif // WITH_GRAPHICS&& DEBUG_GRAPHICS
    
  }
  
  /** \internal Step 2: Compute the cyclic routeGraph from the vroniGraph. */
  {
    // an intermediady cyclic version of the vroniGraph. This graph has the 
    // correct shape for the routeGraph, but it's structure is wrong - it still
    // is a VroniGraph.
    auto cyclicVroniGraph = Graph::pruneNonCyclicBranches( vroniGraph ); 

    using namespace boost;
    
    VroniGraph::Vertex_Position2d_Map vPosMap = get(vertex_position_2D, cyclicVroniGraph);
    
    std::map<VroniGraph::Vertex_Descriptor, RouteGraph::Vertex_Descriptor> rVertexMap;
    
    
    
    // copy the vertices and create the associated locations
    VroniGraph::Vertex_Iterator vi,vi_end;
    for( tie(vi,vi_end)=vertices(cyclicVroniGraph); vi!=vi_end; ++vi)
    {
      Location& location = makeLocation(vPosMap[*vi]);
      
      RouteGraph::Vertex_Descriptor rVertex = add_vertex(routeGraph);
      rVertexMap[*vi] = rVertex;
      location.setRouteGraphVertex(rVertex);
      
      
      put(vertex_locationId,routeGraph,rVertex,location.id());
      put(vertex_index, routeGraph, rVertex, get(vertex_index, cyclicVroniGraph, *vi));
    }
    
    // recreate the edges and transfer the weights (lengths).
    VroniGraph::Edge_Iterator ei,ei_end;
    for( tie(ei,ei_end)=edges(cyclicVroniGraph); ei!=ei_end; ++ei)
    {
      RouteGraph::Edge_Descriptor rEdge
        = add_edge(rVertexMap[source(*ei,cyclicVroniGraph)]
                  ,rVertexMap[target(*ei,cyclicVroniGraph)]
                  ,routeGraph
                  ).first;
      
      double weight = get(edge_weight, cyclicVroniGraph, *ei);
      put(edge_weight, routeGraph, rEdge, weight );
      
      //NOTE: Copying over the edge indices is important for work on the derived
      // graphs, particular the bifurcation graph!
      put(edge_index, routeGraph, rEdge, get(edge_index, cyclicVroniGraph, *ei));
    }
    
    routeGraphIsCurrent = true;

  }
    
    
  /** \internal Step 3: Generate graphics.... */  
#ifdef WITH_GRAPHICS
{ // routeGraph graphics
    delete routeGraphGraphics;
    routeGraphGraphics = nullptr;
       
    using namespace boost;
    
    auto locationIdMap = get(vertex_locationId, routeGraph);
    auto locationMap = make_compose_property_map(location_pMap, locationIdMap);
    
//     routeGraphGraphics = Graphics_2D::generateQtGraphicsFromGraph(
//        routeGraph, locationMap
//       ,Graphics_2D::Pens::Outline(Graphics_2D::Colors::routeGraph)
//       ,Graphics_2D::Pens::Outline(Graphics_2D::Colors::routeGraph)
//       ,/*drawEdges*/ true
//       ,/*drawVertices*/ true);
//     routeGraphGraphics->setParentItem(datumItem);
//     routeGraphGraphics->setZValue(GraphicItemZLevels::RouteGraphZLevel);
    
    routeGraphGraphics = new Graphics_2D::GraphItem(q,datumItem);
    routeGraphGraphics->generate(routeGraph, locationMap);
    routeGraphGraphics->setEdgePen(Graphics_2D::Pens::Outline(Graphics_2D::Colors::routeGraph));
    routeGraphGraphics->setVertexPen(Graphics_2D::Pens::Outline(Graphics_2D::Colors::routeGraph));
    routeGraphGraphics->setZValue(GraphicItemZLevels::RouteGraphZLevel);
    
    routeGraphGraphics->show();
    routeGraphGraphics->vertices()->hide();
    
    
    
}
#endif // WITH_GRAPHICS 

    

}


// void NodePrivate::updateDerivedGraphs() { return ; }
void NodePrivate::updateDerivedGraphs()
{ 
  
  QMutexLocker locker(&derivedGraphUpdateMutex);
  
  auto vPosMap =  boost::get(boost::vertex_position_2D, vroniGraph);
  using FaceGraph = Simple2dGraph::Graph; /// Previously: ObstacleGraph::Graph;
  Graph::FaceGeometry_visitor< VroniGraph::Graph /// the type of the visited graph
                             , boost::property_map< VroniGraph::Graph, boost::vertex_position_2D_t>::type  /// the vertex position map type of the visited graph
                             , /*BiGraph*/ Simple2dGraph::Graph /// the type used to store the bifurcation graph in
                             , /*FaceGraph*/ FaceGraph /// the type used to store the (face) dual of the visited graph in
                             > visitor(vroniGraph, vPosMap);

  /**  \internal Step 1: Create a planar embedding and run the visitor
   *
   * ISSUE:  
   * There could be a problem: The route graph may be planar, but the
   * face assignment might not be unique:
   * 
   * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
   * A------------O---------------B
   * |           / \              |
   * |   1      a 2 b       3     |
   * |           \ /              |
   * A------------O---------------B
   * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
   * 
   *  is identical to  
   * 
   * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
   * A------------O---------------B
   * |           / \              |
   * |   1      b 2 a       3     |
   * |           \ /              |
   * A------------O---------------B  
   * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
   *   
   * RESPONSE:  
   * This is solved, use an external embedding that is correct ;)  
   * In this case,VroniInterface::extractPlanarEmbedding is used.
   * 
   */
  { 
    using namespace boost; 
    
    //Define the storage type for the planar embedding
    using embedding_storage_t = std::vector< std::vector< graph_traits<VroniGraph::Graph>::edge_descriptor > > ;
    using embedding_pMap_t =  iterator_property_map
      < embedding_storage_t::iterator, /// the underlying data structure holding the property 
        property_map<VroniGraph::Graph, vertex_index_t>::type /// the offset map used to translate vertex_descriptor to property
      >;

    // Create the planar embedding
    embedding_storage_t embedding_storage(num_vertices(vroniGraph));
    embedding_pMap_t embedding(embedding_storage.begin(), get(vertex_index,vroniGraph));
    
    vroni.extractPlanarEmbedding( vroniGraph,
                                  embedding,
                                  [](VRONI::edge* e) {return e->w_mat.in_w_mat;}
                                );
    
    /** \internal Run the visitor over the route graph  */ 
    planar_face_traversal(vroniGraph, embedding, visitor);  
    
#if defined WITH_GRAPHICS && defined DEBUG_GRAPHICS
{ 
    using namespace boost;

    delete faceGraphGraphics;
    faceGraphGraphics = Graphics_2D::generateQtGraphicsFromGraph(
        visitor.faceGraph, get(vertex_position_2D, visitor.faceGraph)
        ,/*edgePen*/ Graphics_2D::Pens::Outline(Qt::red)
        ,/*vertexPen*/ Graphics_2D::Pens::Outline(Qt::darkRed)
        ,/*drawEdges*/ true
        ,/*drawVertices*/ true);
    
    faceGraphGraphics->setParentItem(datumItem); 
    faceGraphGraphics->setZValue(NodePrivate::FaceGraphGraphicZLevel);
    faceGraphGraphics->hide();
}
#endif // WITH_GRAPHICS&& DEBUG_GRAPHICS
  }

  // A map between the visitor.facegraph vertices and the corresponding 
  // obstacle graph vertices
  std::map< typename boost::graph_traits<FaceGraph>::vertex_descriptor
        , ObstacleGraph::Vertex_Descriptor
        > faceVertex_to_obstacleVertex_map;      
  
  // the faceGraph vertex identified as corresponding to the outer face
  boost::graph_traits<FaceGraph>::vertex_descriptor outerFaceVertex
    = boost::graph_traits<FaceGraph>::null_vertex();
    
  // the obstacleGraph vertex indentifying the outer face, i.e. the motion 
  // boundary
  ObstacleGraph::Vertex_Descriptor outerObstacleVertex
      = locationData[motionBoundaryLocationId].obstacleGraphVertex();
        
  /** \internal Step 2: Create the obstacle graph from the face graph.
   * 
   * The problem now is that we have to graphs, the faceGraph (from the visitor)
   * and the obstacleGraph, which we would like to construct/extract from that:
   *   
   * - The faceGraph's vertex positions are the centroids of the route graph 
   *   faces.
   * - The faceGraph's vertex indices are corresponding to the face geometries
   *   stored in the visitor.faceGeometries vector.
   * - The faceGraph's edges correspond to neighboring faces
   * 
   * - The obstacleGraph's vertex position are the centroids of the obstacles
   *   (which are inside the faces), respectively the associated location.
   * - The obstacleGraph's vertex indices are corresponding to the order in which
   *   the obstacles were added to the Node's obstacle database (hence they are 
   *   fairly useless and the ID is used instead)/
   * - The obstacleGraph currently does not have any edges (or only ones that
   *   should be considered invalid).
   *   
   * The task at hand is to merge the information stored in the  face graph
   * and the obstacle graph and associate the face geometries with their 
   * corresponding obstacles:
   *  
   * - match the faces with their corresponding obstacles
   * - connect the obstacle vertices according to the respective face graph
   *   edges
   * - extract and reorder the faceGeometries so that the obstacleGraph's vertex
   *   indices correspond to it
   *   
   * After that, the face graph is useless and can be discarded.
   *   
   * However, there is the issue with the "outer face" that contains all 
   * obstacles...
   */   
  {
    using namespace boost;
    
    // reset face/cell data
    obstacleDelaunayFaceData.clear();  
      
    qDebug() << "Removing all edges from the" << num_vertices(obstacleGraph) 
      << "vertices of the obstacleGraph.";
      
    // remove all edges from the obstacle graph
    ObstacleGraph::Vertex_Iterator vi, vi_end;
    for(tie(vi, vi_end)=vertices(obstacleGraph); vi!=vi_end; ++vi)
    { clear_vertex(*vi, obstacleGraph); }
            
    // the graph representing the faces  
    FaceGraph const& faceGraph = visitor.faceGraph;
    
//     using FaceGeometry = Geometry::Polygon_2D::ring_type;  
    
    /** \internal Create a property map interface to the face geometry and 
     * centroid data so that there is no (external) reliance on the vertex_index
     * property of the face graph. */
    
    auto faceIndexMap = get(vertex_index, faceGraph);
    auto faceGeometryMap = make_iterator_property_map(visitor.faceGeometries.begin(),faceIndexMap);
    auto faceCentroidMap = make_iterator_property_map(visitor.facePositions.begin(),faceIndexMap);
    
    
    /** \internal Find the largest face as that is the outer face and store it
     * the corresponding vertes as outerFaceVertex */
    {
      unsigned int outerFaceIndex = 0;
      Q_ASSERT(visitor.faceGeometries.size() > 0);
      
      auto largestFaceArea = geometry::area(visitor.faceGeometries[0]);
      for( unsigned int i=0; i<visitor.faceGeometries.size(); ++i)
      {
        auto faceArea =  geometry::area(visitor.faceGeometries[i]);
        if(faceArea > largestFaceArea)
        {
          largestFaceArea = faceArea;
          outerFaceIndex = i;
        }
      }
      
      Q_Q( Node ); Q_ASSERT(q);
      qDebug() << q->objectName() <<": Determined outer face index :" 
        << outerFaceIndex << ", area =" << largestFaceArea <<".";
      
      bool foundOuterFaceVertex = false;
      typename graph_traits<FaceGraph>::vertex_iterator vi, vi_end;
      for( tie(vi, vi_end)=vertices(faceGraph); vi!=vi_end; ++vi )
      {
        if( faceIndexMap[*vi]==outerFaceIndex )
        { 
          outerFaceVertex = *vi; 
          faceVertex_to_obstacleVertex_map[outerFaceVertex] = outerObstacleVertex;
          foundOuterFaceVertex = true;
          break;
        }
      }
      
      Q_ASSERT(foundOuterFaceVertex == true); // if this fails, the outerFaceVertex hasn't been assigned.
    }  
       
    auto obstacleLocationIdMap = get(vertex_locationId, obstacleGraph);
    auto obstacleLocationMap = make_compose_property_map(location_pMap, obstacleLocationIdMap);
    
   
    /** \internal Associate faceGraph and obstacleGraph vertices
     * 
     * traverse all vertices of the faceGraph and map them to the corresponding
     * obstacle graph vertices. */
    {        
      // a vector holding obstacle graph vertices that have not yet been 
      // correlated with a face
      std::set<ObstacleGraph::Vertex_Descriptor> unmatchedObstacles;  

      /** \internal Fill the obstacles vector, set the border flag of all 
       * vertices to false, and reinitialize the vertex_index of the obstacle 
       * graph */
      { 
        unsigned int index = 0;
        
        ObstacleGraph::Vertex_Iterator oi, oi_end;
        for(tie(oi,oi_end)=vertices(obstacleGraph); oi != oi_end; ++oi)  
        { 
          unmatchedObstacles.insert(*oi);
          put(vertex_borderFlag, obstacleGraph, *oi, false);
          put(vertex_index, obstacleGraph, *oi, index++);
        }  
        
        qDebug() << "Importet" << index << "obstacle vertices into unmatchedObstacles, including the 'fake' obstacle for the motion boundary.";
        
        Q_ASSERT( unmatchedObstacles.count(outerObstacleVertex)==1 );
        unmatchedObstacles.erase(outerObstacleVertex);
        
        qDebug() << "Importet" << unmatchedObstacles.size() << "obstacle vertices into unmatchedObstacles (actual obstacles, i.e. without the motion boundary).";
      }

#ifndef NDEBUG //NOTE: Double negative...    
      { //DEBUG: Check that all obstacles are associated with valid obstacleGraph
        // vertex descriptors
        for( auto& pair : obstacleData )
        {
//           qDebug() <<  pair.second;
          ObstacleGraph::Vertex_Descriptor obsVertex = pair.second.obstacleGraphVertex();
          Q_ASSERT( obsVertex != ObstacleGraph::Traits::null_vertex() );
          Q_ASSERT( unmatchedObstacles.count( obsVertex ) == 1 );
        }
      }
#endif // DEBUG
      
#if defined WITH_GRAPHICS && defined DEBUG_GRAPHICS
{ //DEBUG GRAPHICS
      delete faceToObstacleCorrespondence;
      faceToObstacleCorrespondence = new QGraphicsItemGroup(datumItem); 
      faceToObstacleCorrespondence->setZValue(GraphicItemZLevels::FaceToObstacleCorrespondenceGraphicZLevel);;
      faceToObstacleCorrespondence->hide();
}
#endif // WITH_GRAPHICS&& DEBUG_GRAPHICS

      typename graph_traits<FaceGraph>::vertex_iterator fi, fi_end;
      for(tie(fi, fi_end) = vertices(faceGraph); fi != fi_end; ++fi)  
      { 
        if( *fi == outerFaceVertex )
        { // dealing with the outer face 

//NOTE: This has been moved to the outer face detection part above.          
//           faceVertex_to_obstacleVertex_map[outerFaceVertex]
//             = locationData[motionBoundaryLocationId].obstacleGraphVertex();
        
          //
          // The outer cell is special as it has the motion boundary as its outer 
          // and face geometry as its inner. It also is not associated to an actual 
          // obstacle.
          // 

          Geometry::Polygon_2D outerCell;
          geometry::assign(outerCell.outer(), motionBoundary.outer());
          outerCell.inners().push_back(
            Geometry::getBoostEqualSimplified(faceGeometryMap[*fi]) );
          geometry::correct(outerCell); //NOTE: this is necesasry to switch the orientation as an outer became an inner.
          obstacleDelaunayFaceData[motionBoundaryElementId] = outerCell;

#if defined WITH_GRAPHICS && defined DEBUG_GRAPHICS
{ 
          auto faceCentroid = faceCentroidMap[*fi];
          auto obstacleLocation = obstacleLocationMap[faceVertex_to_obstacleVertex_map[*fi]];
          auto line = new Graphics_2D::Arrow(faceCentroid.x(), faceCentroid.y(),
                                              obstacleLocation.x(), obstacleLocation.y(),
                                              faceToObstacleCorrespondence
                                            );
          line->setPen(Graphics_2D::Pens::Outline(Qt::blue));
}
#endif // WITH_GRAPHICS&& DEBUG_GRAPHICS

        }
        else 
        { // dealing with a face that corresponds to an obstacle
          auto faceGeometry = Geometry::getBoostEqualSimplified(faceGeometryMap[*fi]);
          geometry::correct(faceGeometry);
          
          //
          // traverse all obstacles to find the one that is inside the faceGeomery
          //
          
          std::set<ObstacleGraph::Vertex_Descriptor>::iterator oi, oi_end; //NOTE: these are _NOT_ Vertex_Iterators! (although they can be used in the same way.)
          oi = unmatchedObstacles.begin();
          oi_end = unmatchedObstacles.end();   
          for(/*empty*/; oi != oi_end; ++oi)
          {
            auto obstacleLocation =  obstacleLocationMap[*oi];
            if( geometry::within(obstacleLocation,faceGeometry) )
            { 
              faceVertex_to_obstacleVertex_map[*fi] = *oi;
              
              Geometry::Polygon_2D obstacleCell;
          
              /** \note An artefakt of using the vroniGraph (which contains non-cyclic 
              * branches and which we needed to use in order to get a correct planar
              * embedding) is the fact that the face geometries contain zero-area 
              * spikes into their interior. getBoostEqualSimplified eliminates them.
              */
              geometry::assign(obstacleCell.outer(), faceGeometry );

              obstacleDelaunayFaceData[get(vertex_id, obstacleGraph, *oi)] = obstacleCell;

              // erase the just assinged obstacle so that it doesn't have to be 
              // checked for inclusion again.
              unmatchedObstacles.erase(oi);             
#if defined WITH_GRAPHICS && defined DEBUG_GRAPHICS
{ 
              auto faceCentroid = faceCentroidMap[*fi];
              auto line = new Graphics_2D::Arrow(faceCentroid.x(), faceCentroid.y(),
                                                obstacleLocation.x(), obstacleLocation.y(),
                                                faceToObstacleCorrespondence
                                                );
              line->setPen(Graphics_2D::Pens::Outline(Qt::blue));
}
#endif // WITH_GRAPHICS&& DEBUG_GRAPHICS
              break;
            }
          }
        }    
//         { //DEBUG
//           qDebug() << "faceVertex_to_obstacleVertex_map.size =" 
//             << faceVertex_to_obstacleVertex_map.size();
//          for( auto pair : faceVertex_to_obstacleVertex_map )
//          {
//            qDebug() 
//             << "Face" << get(vertex_index, faceGraph, pair.first) 
//             << "|"
//             << "Obstacle" << get(vertex_id, obstacleGraph, pair.second);
//          }
//         }
      } // end for loop
    }
    
    /** \internal Mark the border obstacles in the obstacleGraph
     * 
     * At this point all border flags should have been rest to "false" so this
     * section only needs to set the flag to "true" for the actual border 
     * elements. */
    {
      typename graph_traits<FaceGraph>::adjacency_iterator afi, afi_end;
      for( tie(afi, afi_end)=adjacent_vertices(outerFaceVertex, faceGraph); afi!=afi_end;++afi )
      { 
        Q_Q( Node ); Q_ASSERT(q);
        typename graph_traits<FaceGraph>::vertex_descriptor fVertex = *afi;
        
        if( fVertex == outerFaceVertex )
        { continue; } /// \todo Why does the outer face show up as an adjacent vertex to itself???
        
        if(faceVertex_to_obstacleVertex_map.count(fVertex)!=1)
        { 
          qWarning() << q->qualifiedName() << ": Cannot place border flag."
          << "(Missmatch between Face and Obstacle graph at vertex.)";
          continue;
        };
        
        ObstacleGraph::Vertex_Descriptor oVertex = faceVertex_to_obstacleVertex_map[fVertex];
        put(vertex_borderFlag, obstacleGraph, oVertex, true);
      }
      //NOTE: we cannot clear the outer face yet as the edge_to_face map still 
      // refercences it
//       clear_vertex(outerFaceVertex, faceGraph);
    }
 
    /** \internal Replicate the edges of the face graph in the obstacle graph. */
    {
      Q_Q( Node ); Q_ASSERT(q);
      ObstacleGraph::Edge_Index_Map::value_type edgeIndex = 0;  
      typename graph_traits<FaceGraph>::edge_iterator ei,ei_end;
      for( tie(ei,ei_end)=edges(faceGraph); ei!=ei_end; ++ei )
      {
        graph_traits<FaceGraph>::vertex_descriptor fSource,fTarget;
        fSource = source(*ei, faceGraph);
        fTarget = target(*ei, faceGraph);
        
        if( fSource == outerFaceVertex or fTarget == outerFaceVertex )
        { continue; }
        
        if(faceVertex_to_obstacleVertex_map.count(fSource)!=1)
        { 
          //NOTE: qFatal() will _abort_ on UNIX systems!
          qFatal("%s: Missmatch between Face and Obstacle graph at source."
          ,q->qualifiedName().toStdString().c_str());
          throw;
        };
        ObstacleGraph::Vertex_Descriptor oSource
          = faceVertex_to_obstacleVertex_map.at(fSource);
          
        if(faceVertex_to_obstacleVertex_map.count(fTarget)!=1)
        { 
          //NOTE: qFatal() will _abort_ on UNIX systems!
          qFatal("%s: Missmatch between Face and Obstacle graph at target."
          ,q->qualifiedName().toStdString().c_str());
          throw;
        };
        ObstacleGraph::Vertex_Descriptor oTarget
          = faceVertex_to_obstacleVertex_map.at(fTarget);
        
        ObstacleGraph:: Edge_Descriptor oEdge = 
          add_edge(oSource, oTarget, obstacleGraph).first;
                  
        put(edge_index, obstacleGraph, oEdge, edgeIndex++);
      }
    }

    obstacleGraphIsCurrent = true;
   
#ifdef WITH_GRAPHICS
    { // obstacle cell graphics
      
      qDeleteAll( obstacleCellGraphics->childItems() ); // free the memory   
      obstacleDelaunayFaceGraphics.clear(); //NOTE: The assoc. memory of the graphics items has been freed by deleting the container group above.
      
      for( auto& pair : obstacleDelaunayFaceData )
      {
        ElementId obstacleId = pair.first;
        Geometry::Polygon_2D & polygon = pair.second;
     
        auto cellGraphics = new Graphics_2D::Polygon(polygon, obstacleCellGraphics);// when the parent gets deleted, cellGraphics gets deleted...
        cellGraphics->setToolTip(Node::tr("Obstacle Cell").append(" ").append(QString::number(obstacleId)));
        cellGraphics->setBrush(Graphics_2D::Colors::randomColor());
        cellGraphics->setZValue(GraphicItemZLevels::ObstacleCellGraphicZLevel); 
        cellGraphics->setOpacity(0.2); //FIXME: Magic Numbers: Opacity of the obstacle cell graphics
        if( obstacleId != NULL_ELEMENT_ID )
        { cellGraphics->hide(); } 
//         qDebug() << "Area of" << cellGraphics->toolTip() <<":" << boost::geometry::area(polygon);

        obstacleDelaunayFaceGraphics[obstacleId] = cellGraphics;
      }
      
      obstacleCellGraphics->hide();
    }
    { // obstacleGraph graphics 
      bool visibility = false;
      
      if(obstacleGraphGraphics != nullptr)
      { visibility = obstacleGraphGraphics->isVisible(); }
      
      delete obstacleGraphGraphics;
      obstacleGraphGraphics = nullptr;

      obstacleGraphGraphics = Graphics_2D::generateQtGraphicsFromGraph(
        obstacleGraph, obstacleLocationMap
        ,/*edgePen*/ Graphics_2D::Pens::Outline(Graphics_2D::Colors::routeGraph)
        ,/*vertexPen*/ Graphics_2D::Pens::Outline(Graphics_2D::Colors::routeGraph)
        ,/*drawEdges*/ true
        ,/*drawVertices*/ true);
      obstacleGraphGraphics->setVisible(visibility);
      obstacleGraphGraphics->setZValue(NodePrivate::ObstacleGraphGraphicZLevel);
      obstacleGraphGraphics->setParentItem(datumItem);
      /** \todo Find an intelligent way to define the graphics of a graph. */
    }
#endif // WITH_GRAPHICS  
  }
   
  /** \internal Step 3: Associate route graph intersection vertices to obstacles
   * 
   * \todo Associate the obstacle graph edges with the corresponding dual edges
   * of the bifurcation graph (which then in turn allow access to the associated
   * intersection vertices of the bifurcation and route graphs).
   * 
   * This step requires access to:
   *  - visitor.faceGraph
   *  - faceVertex_to_obstacleVertex_map
   */
  {
  
  using FaceGraphEdge = typename boost::graph_traits<FaceGraph>::edge_descriptor;
  using FaceGraphVertex = typename boost::graph_traits<FaceGraph>::vertex_descriptor;
  
  //
  // Migrate the edgeToFaceMap from the visitor to an edgeIndexToObstacleIdMap
  // Recording the edge descriptors doesn't help as the edges will be eliminated 
  // in the bifurcation simplification step
  //
  // NOTE that the indices are retained through a nasty copy-paste HACK in the 
  // merging step of the Graph::removeDegree2Vertices method.  
  //
  
  using namespace boost;
  
  // lambda to translate a face graph vertex (i.e. a face) to an ObstacleId
  auto assocObstacleElementId = [&](FaceGraphVertex fVertex) -> ElementId
  {
    ObstacleGraph::Vertex_Descriptor obs = faceVertex_to_obstacleVertex_map[fVertex];
    ElementId id = get(vertex_id, obstacleGraph, obs);
    
    return id;
  };
  
  std::map<unsigned int, std::pair<ElementId, ElementId>> edgeIndexToObstacleIdMap;
  for( auto pair : visitor.edgeToFaceMap )
  {
    FaceGraphEdge edgeDescriptor = pair.first;
    std::pair<FaceGraphVertex,FaceGraphVertex> faceDescriptors = pair.second;
    
    // (vroniGraph) edge index of the currently processed face graph edge
    unsigned int eIndex = get(edge_index, vroniGraph, edgeDescriptor);
  
//     Q_ASSERT(faceDescriptors.first != faceDescriptors.second );
    
    ElementId id1 = assocObstacleElementId(faceDescriptors.first);
    ElementId id2 = assocObstacleElementId(faceDescriptors.second);

//     Q_ASSERT( id1!=id2 );
    
    edgeIndexToObstacleIdMap[eIndex] = std::make_pair(id1,id2);        
  }


//   qDebug() << "biEdgeLocationData.size() =" << biEdgeLocationData.size();
//   qDebug() << "num_edges(bifurcationGraph) =" << num_edges(bifurcationGraph);

  updateBifurcationGraph(); //NOTE: this reduces the graph to intersection-only vertices!

//   qDebug() << "biEdgeLocationData.size() =" << biEdgeLocationData.size();
//   qDebug() << "num_edges(bifurcationGraph) =" << num_edges(bifurcationGraph);
  

  
  /** \internal Repopulate the map correlating obstacles and the surrounding
   * bifurcation graph edges (which indirectly hold the intersection vertices
   * as their source and target nodes) */
  
  obstacleBifurcationEdgeData.clear();
  obstacleBifurcationVertexData.clear();
  { // populate obstacleBifurcationEdgeData
    BifurcationGraph::Edge_Iterator ei,ei_end;
    for( tie(ei, ei_end)=edges(bifurcationGraph);ei!=ei_end;++ei)
    {
      
      /** \note \e Important robustness note: 
      * This trick here with the edge indices only works because the 
      * bifurcationGraph has the same edge indices as the routeGraph which has 
      * the same edge indices as the vroniGraph (as the former is a copy from 
      * the later and so on [minus the non-cyclic branches (and no reindexing 
      * of the edgesafter the deletion)]). */
      
      auto eIndex = get(edge_index, bifurcationGraph, *ei); // this gets a bifurcationGraph edge index (which is a route graph index, retained in updateBifurcationGraph)
      auto obstacleId1 = edgeIndexToObstacleIdMap[eIndex].first; // this is based on vroniGraph indices (which are copied over into the routeGraph)
      auto obstacleId2 = edgeIndexToObstacleIdMap[eIndex].second; // this is based on vroniGraph indices (which are copied over into the routeGraph)
            
      obstacleBifurcationEdgeData[obstacleId1].insert(*ei);
      obstacleBifurcationEdgeData[obstacleId2].insert(*ei); 
      
      obstacleBifurcationVertexData[obstacleId1].insert(source(*ei, bifurcationGraph));
      obstacleBifurcationVertexData[obstacleId1].insert(target(*ei, bifurcationGraph));      
      obstacleBifurcationVertexData[obstacleId2].insert(source(*ei, bifurcationGraph));
      obstacleBifurcationVertexData[obstacleId2].insert(target(*ei, bifurcationGraph));
    }
  }
  
  obstacleRouteEdgeData.clear();
  { // poplulate obstacleRouteEdgeData
    RouteGraph::Edge_Iterator ei, ei_end;
    for( tie(ei, ei_end)=edges(routeGraph);ei!=ei_end;++ei)
    {
      
      /** \note \e Important robustness note: 
       * This trick here with the edge indices only works because the routeGraph
       * has the same edge indices as the vroniGraph (as the former is a copy from 
       * the later [minus the non-cyclic branches (and no reindexing of the edges
       * after the deletion)]). */
      
      auto eIndex = get(edge_index, routeGraph, *ei); // this gets a bifurcationGraph edge index (which is a route graph index, retained in updateBifurcationGraph)
      auto obstacleId1 = edgeIndexToObstacleIdMap[eIndex].first; // this is based on vroniGraph indices (which are copied over into the routeGraph)
      auto obstacleId2 = edgeIndexToObstacleIdMap[eIndex].second; // this is based on vroniGraph indices (which are copied over into the routeGraph)
            
      obstacleRouteEdgeData[obstacleId1].insert(*ei);
      obstacleRouteEdgeData[obstacleId2].insert(*ei); 
    }
  
  }
   
  /** \internal Repopulate the map correlating the obstacles and the surrounding
   * route graph vertices. */
  
  obstacleRouteVertexData.clear();
   
  if( true )
  { // populate obstacleRouteVertexData (via obstacleBifurcationEdgeData)
  
  BifurcationGraph::Vertex_LocationId_Map biVertexLocationId
    = get(vertex_locationId, bifurcationGraph);
    
  auto corresponding_routeGraph_vertex 
    = [&](BifurcationGraph::Vertex_Descriptor biVertex)
    -> RouteGraph::Vertex_Descriptor
  {
    Location::Id id = biVertexLocationId[biVertex];
    Location const& location = locationData[id]; 
    return location.routeGraphVertex();
  };
  
  for( auto& pair : obstacleBifurcationEdgeData )
  {
    auto& obstacleId = pair.first;
    auto& biEdgeSet = pair.second;
    
    // The set of vertices associated with the obstacle
    // NOTE: as obstacleRouteVertexData has been cleared outside this for-loop,
    // this set is initially empty.
    std::set<RouteGraph::Vertex_Descriptor>& rVertexSet 
      =  obstacleRouteVertexData[obstacleId];  

//     std::cout << "BiEdge association of " 
//       << obstacleData[obstacleId].objectName().toStdString() 
//       << " -> {";
      
    for( auto const& biEdge : biEdgeSet)
    {
//       std::cout << get(edge_index, bifurcationGraph, biEdge) <<", ";
      
      auto biVertexSource = boost::source(biEdge, bifurcationGraph);
      auto biVertexTarget = boost::target(biEdge, bifurcationGraph);
      
      auto rVertexSource = corresponding_routeGraph_vertex(biVertexSource);
      auto rVertexTarget = corresponding_routeGraph_vertex(biVertexTarget);
      
      //
      // add source and target vertices of the bifurcation edge
      //  
      
      rVertexSet.insert(rVertexSource); 
      rVertexSet.insert(rVertexTarget); 
      
      //
      // add the location(s) that the bifurcation edge also contains, i.e. all 
      // the route graph vertex locations that were stripped away when the 
      // bifurcation edge was created.
      //
      
      if( biEdgeLocationData.count(biEdge) != 0 )
      { 
        auto const& locationIdSet = biEdgeLocationData[biEdge];
//         std::cout << "BiEdge Locations ("<< locationIdSet.size() <<"): \n";
        for( auto const& id : locationIdSet )
        { 
          Location const& location = locationData[id];
//           std::cout << id << ", " << location << "\n";
          if(location.isRouteGraphVertexValid() == true )
          { rVertexSet.insert(location.routeGraphVertex()); }
        }   
//         std::cout << std::endl;
      }
    }
    
//     std::cout << "}" << std::endl;
    
//     qDebug() << obstacleData[obstacleId].objectName() << "has these intersection vertices:";
//     for( auto v : rVertexSet )
//     { qDebug() << get(vertex_index, bifurcationGraph, v); }
  }  
  
  
  }
  
//   if(false)
//   { // populate obstacleRouteVertexData (via obstacleRouteEdgeData)
//     for( auto& pair : obstacleRouteEdgeData )
//     {
//       auto& obstacleId = pair.first;
//       auto& rEdgeSet = pair.second;
//       
//       std::set<RouteGraph::Vertex_Descriptor>& rVertexSet 
//         =  obstacleRouteVertexData[obstacleId];  
// 
//       //
//       // add the source and target vertices of the edge. Double entries don't 
//       // matter as the std::set doesn't allow duplicates and discards them.  
//       //
//         
//       for( auto const& rEdge : rEdgeSet)
//       {
//         auto rVertexSource = boost::source(rEdge, routeGraph);
//         auto rVertexTarget = boost::target(rEdge, routeGraph);
//       
//         rVertexSet.insert(rVertexSource); 
//         rVertexSet.insert(rVertexTarget); 
//       }
//     }
//   }
  
#if defined WITH_GRAPHICS && defined DEBUG_GRAPHICS
 if(true){
    auto locationIdMap = get(vertex_locationId, routeGraph);
    auto locationMap = make_compose_property_map(location_pMap, locationIdMap);
    
    delete obstacleBiEdgeAssocGraphics;
    obstacleBiEdgeAssocGraphics = new QGraphicsItemGroup(datumItem);
    
    // populate obstacleBiEdgeAssocGraphics
    for( auto pair : obstacleBifurcationEdgeData )
    {
      /** \internal There was something funny here: when obs was only a Obstacle,
       * not a const reference to an Obstacle, code outside and _after_ this
       * for loop would not get executed... Odd. */
      
      Obstacle const& obs = obstacleData[pair.first]; 
      std::set<BifurcationGraph::Edge_Descriptor> edgeSet = pair.second;
      
      Geometry::Point_2D obsLoc = obs.centroid();
      
      qreal x1 = obsLoc.x();
      if( qIsInf(x1) or qIsNaN(x1) ) 
      { continue;}

      qreal y1 = obsLoc.y();
      if( qIsInf(y1) or qIsNaN(y1) ) 
      { continue;}
      
      for( auto edge : edgeSet )
      {
        Geometry::Point_2D midpoint = Graph::edgeMidpoint(edge, bifurcationGraph, locationMap);
        
        qreal x2 = midpoint.x();
        if( qIsInf(x2) or qIsNaN(x2) )
        { continue;}

        qreal y2 = midpoint.y();
        if( qIsInf(y2) or qIsNaN(y2) )
        { continue;}
        
        auto line = new QGraphicsLineItem( x1,y1,x2,y2,obstacleBiEdgeAssocGraphics );
        line->setPen(Graphics_2D::Pens::Outline(Qt::magenta));
      }
    }
    
    obstacleBiEdgeAssocGraphics->setZValue(GraphicItemZLevels::ObstacleEdgeAssociationGraphicZLevel);
    obstacleBiEdgeAssocGraphics->hide();
    
    
    
    delete obstacleRouteVertexAssocGraphics;
    obstacleRouteVertexAssocGraphics = new QGraphicsItemGroup(datumItem);
    
    // populate obstacleRouteVertexAssocGraphics
    for( auto& pair : obstacleRouteVertexData )
    {
      auto const obstacleId = pair.first;
      auto const intersectionVertexSet = pair.second;
      
      if( obstacleId == motionBoundaryElementId )
      { continue; }
      
      Obstacle const& obstacle = obstacleData[obstacleId];
      
//       qDebug() <<"Drawing vertex associations for:" << obstacle;
      
//       ObstacleGraph::Vertex_Descriptor obstacleVertex = obstacle.obstacleGraphVertex();
//       Location obsLoc = locationMap[obstacleVertex];
      
      Geometry::Point_2D obsLoc = obstacle.centroid();
       
      qreal x1 = obsLoc.x();
      if( qIsInf(x1) or qIsNaN(x1) ) 
      { continue;}

      qreal y1 = obsLoc.y();
      if( qIsInf(y1) or qIsNaN(y1) ) 
      { continue;}
      
      for( auto const vertex : intersectionVertexSet )
      {   
        Location vPos = locationMap[vertex]; 

        qreal x2 = vPos.x();
        if( qIsInf(x2) or qIsNaN(x2) )
        { continue;}

        qreal y2 = vPos.y();
        if( qIsInf(y2) or qIsNaN(y2) )
        { continue;}

        auto line = new QGraphicsLineItem(x1,y1,x2,y2,obstacleRouteVertexAssocGraphics);
        line->setPen(Graphics_2D::Pens::Outline(Qt::black));
      }
    }
    
    obstacleRouteVertexAssocGraphics->setZValue(GraphicItemZLevels::ObstacleEdgeAssociationGraphicZLevel);
    obstacleRouteVertexAssocGraphics->hide();
}
#endif // WITH_GRAPHICS&& DEBUG_GRAPHICS

  }
}



void NodePrivate::updateBifurcationGraph()
{       

  using namespace boost;

  bifurcationGraph.clear();
  
  copy_graph(routeGraph, bifurcationGraph);
    
  {
//     qDebug() << "ROUTE GRAPH =================================";
//     auto rLocationIdMap = get(vertex_locationId, routeGraph);
//     auto rLocationMap = make_compose_property_map(location_pMap, rLocationIdMap);
//     print_graph(routeGraph, rLocationMap);
//     
//     qDebug() << "LOCAL BI GRAPH ==============================";
//     auto biLocationIdMap = get(vertex_locationId, bifurcationGraph);
//     auto biLocationMap = make_compose_property_map(location_pMap, biLocationIdMap);
//     print_graph(routeGraph, biLocationMap);
  }
  
  biEdgeLocationData.clear();
  

  { // remove the degree-2 vertices from the freshly created bifurcation graph
    
    using namespace boost;
    using Graph=BifurcationGraph::Graph;
    auto& graph=bifurcationGraph;
    
    BOOST_CONCEPT_ASSERT(( MutableGraphConcept< Graph > ));
    
    //
    // iterate over all vertices in the graph
    // 

    
    //NOTE: this setup, using a 'next' iterator, is necesarry in order to 
    // maintain iterator stability when deleting vertices.
    typename graph_traits<Graph>::vertex_iterator vi, vi_end, next;
    tie(vi,vi_end)=vertices(graph);
    for( next=vi; vi!=vi_end; vi=next )
    {
      ++next;
      
      //
      // if it's a degree 2 vertex, erase it and merge the two corresponding 
      // edges into a new one.
      //
       
      if( out_degree(*vi,graph) == 2 )
      { 
        typename graph_traits<Graph>::adjacency_iterator ai,ai_end;
        tie(ai,ai_end) = adjacent_vertices(*vi,graph);
        
        //NOTE: add_edge(*ai,*(++ai),g); DOES NOT WORK!!! add_edge() invalidates adjacency iterators...
        
        typename graph_traits<Graph>::vertex_descriptor vBefore, vAfter;
        vBefore = *ai;
        vAfter = *(++ai);
        
        typename graph_traits<Graph>::edge_descriptor eBefore, eAfter, eMerged;
        eMerged = add_edge(vBefore, vAfter, graph).first;
        
        {// getting the (two) outedges
          typename graph_traits<Graph>::out_edge_iterator ei,ei_end;
          tie(ei,ei_end) = out_edges(*vi,graph);
          eBefore = *ei;
          eAfter = *(++ei);
        }
          
        // doing the merging..          
        put(edge_index,   graph, eMerged, get(edge_index, graph, eBefore)); //HACK: This is done so that the edge index to face index map still contains valid data.
        put(edge_weight,  graph, eMerged, get(edge_weight, graph, eBefore)); //HACK: This is done to keep an accumulated edge weight(length) for path graphs.  
        
        // the set of Locations the merged edge represents / replaces
        std::set<LocationId>& locations = biEdgeLocationData[eMerged];  
        
        // the location of the vertex that is erased in this round
        LocationId vLocationId = get(vertex_locationId, graph, *vi);
        locations.insert(vLocationId);
        
        // and the locations already in the 'before' and 'after' edges
        auto locBefore = biEdgeLocationData[eBefore];
        auto locAfter = biEdgeLocationData[eAfter];
        locations.insert(locBefore.begin(),locBefore.end());
        locations.insert(locAfter.begin(),locAfter.end());

        // remove the stored location sets in the mapping
        biEdgeLocationData.erase(eBefore);
        biEdgeLocationData.erase(eAfter);
        
        clear_vertex(*vi,graph); // remove any edges
        remove_vertex(*vi,graph); // remove the vertex
      }
    }
    
    
  }
  
  
  
  BifurcationGraph:: Vertex_LocationId_Map locationIdMap 
    = get(vertex_locationId, bifurcationGraph);
  auto locationMap = make_compose_property_map(location_pMap, locationIdMap);
    
  BifurcationGraph::Vertex_Iterator vi, vi_end;
  for( tie( vi, vi_end)=vertices(bifurcationGraph); vi!=vi_end; ++vi)
  {
    Location& location = locationMap[*vi];
    location.setBifurcationGraphVertex(*vi);
  }

#if defined WITH_GRAPHICS && defined DEBUG_GRAPHICS 
{
  updateBifurcationGraphGraphics(); 
  
  delete biEdgeAssociationGraphics;
  biEdgeAssociationGraphics = new QGraphicsItemGroup();
  
  for( auto pair : biEdgeLocationData )
  {
    auto biEdge = pair.first;
    auto locationSet = pair.second;
    
    Geometry::Point_2D edgeMidpoint 
      = Graph::edgeMidpoint(biEdge, bifurcationGraph, locationMap);
    
    
//       std::cout << "BiEdge[" << get(edge_index, bifurcationGraph, biEdge) <<"] -> {";
    for( auto locationId : locationSet )
    { 
      Location vertexLocation = locationData[locationId];
      
//         std::cout << locationId << ", "; 
      auto line = new QGraphicsLineItem(edgeMidpoint.x(), edgeMidpoint.y(),
                            vertexLocation.x(), vertexLocation.y(),
                            biEdgeAssociationGraphics );
      line->setPen(Graphics_2D::Pens::Outline(Qt::blue));
    }
//       std::cout <<"}"<<std::endl;
  }
  
  biEdgeAssociationGraphics->setParentItem(datumItem);
  biEdgeAssociationGraphics->setZValue(GraphicItemZLevels::BifurcationEdgeAssociationGraphicZLevel);
  biEdgeAssociationGraphics->hide(); 
}
#endif // WITH_GRAPHICS && DEBUG_GRAPHICS
    
}
    
    
#if defined WITH_GRAPHICS && defined DEBUG_GRAPHICS  
void NodePrivate::updateBifurcationGraphGraphics()
{
  bool visibility = false;
  if(bifurcationGraphGraphics != nullptr)
  { visibility = bifurcationGraphGraphics->isVisible(); }
    
  delete bifurcationGraphGraphics;
  bifurcationGraphGraphics = nullptr;
  
    
  using namespace boost;
  
  auto locationIdMap = get(vertex_locationId, bifurcationGraph);
  auto locationMap = make_compose_property_map(location_pMap, locationIdMap);
  
  bifurcationGraphGraphics = Graphics_2D::generateQtGraphicsFromGraph(
    bifurcationGraph, locationMap
    ,/*edgePen*/ Graphics_2D::Pens::Outline(Qt::cyan)
    ,/*vertexPen*/ Graphics_2D::Pens::Outline(Qt::magenta)
    ,/*drawEdges*/ true
    ,/*drawVertices*/ true
    );
  bifurcationGraphGraphics->setVisible(visibility);
  bifurcationGraphGraphics->setParentItem(datumItem);
  bifurcationGraphGraphics->setZValue(NodePrivate::BifurcationGraphGraphicZLevel);
}   
#endif // WITH_GRAPHICS && DEBUG_GRAPHICS

  
// void NodePrivate::createTargetEnvironment(ElementId const& targetId){ return; }
void NodePrivate::createTargetEnvironment(ElementId const& targetId)
{
  Q_ASSERT(targetId != NULL_ELEMENT_ID);
  Q_Q(Node); Q_ASSERT(q);
  
  // the targeted obstacle element.
  Obstacle const& target = q->obstacle(targetId);
  
  ObstacleGraph::Vertex_Descriptor targetObsVertex =  target.obstacleGraphVertex();
  
  std::list<ObstacleGraph::Vertex_Descriptor> targetEnvironemntVertices;
  targetEnvironemntVertices.push_back(targetObsVertex);
  
  { // add the neighboring obstacles
   std::set<ObstacleGraph::Vertex_Descriptor> neighbors = Graph::boundaryVertices<>(obstacleGraph, targetEnvironemntVertices);
   targetEnvironemntVertices.insert( targetEnvironemntVertices.end()
    ,neighbors.begin(), neighbors.end());
  }
  
//   
//   qDebug() << q->qualifiedName() << ": Creating local target environment around" 
//     << target.objectName();
//   
  
  std::size_t holeCounter = 0;
  static const double areaThreshold = 0.1; //FIXME: Magic Numbers: Threshold to discard polygon artefacts
  
  // a (local) representation of the target environment. This is used to build 
  // up the polygon before it is moved to the Node-level  
  Geometry::Polygon_2D localTargetEnvironment;

  
  for( auto obsVertex : targetEnvironemntVertices )
  {
    using namespace boost;
    
    std::vector<Geometry::Polygon_2D> combination;
    geometry::union_( localTargetEnvironment
                    , obstacleDelaunayFaceData[ get(vertex_id, obstacleGraph, obsVertex) ]
                    , combination);
    
    if( combination.size() == 1 )
    {  geometry::assign(localTargetEnvironment, combination[0] ); }
    else
    { 
        qDebug() << q->qualifiedName() << ": Detected issues during polygon combination while creating the target environment.";
      
      // something went wrong during the creation of the union. Let's check if
      // some of the parts could be numerical artefacts (really small area) and 
      // if yes, discard them. 
      // If only one is left, assign that to the localTargetEnvironemnt. If that
      // is not the case, keep track of the troubling elements and
      // add them later (in "round two") when the shape has changed (which might
      // affect things).
      

      // a container holding the polygons from combination that have an area
      // larger than areaThreshold
      std::vector<Geometry::Polygon_2D> validCombinationPolygons;
      
      for( auto& polygon : combination )
      {
        if( boost::geometry::area( polygon ) >= areaThreshold )
        { validCombinationPolygons.push_back(polygon); }
      }
      
      if( validCombinationPolygons.size() == 1 )
      { 
        geometry::assign(localTargetEnvironment, validCombinationPolygons[0]);
//           qDebug() << q->qualifiedName() << ": ... which could be fixed.";
        break;
      }
      
      qWarning() << q->qualifiedName() << ": Cannot combine obstacle cells correctly."
        << "(combination.size() ==" << combination.size() << ")";     
      
      int i=0;  
      for( auto& p : combination )
      {
        qWarning() << q->qualifiedName() << ": Area of combination[" << i++ << "] = " 
          << boost::geometry::area(p) ; 
      }
      
      
      
      
    }
    
    if( localTargetEnvironment.inners().size() >  holeCounter)
    {
//         qDebug() << q->qualifiedName() << ": Created a hole (inner polygon) in the local target environment.";
//         qDebug() << q->qualifiedName() << ": area(inners[" << holeCounter <<"]) =" 
//           << boost::geometry::area(localTargetEnvironment.inners().at(holeCounter));
      ++holeCounter;
    }
  }
  
  // clear any holes that are smaller than the area threshold as those are 
  // numerical artefacts and correct zero-area spikes
  {
    boost::geometry::correct(localTargetEnvironment);
    
    for( auto i = localTargetEnvironment.inners().end() //NOTE: this starts "one past the last element"
       ; i != localTargetEnvironment.inners().begin()
       ; /* no decrementing action here, it's done in the body of the loop*/ )
    { 
      // Decrement the itereator here _before_ doing anythign as we are moving
      // backwards without using reverse iterators.
      --i;
      
      if( std::abs(boost::geometry::area(*i)) <= areaThreshold ) //NOTE: holes of (corrected) polygons have a negative area.
      { localTargetEnvironment.inners().erase(i); } //WARNING: This should/could/will destroy iterators... but we are moving backward... so we are good???
    }
    
    localTargetEnvironment.outer() = 
      Geometry::getBoostEqualSimplified(localTargetEnvironment.outer(),areaThreshold);
  }

  { // add holes for the obstacles
    for( auto obsVertex : targetEnvironemntVertices )
    {
      Obstacle const& obstacle = obstacleData.at(boost::get(boost::vertex_id, obstacleGraph, obsVertex)); //NOTE: this is needed for const correctnes
      localTargetEnvironment.inners().push_back( obstacle.shape().outer() );
    }
  }
  
  
#ifdef WITH_GRAPHICS
  if( targetEnvironmentGraphics == nullptr )
  { 
    targetEnvironmentGraphics = new Graphics_2D::Polygon(targetEnvironment);
    targetEnvironmentGraphics->setZValue(GraphicItemZLevels::TargetEnvironmentGraphicZLevel);
    targetEnvironmentGraphics->setParentItem(datumItem);
  }

  targetEnvironmentGraphics->setGeometry(localTargetEnvironment);
#else // WITH_GRAPHICS
  // "export" the localTargetEnvironemnt to NodePrivate::targetEnvironment
  boost::geometry::assign(targetEnvironment, localTargetEnvironment);  
#endif // WITH_GRAPHICS

  isTargetEnvironmentValid = true;
}

void NodePrivate::unsetTargetEnvironment()
{
  isTargetEnvironmentValid = false;
  isFormationGraphValid = false;
  
#ifdef WITH_GRAPHICS
{
  delete targetEnvironmentGraphics;
  targetEnvironmentGraphics = nullptr;
}
#endif // WITH_GRAPHICS
}


void NodePrivate::createComCorridor(ElementId const& targetId, NodeId const& source)
{
  unsetComCorridor();
#ifdef WITH_GRAPHICS
  if(comCorridorGraphics == nullptr) { 
   comCorridorGraphics = new Graphics_2D::Polygon(comCorridor,datumItem); 
   comCorridorGraphics->setZValue(GraphicItemZLevels::ComCorridorGraphicZLevel);
  }
#endif // WITH_GRAPHICS

  /** \todo non-convex motion boundary creates COM issue
   * 
   * There is a problem with non-convex motion boundaries: the R/F polygons 
   * in general (i.e. when computing the COM graph) are bound by the motion 
   * boundary as that is currently the outer ring of the environment2D 
   * polygon. The formation graph, however, uses the the COM corridor as the 
   * outer ring. This can lead to odd cases where connectivity that seems 
   * possible during the formation graph computation suddenly is infeasible 
   * during the realization phase of the formation. (so happening in with the
   * shape of McKenna _before_ this 'correction').
   *
   * A problem there then is the different usages of the environment for 
   * motion (motionboundary results in voronoi paths) and for r/f (convex hull
   * allows for free LOS 'inside' the motion boudary).
   *
   * A possible approach would be to use the convex hull of the motion 
   * boundary as the outer ring for the environment (and then do some 
   * trickery to recreate the graphical effect of only having lines on the 
   * outside of it. Maybe with a second item?
   * 
   * However, there is a conceptual question as to this replicates "reality" 
   * as stuff outside the motion boundary might indeed be more obstacles 
   * which could block R/F.... In which case the COM corridor should be
   * intersected with the environment first, eliminating the case where 
   * the formation graph can generate R/F connections where the COM graph 
   * cannot... :/ */

  
  computeSlotBasedComCorridor(targetId,source);
//   computeCellBasedComCorridor(targetId,source);
  
  isComCorridorValid = true;
  
#ifdef WITH_GRAPHICS
  {
  Q_Q( Node ); Q_ASSERT(q);
  q->showComCorridor();
  }
#endif // WITH_GRAPHICS
  
};
  
  
  

void NodePrivate::computeCellBasedComCorridor(ElementId const& targetId, NodeId const& source)
{
  Q_ASSERT( targetId != NULL_ELEMENT_ID );
  Q_Q( Node ); Q_ASSERT(q); 
    

  
  /** \internal Step 1: Get the (obstacle) cells between the current GCS 
   * position and the target */
  
  ObstacleGraph::Vertex_Descriptor targetCellVertex 
    = obstacleData[targetId].obstacleGraphVertex();
    
  ObstacleGraph::Vertex_Descriptor sourceCellVertex 
    = obstacleData[ blipData[source]->closestObstacleId() ].obstacleGraphVertex();
    
    
  // (ordered) set of obstacles/vertices/cells that connect the source to the 
  // target cell with a minimal hop count.
  // NOTE: This path does not have to be unique. Other paths with the same hop 
  // count can exist!
  std::vector<ObstacleGraph::Vertex_Descriptor> coreCorridorVertices;
  
  auto elementIdMap = boost::get(boost::vertex_id, obstacleGraph);
  
  { // breadth first search through the obstacle graph    
    using namespace boost;    
    using Vertex = ObstacleGraph::Vertex_Descriptor;
    
    std::map<Vertex,Vertex> predecessors;
    predecessors[targetCellVertex] = targetCellVertex;  
    
    associative_property_map< std::map<Vertex,Vertex> > predecessor_map(predecessors);
    breadth_first_search(obstacleGraph, targetCellVertex
      ,visitor(make_bfs_visitor(record_predecessors(predecessor_map, on_tree_edge())))
    );
    
    for( Vertex v = sourceCellVertex; /* empty */; v = predecessor_map[v] )
    {
//       qDebug() << "Adding " << obstacleData[elementIdMap[v]].objectName() 
//         << "(next up:" << obstacleData[elementIdMap[predecessor_map[v]]].objectName() <<")";
      coreCorridorVertices.push_back(v);
      
#if defined WITH_GRAPHICS && defined DEBUG_GRAPHICS
if(false){ 
  // highlight the core COM corridor obstacle cells
      QAbstractGraphicsShapeItem* cell = obstacleDelaunayFaceGraphics[elementIdMap[v]];
      cell->setBrush(Qt::darkYellow);
      cell->show();
}
#endif // WITH_GRAPHICS&& DEBUG_GRAPHICS

      if( v == predecessor_map[v]) 
      { break; }
    }
  }
  
 
  Geometry::Polygon_2D localComCorridor;
  // add the first cell...
  boost::geometry::assign(localComCorridor, obstacleDelaunayFaceData[boost::get(boost::vertex_id, obstacleGraph, coreCorridorVertices[0])]);
  // ... and start combining it with the other core cells, starting at the second.
  for( auto oi = ++coreCorridorVertices.begin(); oi!= coreCorridorVertices.end(); ++oi )
  {
    ElementId elementId = boost::get(boost::vertex_id, obstacleGraph, *oi);
    std::vector<Geometry::Polygon_2D> combination;
   
    // This should work without issues as the coreCorridorVertices are sorted
    // by their traversal, i.e. the union of two associated cells shouldn't 
    // result in non-simple polygons as they are always neighboring.
    boost::geometry::union_( localComCorridor
                    , obstacleDelaunayFaceData[ elementId ]
                    , combination);
    if( combination.size() == 1 )
    {  boost::geometry::assign(localComCorridor, combination[0] ); }
    else
    { qCritical() << q->qualifiedName() << ": Can't add a part of the core COM corridor."; }
  }

  std::set<ObstacleGraph::Vertex_Descriptor> comCorridorPaddingVertices;
  comCorridorPaddingVertices = Graph::boundaryVertices(obstacleGraph, coreCorridorVertices);
  
  for( auto oi = comCorridorPaddingVertices.begin(); oi!=comCorridorPaddingVertices.end(); ++oi )
  { 
    ElementId elementId = boost::get(boost::vertex_id, obstacleGraph, *oi);
    
    std::vector<Geometry::Polygon_2D> combination;
    
    // This cannot be done without some prior work on the core as the padding
    // cells are not sorted. Hence, if they were just unified without the core,
    // it cannot be guaranteed that the union remains a simple polygon.
    boost::geometry::union_( localComCorridor
                    , obstacleDelaunayFaceData[ elementId ]
                    , combination);
    if( combination.size() == 1 )
    {  boost::geometry::assign(localComCorridor, combination[0] ); }
    else
    { qCritical() << q->qualifiedName() << ": Can't add padding to the COM corridor."; }
    
#ifdef WITH_GRAPHICS    
    {// highlight the padded COM corridor obstacle cells
      qDebug() << "Adding padding:" << obstacleData[elementIdMap[*oi]].objectName() ;
      QAbstractGraphicsShapeItem* cell = obstacleDelaunayFaceGraphics[elementId];
      cell->setBrush(Qt::yellow);
      cell->show();
    }
#endif // WITH_GRAPHICS
  }
  
  // remove the obstacles -> add them as holes to the corridor polygon
  for( auto vertex : coreCorridorVertices )
  {
    Obstacle const& obstacle = obstacleData[elementIdMap[vertex]];
    localComCorridor.inners().push_back( obstacle.shape().outer() );
  }
  for( auto vertex : comCorridorPaddingVertices )
  {
    Obstacle const& obstacle = obstacleData[elementIdMap[vertex]];
    localComCorridor.inners().push_back( obstacle.shape().outer() );
  }
  boost::geometry::correct(localComCorridor);  
  

#ifdef WITH_GRAPHICS
  comCorridorGraphics->setGeometry(localComCorridor);
//   q->showObstacleCells();
#else // WITH_GRAPHICS
  boost::geometry::assign(comCorridor, localComCorridor);
#endif // WITH_GRAPHICS
}

void NodePrivate::computeSlotBasedComCorridor(const ElementId& targetId, const NodeId& source)
{
  Q_ASSERT( blipExists(source) );
  Q_ASSERT( targetId!=NULL_ELEMENT_ID );
  Q_ASSERT( obstacleData.count(targetId)==1 );
  
  Geometry::Polygon_2D corridor;
  
  {
    Obstacle const& target = obstacleData[targetId];
    
    Geometry::Point_2D p1 = blipData[ source ]->position2D();
    Geometry::Point_2D p2 = target.centroid();
    double width = 2* RfZoneLimits::MaxRfRange;
    
//     auto ring = Geometry::makeFlatSlot<Geometry::Polygon_2D::ring_type>(p1, p2, width);
//     auto ring = Geometry::makeSquareSlot<Geometry::Polygon_2D::ring_type>(p1, p2, width);
    auto ring = Geometry::makeCircularSlot<Geometry::Polygon_2D::ring_type,20>(p1, p2, width);
    
    boost::geometry::assign(corridor.outer(), ring);
  }


//   auto const obstacleSet = getObstaclesWithinArea(ring); // not yet implemented :(

  for( auto& pair: obstacleData )
  {
    
    // skip the outer cell stuff.
    if( pair.first == NULL_ELEMENT_ID )
    { continue; }

    using namespace boost::geometry;
    
    Obstacle& obstacle = pair.second;
    
//     {// debug
//       qDebug() << "Processing obstacle" << pair.first << "," << obstacle.objectName();
//       obstacle.markAsTarget();
//       comCorridorGraphics->setGeometry(comCorridor);
//       scene->update();
//       qApp->processEvents();
//     }
    

//       qDebug() << "Corridor area before :" /*<< area(corridor)*/;
      
      std::list<Geometry::Polygon_2D> output;
      difference(corridor,obstacle.shape(), output );
    
      
      if( output.size() == 1 )
      { 
//         { // debug
//           delete targetEnvironmentGraphics;
//           targetEnvironmentGraphics = new Graphics_2D::Polygon(targetEnvironment,datumItem);
//           targetEnvironmentGraphics->setZValue(GraphicItemZLevels::TargetEnvironmentGraphicZLevel);
//           targetEnvironmentGraphics->setGeometry(output.front());
//           targetEnvironmentGraphics->setBrush(Qt::darkCyan);
//           scene->update();
//           qApp->processEvents();
//         }
        assign(corridor,output.front()); 
//         qDebug() << "Output area after :" /*<< area(corridor)*/;
      }
      else
      { 
        Q_Q( Node ); Q_ASSERT(q);
        qDebug() << q->qualifiedName() << ": Detected a non-simple polygon when creating the COM corridor."
          << "(Polygon count:" << output.size() << ", Obstacle:" << obstacle.objectName()<< ")";       
        
        for( auto polygon : output )
        {
         if( covered_by(ownshipBlip->position2D(), polygon) )
         {
           assign( corridor, polygon);
           break;
         };
        }
          
        qCritical() << q->qualifiedName() << ": Could not recover a simple polygon when creating the COM corridor.";  
      }   
  }
  
  boost::geometry::correct(corridor);
  
#ifdef WITH_GRAPHICS
  comCorridorGraphics->setGeometry(corridor);
#else // WITH_GRAPHICS
  boost::geometry::assign(comCorridor, corridor);
#endif // WITH_GRAPHICS
}

void NodePrivate::unsetComCorridor()
{
  Q_Q( Node ); Q_ASSERT(q);
  isComCorridorValid = false;
#ifdef WITH_GRAPHICS
  q->hideComCorridor();
  delete comCorridorGraphics;
  comCorridorGraphics = nullptr;
#endif // WITH_GRAPHICS
  return;
}

/** \note This function does not work (yet) due to some missing boost 
 *implementations. <em> It will always return an empty set!</em> */
std::set<ElementId>  NodePrivate::getObstacleCellsWithinArea(Geometry::Polygon_2D::ring_type const& outline)
{
  std::set<ElementId> cellSet;
  for( auto& pair : obstacleDelaunayFaceData )
  {
    Geometry::Polygon_2D& polygon = pair.second;
    if( false /*not yet implemented :(  boost::geometry::overlaps(polygon.outer(), outline) */)
    { cellSet.insert(pair.first); }
  }
  
  return cellSet;
};

/** \note This function does not work (yet) due to some missing boost 
 *implementations. <em> It will always return an empty set!</em> */
std::set<ElementId> NodePrivate::getObstaclesWithinArea(Geometry::Polygon_2D::ring_type const& outline)
{
  // get the obstacle cells within the outline
  
  auto cellSet = getObstacleCellsWithinArea(outline);
  
  // check the corresponding obstacles for inclusion/coverate by the outline
  
  std::set<ElementId> obstacleSet;
  for( auto& id : cellSet )
  {
    Obstacle const& obstacle = obstacleData[id];
    if( false /*not yet implemented :(  boost::geometry::overlaps(obstacle.shape().outer(), outline) */)
    { obstacleSet.insert(id); }
  }
  
  return obstacleSet;
};

Geometry::Polygon_2D::ring_type NodePrivate::deltaDisc(
  const Geometry::Point_2D& center,
  const boost::geometry::coordinate_type<Geometry::Polygon_2D>::type& range
) const
{
  return Geometry::makeCircle<Geometry::Polygon_2D::ring_type,100>(center,range); //FIXME: Magic Numbers
}

Geometry::Polygon_2D::ring_type NodePrivate::deltaPie( Geometry::Point_2D const & center
  ,boost::geometry::coordinate_type<Geometry::Polygon_2D>::type const& range
  ,double const & direction, double const & openingAngle
) const
{
 return Geometry::makePie<Geometry::Polygon_2D::ring_type,100,boost::geometry::degree>
          (center, range, direction, openingAngle); 
}

Geometry::Polygon_2D NodePrivate::subEnvironment(
    const Geometry::Point_2D& location, const Geometry::Polygon_2D::ring_type& stencil) const
{
  Q_ASSERT(!boost::geometry::intersects(stencil));
//   Q_ASSERT(!boost::geometry::intersects(environment2D));
  

  std::deque<Geometry::Polygon_2D> intersectionPoly;
  boost::geometry::intersection(environment2D,stencil,intersectionPoly);

  
  Geometry::Polygon_2D returnPoly;
  
  size_t polygonCount =  intersectionPoly.size();
  if( polygonCount == 0 )
  {
    Q_Q(const Node); Q_ASSERT(q);   
    qDebug() << q->qualifiedName() << ": Cannot compute a subEnvironment. Does the stencil cover the environment?";
    qWarning() << q->qualifiedName() <<": Setting subEnvironment to a triangle.";
    boost::geometry::assign(returnPoly,Geometry::makeCircle<Geometry::Polygon_2D::ring_type,50>(location,3)); //FIXME: Magic Numbers
  }
  else if( polygonCount == 1)
  {
    returnPoly = intersectionPoly.front();    

    if( !boost::geometry::covered_by(location,returnPoly ) )
    {
      Q_Q(const Node); Q_ASSERT(q);   
      qDebug() << q->qualifiedName() << ": Cannot compute a subEnvironment that contains the location. Is the location in the environment?";
      qWarning() << q->qualifiedName() <<": Setting subEnvironment to a square.";
      boost::geometry::assign(returnPoly,Geometry::makeCircle<Geometry::Polygon_2D::ring_type,50>(location,4)); //FIXME: Magic Numbers
    };
    
    
  }
  else /*if( polygonCount > 1)*/
  {
    for( auto poly : intersectionPoly )
    {
      if( boost::geometry::covered_by(location,poly ) )
      {  
        returnPoly = poly;
        break;
      }
      polygonCount--;
    }
    
    if(polygonCount == 0)   
    {
      Q_Q(const Node); Q_ASSERT(q);
      qDebug() << q->qualifiedName() << ": Cannot determine the subEnvironment that contains the location from a disonnected set."
        <<"(Number of disconnected polygons:" << intersectionPoly.size() << ")";
      qWarning() << q->qualifiedName() <<": Setting subEnvironment to a circle.";
      boost::geometry::assign(returnPoly,Geometry::makeCircle<Geometry::Polygon_2D::ring_type,50>(location,10)); //FIXME: Magic Numbers
    }
  }
  return returnPoly; 
}


bool NodePrivate::computeRfPolygon( Geometry::Point_2D const & position, 
                                    Geometry::Polygon_2D::ring_type & rfRing )
{
  constexpr double rfRange = RfZoneLimits::MaxRfRange;  ///< \brief R/F range in [m].
 
  /**
   * \internal There is a major trade of in this function:
   *  On the one hand, compute the visbility polygon in the smaller deltaPolygon
   *  environment (at the price of having to not only compute this, but also
   *  recreate the VisiLibity environment),
   *  on the other hand, compute the visibility polygon in the complete arena
   *  (i.e. save the recreating at the cost of a more complex arena) and then
   *  do the range clipping later.
   */

  /**  compute the R/F polygon from a \Delta-polygon */
    
    //NOTE: although it says "computeVisibilityPolygon", due to the fact the utilized
    // environment has been clipped by the R/F range deltaPolygon, this effectively
    // computes the rfPolygon!
  auto ring =  VisilibityInterface::computeVisibilityPolygon<Geometry::Polygon_2D::ring_type>
                (position,deltaPolygon(position,rfRange));
 
                
  //
  // some data integrity checks
  //
  auto num_points = boost::geometry::num_points(ring);
  if ( num_points > 3 )
  { 
    Q_ASSERT( boost::geometry::covered_by(position,ring) );
    boost::geometry::assign(rfRing,ring);
    return true;    
  }
  else
  { 
    Q_Q(Node); Q_ASSERT(q);
    qCritical("\"%s\" : There is a problem computing an R/F polygon at (%f,%f) as is only has %d points."
      ,q->qualifiedName().toStdString().c_str()
      ,position.x(),position.y()
      ,static_cast<int>(num_points));
    return false;
  }
}


bool NodePrivate::updateRfPoly(Blip* const blip, double const & minDistance)
{
  Q_ASSERT(blip);
  
  // check whether the node has moved more than the required minDistance 
  Geometry::Point_2D position2D = blip->position2D();
  Geometry::Point_2D rfCenter2D = blip->rfPolygonCenter();
  
  if( boost::geometry::distance( position2D,rfCenter2D ) < minDistance )
  { return false; }
  
//   Q_Q(Node); Q_ASSERT(q);
//   qDebug() << q->qualifiedName() << ": Updateing R/F polygon for" << blip->name();
  
  Geometry::Polygon_2D::ring_type rfRing;
  
  if( computeRfPolygon(position2D,rfRing) )
  {
    blip->setRfPolygon(rfRing);
    blip->setRfPolygonCenter(position2D);
    return true;
  }
  else
  {
    Q_Q(Node); Q_ASSERT(q);
    qCritical() << q->qualifiedName() << ": Couldn't update the R/F polygon for"
      << blip->name();  
    return false;
  }
}



bool NodePrivate::updateVisPoly(NodeId const& id, const double& minDistance )
{ return updateVisPoly( blipData.at(id), minDistance); }
  
bool NodePrivate::updateVisPoly( Blip* const blip, const double& minDistance )
{
  // check whether the node has moved more than the required minDistance
  
  Geometry::Point_2D position2D = blip->position2D();
  Geometry::Point_2D visCenter2D = blip->visPolygonCenter();
  
  if( boost::geometry::distance( position2D,visCenter2D ) < minDistance )
  { return false; }
    
  Q_Q(Node); Q_ASSERT(q);
  qDebug() << q->qualifiedName() << ": Updateing visibility polygon for" << blip->name();

  auto ring =  VisilibityInterface::computeVisibilityPolygon<Geometry::Polygon_2D::ring_type>
               (position2D,environment2D);
               
  //
  // some data integrity checks
  //
  auto num_points = boost::geometry::num_points(ring);
  if ( num_points > 3)
  { 
    blip->setVisPolygon(ring);
    blip->setVisPolygonCenter(position2D);
    return true;    
  }
  else
  { 
    Q_Q(Node); Q_ASSERT(q);
    qCritical("\"%s\" : There is a problem computing the visibility polygon for \"%s\", as is only has %d points."
      ,q->qualifiedName().toStdString().c_str()
      ,blip->name().toStdString().c_str()
      ,static_cast<int>(num_points));
    return false;
  }                
  
}

/** 
 * \internal A top-down view of the view frustrum.
 *  
\code
                 +X
                  |
    farLeft \xxxxx|xxxxx/ farRight
             \xxxx|xxxx/
              \xxx|xxx/
     nearLeft  \--|--/ nearRight
                \ | /
                 \|/
                  +---------------- +Y
\endcode
  */
bool NodePrivate::updateFovPoly( Blip*const& blip, const double& minDistance, const double& minYaw_deg )
{
  
  // NOTE: if the value is 0 (zero), then the blip simply doesn't have a camera.
  if( blip->horizontalFov() == 0 )
  { return false; }
 
  // the current position of the blip
  Geometry::Point_2D position2D  = blip->position2D();
  
  // the current heading of the blip
  double             heading     = blip->yaw();
  
  // the center of the old/current FOV polygon
  Geometry::Point_2D fovCenter2D = blip->fovPolygonCenter();
  
  // the heading of the old/current FOV polygon
  double             fovHeading  = blip->fovPolygonHeading();
  
  // check whether the node has moved more than the required thresholds
  if( boost::geometry::distance( position2D,fovCenter2D ) < minDistance 
      and std::abs(fovHeading - heading) < minYaw_deg )
  { return false; }

  // The horizontal Field-of-View in degrees of the blip.
  const double hFov_deg = blip->horizontalFov();
    
  //NOTE:   For a 4:3 aspect ratio, the horizontal FOV should be roughly four thirds
  //   of the vertical FOV: hFov = 4/3 * vFov.
  
  double farDistance = 500; //FIXME: Magic Numbers;
  
  
  // the frustrum intersected with the environment
  auto fovPoly_0 = deltaPolygon(position2D,farDistance,heading,hFov_deg);

  // restrict this to the actually visible area
  // NOTE: This "reverse" approach is necessary as VisiLibity has no sense of
  // "looking in a direction".
  auto fovPoly_1 =  VisilibityInterface::computeVisibilityPolygon<Geometry::Polygon_2D::ring_type>
                (position2D,fovPoly_0);  
  
  blip->setFovPolygon(fovPoly_1);
  blip->setFovPolygonCenter(position2D);
  blip->setFovPolygonHeading(heading);
  

  return true;

}

/** \todo The connectedness logic is solemnly based upon the R/F polygon of the
 * dynamic Node. This leads to errors when the R/F polygon of the dynamic node
 * is potentially outdated wrt. to the dynamic node's position.
 * Currently this is the case when the dynamic Node is within a building as at
 * that point the R/F polygon doesn't get updated.
 */
bool NodePrivate::updateComGraph( NodeId const& dynamicId )
{
  using namespace boost;
  Q_Q(Node); Q_ASSERT(q);
  
  /** \internal The updateComGraph function is a method that aims to depict the
   * possible communication links of the current network as a graph. As such, 
   * each Node in the network is a vertex in the COM graph and each edge in the 
   * COM graph represent the fact that the two end vertices of that edege can 
   * directly communicate with each other. I.e. if vertex A and B are connected
   * in the COM graph, Node A and B can directly communicate.
   * By definition the COM graph is based upon assumptions/"simulations" for 
   * \e any connection/edge that is not connected to this Node, the "ownship" or
   * "host", as those are the only connections that the host is able to 
   * physically detect. All other connections either need to deduced from data
   * that the other Nodes share directly (i.e. each Node transmitting whom they 
   * are connected to) or from data that can be extracted by other means.
   * 
   * Currently the later is happenening: based on position and a simple 
   * \Delta -disc model the potential connections are guessed.
   */
  
  
  
  // TRUE if the edges in the COM graph are only to be computed for vertices
  // connected to this node, FALSE if the edges should be updated for all nodes
  // (FALSE enables the update of COM graph edges based on messages which are
  // technically not receivable, i.e. this node can keep track of the complete
  // network even in a not-fully-connected case--asuming that the non-receivable
  // messages somehow make it anyways ;) )
#ifdef WITH_RFSIMULATION
  const static bool limitToHostsConnectedGroup = false; //NOTE: this is a design choice kind of thing...
#else
  //NOTE: There are no non-receivable messages received in this case so it 
  // actually doesn't make a difference...
  const static bool limitToHostsConnectedGroup = true; 
#endif // WITH_RFSIMULATION
  
  /** \internal The update of the COM graph is based upon the assumption that 
   * at the time the graph is updated, only one Node/vertex is moving and all 
   * other nodes remain at their respective position. As such, there are two 
   * types of vertices/Nodes to consider: the one dynamic Node and all the other
   * static nodes. (Note that the host Node can be either or.) */
   bool dynamicNodeIsHostNode =  dynamicId == q->id();
  
  // TRUE if the search is to be restricted to nodes only connected to the host 
  // node, FALSE if the search should encompas all currently known Blips.
  // 
  // Truth table:
  // limitToHostsConnectedGroup | dynamicNodeIsHostNode | restrictToConnectedComponent
  // true                         false                   true
  // true                         true                    false
  // false                        false                   false
  // false                        true                    false
  //     
  // The main idea here is that if the ownship has moved, the ownship needs to 
  // check _all_ other nodes in order to determine whom it is connected to.
  // If a blip, i.e. not the ownship, moves, in the 
  // restrictToConnectedComponent==true case, all the ownship cares is whether 
  // or not that blib is still reachable (i.e. connected), either directly or 
  // via any route through the otherwise connected  blips. In the 
  // restrictToConnectedComponent==false case, it is _always_ necessary to 
  // search all nodes.
  bool restrictToConnectedComponent = limitToHostsConnectedGroup and !( dynamicNodeIsHostNode );
  
  // The "dynamic" blip is the blip that is "moving", all other are assumed to 
  // be static.
  Blip* dynamicBlip = nullptr;
  if( blipExists(dynamicId) )
  { dynamicBlip = &(q->blip( dynamicId )); }
  else
  {
    QString message;
    message
      .append(q->qualifiedName())
      .append(" : Blip(%s) doesn't exists, hence it is not connected.").arg( dynamicId.toQString() );
    qCritical("%s",message.toStdString().c_str());
    
    return false;
  };
  
  ComGraph::Vertex_Descriptor_t dynamicVertex = dynamicBlip->comGraphVertex();

  // disconnect the dynamic vertex from the comGraph (that is, if there are edeges)
  boost::clear_vertex(dynamicVertex,comGraph);
  
  
  //
  // 1. Recompute the COM Graph
  // ==========================
  //
  
  
  // A map associating each vertex to a connected component (subgraph) of
  // the overall comGraph.
  //NOTE: for this to work, the comGraph _needs_ to have a vertex_index !!!
  auto componentMap = Graph::components(comGraph);

  
  // The component of the graph the host node is in.
  int ownComponent = componentMap[ownshipBlip->comGraphVertex()];
  
  // The comGraph property map for the bundled data of all vertices, mapping
  // the vertex descriptors to vertex_id, i.e. NodeId.
  comGraphIdMap = get(vertex_id, comGraph); 
  
  // A list of all vertices which will be checked for connectedness with the 
  // dynamic Node, i.e. the sender of the currently processed message
  std::vector<ComGraph::Vertex_Descriptor_t> vertexList;
  
  if( restrictToConnectedComponent == true)
  { // vertexList = list of all vertices in the connected component of the 
    // receiver, i.e. this node, aka the ownship, aka the host
    
    // create a list of all vertices in the same component as this node, i.e. the ownshipVertex
    std::vector<ComGraph::Vertex_Descriptor_t> connectedVertices;
    
    for( ComGraph::Vertex_Descriptor_t i = 0; i<componentMap.size(); ++i) // remember, the key type of component _has_ to be the vertex_descriptor type of the comGraph - hence this works
    {
      if( componentMap[i] == ownComponent )
      { connectedVertices.push_back(i); }
    }
    
    // some sanity checks
    Q_ASSERT( connectedVertices.size() >= 1 ); // This list should at least contain this node...
  
    vertexList = connectedVertices;   
  }
  else // restrictToConnectedComponent == FALSE
  { // vertexList = list of all vertices in the comGraph
  
    // create a list of all vertices in the comGraph
    std::vector<ComGraph::Vertex_Descriptor_t> allVertices;
    
    
    for(auto itRange = vertices(comGraph); // itRange  type is std::pair<vertex_iterator, vertex_iterator>
        itRange.first!=itRange.second; ++itRange.first)
    {
      ComGraph::Vertex_Descriptor_t staticVertex = *(itRange.first);
      if(q->blipExists(comGraphIdMap(staticVertex)))
      {allVertices.push_back(staticVertex);};
    };
    
    // copy it over to the vertex list for later processing
    vertexList = allVertices;  
  }
   
   unsigned int edgeIndex = 0;
   
   
  /** \todo Make this an iterator range based function
    *   Although this would mean that the the list of vertices in the 
    *   connected component of the receiving node would have to be computed
    *   differently...
    */
  // get the position of each of the vertices in the list and
  // check if they are wihtin the R/F polygon of the dynamicVertex.
  for( ComGraph::Vertex_Descriptor_t staticVertex : vertexList )
  {
    
    if( staticVertex == dynamicVertex )
    { continue; } //NOTE: avoiding self loops
    
    NodeId sv_id = comGraphIdMap(staticVertex);
    
    auto staticVertexPosition = q->blip(sv_id).position2D();
    
    bool withinRfPoly = geometry::within( staticVertexPosition,dynamicBlip->rfPolygon());
 
    if(false){ // debug messages
      auto dynamicRfPolygon = dynamicBlip->rfPolygon();
      std::cout << "Dynamic Blip: " << dynamicBlip->name().toStdString() << " at " << dynamicBlip->position2D() << std::endl;
      std::cout << "Static Blip: " << q->blip(sv_id).name().toStdString() << " at " << staticVertexPosition << std::endl;
      std::cout << "R/F poly area: " << boost::geometry::area(dynamicRfPolygon) << std::endl;
      std::cout << "R/F poly center: " << dynamicBlip->rfPolygonCenter() << std::endl;
      std::cout << "distance Dynamic Blip to R/F Poly center: " << boost::geometry::distance(staticVertexPosition,dynamicBlip->rfPolygonCenter()) << std::endl;
      qDebug() << "within R/F polygon :"<< withinRfPoly;
    }
    
    
    if( withinRfPoly )
    { // the currently processed (static) vertex of the vertexList is inside the R/F 
      // polygon of the dynamic vertex (i.e. the sender of the message causing this comGraphUpdate)
      
//       auto staticVertexComponent =  componentMap[staticVertex];
      
      // create an edge in the comGraph to indicate this possible COM link
      auto pair = boost::add_edge(staticVertex,dynamicVertex,comGraph);
      
      double edge_length = boost::geometry::distance(staticVertexPosition,dynamicBlip->position2D());    
      boost::put(edge_quality, comGraph, pair.first, static_cast<uint>(correspondingRfZone(edge_length)));
      boost::put(edge_weight, comGraph, pair.first, edge_length );
      boost::put(edge_index, comGraph, pair.first, edgeIndex++);
      
// make graphics aware of new edeges ???
//       if( pair.second == true )
//       { emit ComGraph::edge_added(); }
    }
  } 
  
  
  //
  // 2. Determine Connectedness
  // ==========================NodeId,Blip*
  //
  
  // recompute the connected sets
  componentMap = Graph::components(comGraph);
  ownComponent = componentMap[ownshipBlip->comGraphVertex()];
  
  // revisit all potentially affected vertices and updated the connectedness
  for( ComGraph::Vertex_Descriptor_t staticVertex : vertexList )
  {
    q->blip(comGraphIdMap(staticVertex)).setConnectednessToHost(componentMap[staticVertex] == ownComponent);
  }
  

  if( false ) // debug printout
  { 
    // print the graph, label the vertices by vertex index (automatic from the vecS type) 
    print_graph(comGraph,comGraphNameMap);
  }
  
//NOTE: DO NOT DO THIS HERE !!! updateComGraph is called from the realm of the
// DDS listeners, i.e. it comes potentially not from the main, i.e. graphics,
// thread. As such, messing with the graphics here is prone to break things! 
// To counteract this, updateComGraphGraphics is located in the processing of 
// the tertiary data, which, as it is triggered from processReceivedMessage(),
// is moved in or called from the correct (GUI) thread.
//
// #ifdef WITH_GRAPHICS 
//   updateComGraphGraphics();
// #endif //WITH_GRAPHICS
//
// use the apropriate SIGNAL:
  emit q->comGraphChanged();
  
  return q->blip(dynamicId).isConnectedToHost();
};


QList<NodeId> NodePrivate::comRelayNodes(NodeId const& target, QList<NodeId> const& excludes) const
{
  Q_ASSERT( blipExists(target) );
  Q_Q(const Node); Q_ASSERT(q);
  using namespace boost;
  
  if( ! q->blip(target).isConnectedToHost() )
  { 
    // the target isn't COM-connected, hence there are no blips providing the
    // respective R/F coverage.
    return QList<NodeId>();
    // NOTE: If this step wouldn't be done, the componentMap will contain one
    // element, the target - which is correct from the component map 
    // perspective, but wrong for the R/F converage aspect.
  }
  
  ComGraph::Graph_t comGraphCopy = comGraph;
  
  for( auto id_excl : excludes )
  {
    Q_ASSERT( blipExists(id_excl) );
    
    // disconnect the ownship vertex from the comGraph copy (that is, if there are edeges)
    clear_vertex(q->blip(id_excl).comGraphVertex(),comGraphCopy);
  }
  
  // compute the connected components
  std::vector<int> componentMap = Graph::components(comGraphCopy);
  int seedComponent = componentMap[q->blip(target).comGraphVertex()];
  
  
  // The comGraph property map for the bundled data of all vertices, mapping
  // the vertex descriptors to vertex_id, i.e. NodeId.
  ComGraph::VertexIdMap_t nodeIdMap = get(vertex_id, comGraphCopy); 
  
  QList<NodeId> neighbors;
  
  // visit all vertices and check their component id
  ComGraph::Vertex_it_t vi, vi_end;
  for( tie(vi,vi_end) = vertices(comGraphCopy);vi!=vi_end;++vi)
  { 
    if( componentMap[*vi]==seedComponent )
    { neighbors.push_back(nodeIdMap[*vi]); }
  }
  
  return neighbors;
}

#ifdef WITH_GRAPHICS
void NodePrivate::updateComGraphGraphics()
{
  QMutexLocker locker(&mutex_graphics_comGraph);
  
  bool visibility = true; //FIXME: Magic Numbers: default comGraphGraphics visibility is set/created here.
  
  if( comGraphGraphics )
  { 
    visibility = comGraphGraphics->isVisible();
    qDeleteAll(comGraphGraphics->childItems());
  }
  
  delete comGraphGraphics;
  comGraphGraphics = nullptr;
  
  auto rfLinkQualityMap = boost::get(boost::edge_quality, comGraph);
  
  auto edgeColorMap = [=](ComGraph::Edge_Descriptor_t edge)->QPen
  {
//     switch( boost::get(boost::edge_quality, comGraph, edge) )
    switch( rfLinkQualityMap[edge] )
    {
      case  static_cast<int>(RfZone::Green)   : return Graphics_2D::Pens::Outline(Qt::green);
      case  static_cast<int>(RfZone::Yellow)  : return Graphics_2D::Pens::Outline(Qt::yellow);
      case  static_cast<int>(RfZone::Orange)  : return Graphics_2D::Pens::Outline(QColor(255,128,0));
      case  static_cast<int>(RfZone::Red)     : return Graphics_2D::Pens::Outline(Qt::red);
      case  static_cast<int>(RfZone::White)   : /* fall through intended */
      default               : return Graphics_2D::Pens::Outline(Qt::darkGray);
    };
  };
  
  auto vertexPen = [=](ComGraph::Vertex_Descriptor_t vertex)->QPen {return QPen();}; 
  
  ComGraph::VertexPosition2dMap_t positionMap = boost::get(boost::vertex_position_2D, comGraph);
  comGraphGraphics = Graphics_2D::generateQtGraphicsFromGraph
    < ComGraph::Graph_t
    , ComGraph::VertexPosition2dMap_t
    , ComGraph::Edge_Descriptor_t
    , ComGraph::Vertex_Descriptor_t
    >
    (comGraph
    ,positionMap
    ,edgeColorMap
    ,vertexPen
    ,/*drawEdges*/ true
    ,/*drawVertices*/ false
    );
  comGraphGraphics->setVisible(visibility);
  comGraphGraphics->setZValue(NodePrivate::ComGraphGraphicZLevel);
  comGraphGraphics->setParentItem(datumItem);
  


};
#endif // WITH_GRAPHICS

void NodePrivate::updateStatusMessage()
{
  auto stateSet =  stateMachine->configuration();
  
  // This is a "debug" parameter which determines whether or not to
  // honor the reportedStates (sub-)set of states to report.
  constexpr bool honorReportedStatesSubSet = true; 
 
  statusMessage->mode.ensure_length(stateSet.count(),stateSet.count());
  
  int index = 0;
  for( auto& abstractState : stateSet )
  {
    State* state = dynamic_cast<State*>(abstractState);
    
    if(state != nullptr)
    {
      if( honorReportedStatesSubSet == false)
      { statusMessage->mode[index++] = state->stateId(); }
      else if ( /*reducedModeSet == true and */reportedStates.contains(state->stateId() ) )
      { statusMessage->mode[index++] = state->stateId(); }
    }
  }
  
  // indicate the actual length in case there are invalid entries left at the end
  statusMessage->mode.length(index);  
  
  statusMessage->closestObstacle = ownshipBlip->closestObstacleId();
  statusMessage->target = ownshipBlip->targetId();
    
  // highjack the processing functions to transfer the data into the ownshipBlip
  Q_Q(Node); Q_ASSERT(q);
  q->processReceivedMessage(ownshipId,*statusMessage);
}

double NodePrivate::rZeroOne() const
{ return  static_cast<double>(std::rand())/RAND_MAX; }

double NodePrivate::rMinusOneOne() const
{ return (2.0 * rZeroOne()) - 1.0; }

void NodePrivate::randomWalk(double const & distance)
{
//   static const double distance = 0.5;
  
  stateMessage->position.x += distance * rMinusOneOne();
  stateMessage->position.y += distance * rMinusOneOne();
  
  Q_Q(Node); Q_ASSERT(q);
  q->processReceivedMessage(ownshipId, *stateMessage);
}

bool NodePrivate::blipExists(NodeId const & id) const
{
 if( blipData.count(id) > 0)
 { return true; }
 else
 { return false;}
}


unsigned int NodePrivate::blipCount(IDL::Data::StateDescriptorId const & state) const
{
  unsigned int count = 0;
  
  for( auto& pair : blipData )
  {
    Blip const* b = pair.second;
    if( b->mode().contains(state) )
    { count++; }
  }
  
  {// Debug output.
    Q_Q( const Node); Q_ASSERT(q);
    qDebug() << q->qualifiedName() << ": Counting Blips. There are" << count
      << "Blips in State" << state;
  }
  return count;
}


void NodePrivate::postCommand(IDL::Commands::CommandTypes const & command)
{
  Q_Q(Node); Q_ASSERT(q);
  qDebug()<< q->qualifiedName() << ": Posting IdlCommandType" << command;
  stateMachine->postEvent(new Events::UserCommand(command));
}




void NodePrivate::setEnvironment ( const Geometry::Polygon_2D& newEnvironment )
{
  Q_Q(Node); Q_ASSERT(q);
  
  Q_ASSERT( !boost::geometry::intersects(newEnvironment) );
  
  if( q->isInitialized() )
  {
#ifdef WITH_GRAPHICS
    environmentItem->setGeometry(newEnvironment);
#else // WITH_GRAPHICS
    boost::geometry::assign(environment2D,newEnvironment); // copy the loaded arena
#endif // WITH_GRAPHICS
    updateRouteGraph(); 
    updateDerivedGraphs();
  }
  else // isInitialized == false
  { boost::geometry::assign(environment2D,newEnvironment); } // copy the loaded arena
  
}



inline QDebug operator<<(QDebug dbg, const Eigen::Vector3d & vector )
{
  dbg.nospace() << "(" << vector.x() 
    << ", " << vector.y()
    << ", " << vector.z() << ")";
  return dbg.space();
};


// std::list<RouteGraph::Vertex_Descriptor> NodePrivate::getBranchNodes( ElementId const & targetId )
// {
//  return std::list<RouteGraph::Vertex_Descriptor>(); 
// }


// void NodePrivate::computeComLoopsAroundTargetObstacle(){ return; }
void NodePrivate::computeComLoopsAroundTargetObstacle(PathComputationData* data)
{
  
  // the id of the target obstacle
  data->mutex.lock();
    ElementId targetId = data->target;
  data->mutex.unlock();
  
  Q_Q(Node); Q_ASSERT(q);
  Q_ASSERT(q->obstacleExists(targetId));
  
  
  //
  // Get the intersection points of the target cell
  // ==============================================
  // Find the intersection points (bifurcation nodes) of the target cell
  // and get all the paths from there to the ownship.
  //

  // the list of Locations that are intersection points around the target
  std::list<LocationId> targetCellBifurcationLocations;

  for( BifurcationGraph::Vertex_Descriptor const& biVertex 
        : obstacleBifurcationVertexData.at(ownshipBlip->targetId()) ) // cannot use operator[] for const correctnes
  {
    // check that the intersection vertices are within the COM corridor. They
    // can be outside for large obstacle cells that have an obstacle fairly 
    // offset from the cell's centroid.
    
    LocationId locId = boost::get(boost::vertex_locationId, bifurcationGraph, biVertex);
    if(  boost::geometry::covered_by(locationData.at(locId), comCorridor) )
    { 
      targetCellBifurcationLocations.push_back(locId);
    }
  }
  
   
  
  // the list of possible guard locations
  std::list<LocationId> guardLocationCandidates;

  /** \internal Step 1: Determine possible guard locations  */
  {
    enum class Option {
      /** all the vertices of the related obstacle cell */
       A
      
      /** option A plus all the vertices of the route graph edges leading
       * away from the cell edge (this is like a "star" around the obstacle) */
      ,B
      
//       /** take any/all of the edges used in A and/or B and resample the lines 
//        * to get more (evenly) spaces guard locations. */
//       ,C
    } option = Option::B;
    
    if(option == Option::A){    
      std::set<RouteGraph::Vertex_Descriptor> routeVertexSet 
        = obstacleRouteVertexData[targetId];
        
      for( auto rVertex : routeVertexSet )
      {
        using namespace boost;
        guardLocationCandidates.push_back(get(vertex_locationId, routeGraph, rVertex));
      } 
    }
    else if(option == Option::B){ 
     
      std::set<BifurcationGraph::Edge_Descriptor> biEdgeSet;
           
      //
      // populate the biEdgeSet with all edges around and away from the target
      //
      
      std::set<BifurcationGraph::Vertex_Descriptor> biVertexSet
        = obstacleBifurcationVertexData[targetId];
      for( auto biVertex : biVertexSet )
      {
        using namespace boost;
        graph_traits<BifurcationGraph::Graph>::out_edge_iterator ei, ei_end;
        for( tie(ei, ei_end)=out_edges(biVertex, bifurcationGraph);
            ei != ei_end;  ++ei )
        { biEdgeSet.insert(*ei); }
      }
      
      //
      // translate the biEdges into the corresponding location sets
      //
      
      for( auto biEdge : biEdgeSet )
      {
        using namespace boost;
        BifurcationGraph::Vertex_Descriptor s = source(biEdge, bifurcationGraph);
        BifurcationGraph::Vertex_Descriptor t = target(biEdge, bifurcationGraph);
        
        // add the locations of the source and target vertices
        guardLocationCandidates.push_back(get(vertex_locationId, bifurcationGraph,s));
        guardLocationCandidates.push_back(get(vertex_locationId, bifurcationGraph,t));
        
        
        // not all biEdges are represented in the location data set as some 
        // don't have interim points.
        /** \TODO : should this be "fixed" and simply a zero set be entered? */
        if( biEdgeLocationData.count(biEdge) != 0 )
        {
          // add the locations of the route graph vertices the biEdge represents
          std::set<LocationId> locationSet = biEdgeLocationData.at(biEdge);
          guardLocationCandidates.insert( guardLocationCandidates.end(),
            locationSet.begin(), locationSet.end() );
        }
      }
      
    }
    else
    { 
      //NOTE: qFatal() will _abort_ on UNIX systems!
      qFatal("%s: Cannot determine possible guard location option."
        ,q->qualifiedName().toStdString().c_str());
    }
//     qDebug() << q->qualifiedName() << ": Detected" << guardLocationCandidates.size() 
//       << "guard location candidates for" << target.objectName();
  }
  
  qDebug() << q->qualifiedName() << ": Selected" << guardLocationCandidates.size()
    << "guard location candidates around target" << obstacleData.at(targetId).objectName();
  
  
    
  // the graph holding all possible directed R/F connections amongs all the
  // guard location candidates
  DirectedRfGraph::Graph directedRfGraph;

  // the interior angle around the target covered by a particular edge
  std::map<DirectedRfGraph::Edge_Descriptor, double> windingAngle;
  
  // a reverse lookup mal from LocationId to directedRfGraph vertices.
  std::unordered_map<LocationId, DirectedRfGraph::Vertex_Descriptor> locationId_to_dirRfVertex;
  
  /** \internal Step 2: Compute a directed visibilty graph
  * - compute the undirected rf graph for the guard locations in the target 
  *   environment (the comCorridor will be used as the visibility environment)
  * - get a directional version of that graph:
  * -- project any undirected edge of a guard vertex on the vector from the guard 
  *   location to the centroid of the target obstacle (think cross product)
  * -- if the result is positive, keep the edge, make it an out-edge
  * -- if the result is negative, discard the edge
  * 
  */
  {
    /** \internal Instead of porting this over to a VisiLibity environment
     * (guard locations and target environment), I opted to copy the approach
     * that VisiLibity takes in order to compute a visibiliyt graph: "just" 
     * compute the visibility polygon for each guard and check the other guard 
     * locations for inclusion. Yes, there are numerically less expensive
     * methods out there, but this one doesn't require any new code... lazy me.
     * 
     * \internal As this essentially gives me full control over the visibility
     * graph creation process, the step about first computing an undirected 
     * graph can be skipped as guard locations can be checked even before the 
     * call to boost::geometry::within.
     */
    
    using namespace boost;
    

    
    //
    //  add all potential guard positions to the graph
    //
    
    for( LocationId locId : guardLocationCandidates )
    {
      static unsigned int vertexIndex = 0;
      auto vertex =  add_vertex(directedRfGraph);
      put(vertex_locationId, directedRfGraph, vertex, locId);
      put(vertex_index, directedRfGraph, vertex, vertexIndex++);
      
      locationId_to_dirRfVertex[locId] = vertex;
    }

    //
    // Build the directional R/F graph
    // ===============================
    // As this essentially gives me full control over the visibility
    // graph creation process, the step about first computing an undirected 
    // graph can be skipped as guard locations can be checked even before the 
    // call to boost::geometry::covered_by.
    //

   
    unsigned int edgeIndex = 0;
    
    //NOTE: maybe this should be the target environment ?
    // (after all, that is what it originally was computed for...)
    VisiLibity::Environment visEnv = VisilibityInterface::createEnvironment(comCorridor);
    visEnv.enforce_standard_form();  
    Q_ASSERT( visEnv.is_valid() ); //FIXME: This should not create any errors.... :/
    
    
    
    auto locationIdMap = get(vertex_locationId, directedRfGraph);
    auto locationMap = make_compose_property_map(location_pMap, locationIdMap);
    

    
    
    DirectedRfGraph::Vertex_Iterator vOuter, vOuter_end;
    tie(vOuter, vOuter_end) = vertices(directedRfGraph);
    
    RfGraph::Vertex_Iterator const vFirst = vOuter;
    
    // NOTE: this starts at the 'second' vertex as it is sufficient to check 
    // only a diagonal matrix of vertices (the visibility is symmetric, the
    // the added directionality is somewhat artifical)
    for( vOuter++ ; vOuter != vOuter_end; ++vOuter)
    {
      // get the assoc. visible area of the outer vertex
      
      Location vOuterLoc = locationMap[*vOuter];
      VisiLibity::Point vOuterPos = VisilibityInterface::makeVlPoint( vOuterLoc );
      VisiLibity::Visibility_Polygon visPoly(vOuterPos,visEnv,VisilibityInterface::epsilon); 
      auto visibleArea = VisilibityInterface::makeRing<Geometry::Polygon_2D::ring_type>(visPoly);
      
      
      DirectedRfGraph::Vertex_Iterator vInner;
      for( vInner = vFirst; vInner != vOuter; vInner++ )
      {   
        Location vInnerLoc = locationMap[*vInner];

        Geometry::Point_2D vInnerPos = vInnerLoc;

        // check distance as a "cheap" inital check so that we can skip the 
        // computation of the deltaDisc and the clipping of the visibleArea
        
        auto edgeLength = distance(vOuterLoc, vInnerPos);
        
        auto rfZone = correspondingRfZone( edgeLength );  
        if( rfZone == RfZone::White ) 
        { continue; } // distance is outside R/F limits, i.e. not in range   
      
      
        // if that clears, check for inclusion by the visible area.
        // "covered_by" is used (as opposed to "within") to include the boundary
        // of the area as some route graph vertices will be located there
      
        if( geometry::covered_by(vInnerPos, visibleArea) )
        {
          DirectedRfGraph::Edge_Descriptor edge;
          bool edgeIsNew;


          // the targeted obstacle element.
          Obstacle const& targetObs = q->obstacle(targetId);
            
          Geometry::Point_2D targetCentroid = targetObs.centroid();
          
          
          // the vector from the guard position to the centroid of the target
          Eigen::Vector3d innerToCentroid(targetCentroid.x()-vInnerPos.x(),
                                          targetCentroid.y()-vInnerPos.y(),
                                          0);
      
          Eigen::Vector3d innerToOuter(vOuterPos.x()-vInnerPos.x(),
                                        vOuterPos.y()-vInnerPos.y(),
                                        0);
          //
          // establish direction and create edge
          // 
          if( innerToCentroid.cross(innerToOuter).z() < 0 )
          { 
            tie(edge, edgeIsNew) = add_edge(/*from|source*/ *vOuter,
                                            /*to|target*/   *vInner,
                                               directedRfGraph);
          }
          else
          {
            tie(edge, edgeIsNew) = add_edge(/*from|source*/ *vInner,
                                            /*to|target*/   *vOuter, 
                                               directedRfGraph);
          }     
          
          put(edge_index, directedRfGraph, edge, edgeIndex++);
          put(edge_quality, directedRfGraph, edge, static_cast<uint>(rfZone));
          put(edge_weight, directedRfGraph, edge, edgeLength);          
          
          //
          // compute covered inner angle and store that for later
          //

          Eigen::Vector3d centroidToInner = -innerToCentroid;
          Eigen::Vector3d centroidToOuter = centroidToInner + innerToOuter;
            
          double cosineAngle = centroidToOuter.normalized().dot(centroidToInner.normalized());
          double angle = std::acos(cosineAngle);
            
          windingAngle[edge]= angle;
        }
      }
    }
  }

  qDebug() << q->qualifiedName() << ": Created a directed R/F graph with" 
    << boost::num_edges(directedRfGraph) << "edges among those locations.";
  
#if defined WITH_GRAPHICS && defined DEBUG_GRAPHICS
{
  data->mutex.lock();
    data->directedRfGraph = directedRfGraph;
  data->mutex.unlock();
}
#endif // WITH_GRAPHICS&& DEBUG_GRAPHICS



  // the list of comLoop vertex sequences, keyed by route graph intersection 
  // they originate from.
  std::unordered_map< LocationId, std::list< std::list< LocationId >>> comLoops;
 
  
  /** \internal Step 3: Find a suitable set of guard locations to loop around 
   *  the target
   * 
   * - compute all loops in the directed visibility graph (which means that 
   * the guards can see each other and they are distributed around the target)
   * - use a heuristic to pick a favorite
   * -- smallest vertex count (i.e. least number of guards needed)
   * -- shortest overall length ( not sure what that would mean )
   * -- "best" ratio of length-to-vertex count (i.e. best datalink bandwidth or 
   *    throughput)
   * -- ...
   * - return the selected ones as result.
   * 
   * The algorithm uses a sort of maximum-range metric and uses that to detect
   * and identify non-winding cycles in the directed visibility graph.
   * To do so, an edge weight representing the inner angle covered by that
   * edge is computed. Once the accumulated weight/angle over a path through 
   * the visibility graph reaches 360 degrees, this is a loop (at which point 
   * the target of the last edge should also be identical to the source of the 
   * first). If the accumulated angle is > 360 degree, the (depth-first-search
   * based) search can be stopped as no non-winding cycle will be found on that
   * particular branch.
   * In order to limit the recursion in the presence of the thousands of 
   * possible loops, a maximum hop count is also given (which could also match
   * with the maximum number of supportive UAVs available to cover the target 
   * [it's feature, not a bug.]).
   */
  {  
    
    // NOTE: The commented out code doesn't work as it would/will also find 
    // cycles that wind several times around the obtacle. (An examlple would be 
    // a Pentagram, which has two cycles around its centroid.)    
    //     DirectedVisibilityGraph::HawickCycleVisitor<> visitor;
    //     hawick_circuits(/* Graph */visbilityGraph
    //                     ,/* Visitor */ visitor
    //                     ,/* VertexIndexMap */ get(vertex_index, visbilityGraph));
    
    using namespace boost;
    
    using MapType = associative_property_map<std::map<DirectedRfGraph::Edge_Descriptor, double>>;
    MapType windingAngleMap(windingAngle);
//     auto windingAngleMap = make_assoc_property_map(windingAngle);
 

    LoopDetector<DirectedRfGraph::Graph, MapType >
      loopDetector(directedRfGraph, windingAngleMap, /*parent*/ nullptr);
 

      
    /** \todo: 
     * - make loop detector edge based
     * - combine ingress and loops to a single combination
     * - create a score function for these combinations
     * - pick the one with the best score to continue.
     */
    
    /** \todo Check the loop for actual coverage
     * 
     * Depending on the environment it can happen that a loop not only 
     * encompasses the actual target, but also one of the neighboring obstacles.
     * As such it might be prudent to check for that simply (?) through a 
     * "within" check of the obstacles centroid in the loop geometry.
     */

    
    auto locationIdMap = get(vertex_locationId, directedRfGraph);
//     auto locationMap = make_compose_property_map(location_pMap, locationIdMap);
      
    for( LocationId startLocId : targetCellBifurcationLocations )
    {
      DirectedRfGraph::Vertex_Descriptor startVertex
        = locationId_to_dirRfVertex[startLocId];
      
// DEBUG : taking time of the loop detector    
//       QTime tictoc;
//       qDebug() << q->qualifiedName() << ": TIC (Starting LoopDetector) for LocationId"
//         << startLocId;
//       tictoc.start();              
      loopDetector.start(startVertex, /*threshold*/ 2*M_PI); 
// DEBUG : taking time of the loop detector   
//       qDebug() << q->qualifiedName() << ":TOC (Done with LoopDetector):" 
//         << tictoc.elapsed() << "ms.";
    

      
      std::list<  std::list<LocationId> >loopLocationSequences;

      
      /** \todo FIXME: three nested for loops... that cannot be good... */
      
      for( auto pair : loopDetector.detectedLoops )
      {
//         unsigned int hopCount = pair.first;
        
        // all detected loops that start at the startVertex with a particular 
        // hop count
        std::list<std::list<DirectedRfGraph::Vertex_Descriptor>> loops 
          = pair.second;
        
        for( std::list<DirectedRfGraph::Vertex_Descriptor> loop : loops )
        {
          //NOTE: The current LoopDetected creates loops in which each loop 
          // starts at a bifurcation vertex which is _not_ repeated as the last 
          // entry in the list!
          
          std::list<LocationId> locSequence;
          for( DirectedRfGraph::Vertex_Descriptor vertex : loop )
          {  
            LocationId id = get(locationIdMap, vertex);
            locSequence.push_back( id ); 
          }
          loopLocationSequences.push_back(locSequence);
        }
      }
      
      comLoops[startLocId] = loopLocationSequences;
      
#if defined WITH_GRAPHICS && defined DEBUG_GRAPHICS
{
//        
//NOTE: DO _not_ create any graphics in here as that will create segfaulting
//  issues. (I assume the issue is that this funcion is run in the QtConcurrent
//  framework and as such in a different thread than where the QGraphicsItemGroup
//  pointer holding the graphics lives....)    
//           
//       if(loopDetector.detectedLoops.size() > 0 )
//       {
//         auto loopGraphics = new QGraphicsItemGroup();
//             
//         auto lowHopLoops = loopDetector.detectedLoops.begin()->second;
//         
//         int maxLoopGraphics = 10;
//         
//         for( std::list<DirectedRfGraph::Vertex_Descriptor> loop : lowHopLoops )
//         {
//           loop.push_back(loop.front());
//           
//           for( auto vi = ++loop.begin(); vi!= loop.end(); ++vi )
//           {
//               auto tailPos = get(locationMap, *(--vi));
//               auto headPos = get(locationMap, *(++vi));
//             
//               auto a = new Graphics_2D::Arrow(tailPos.x(),tailPos.y(),
//                                               headPos.x(), headPos.y(),
//                                               loopGraphics );
//           }
//           loopGraphics->setParentItem(datumItem);
//           
//           if( --maxLoopGraphics == 0)
//           { break; }
//         }
//         
//         qApp->processEvents();          
//       }
}
#endif // WITH_GRAPHICS&& DEBUG_GRAPHICS
      
        
    } 
    
    
    
  }
  
  
  data->mutex.lock();
    data->loops = comLoops;
  data->mutex.unlock();
  
  return ;
}

/** \todo A LocationId is \e not a unique identifier in the context of RfGraphs
 * as those also could use NodeId for the "dynamic" vertices, i.e. Blips -- all 
 * of which have a NULL_LOCATION_ID. I.e. this method \e will \e fail the 
 * moment dynamicRfVertex.size() > 1 ! */
std::multimap<double, RfGraph::Graph> NodePrivate::combineIngressAndLoops(
  std::unordered_map< LocationId, std::list<LocationId>> ingressData,
  std::unordered_map< LocationId, std::list< std::list< LocationId >>> loopData
  )
{  
  
  
  Q_ASSERT( dynamicRfVertex.size() <= 1 ); //TODO: see note above.
  
  /** \fixme This ought to be a subgraph of rfGraph... but I couldn't get that 
   * to work with non-vecS vertex and edge lists... :/ */
  RfGraph::Graph optionalFormation;
    
  auto globalRfVertex = [&](RfGraph::Vertex_Descriptor localVertex) -> RfGraph::Vertex_Descriptor
  {
    LocationId locId = boost::get(boost::vertex_locationId, optionalFormation, localVertex);  
    if( locId != NULL_LOCATION_ID )
    { return locationId_to_rfVertex.at(locId); } //NOTE: using at() instead of operator[] to get the range check }
    else
    {
      NodeId nId = boost::get(boost::vertex_id, optionalFormation, localVertex);
      return dynamicRfVertex.at(nId);
    }
  };
  
  auto globalRfEdge = [&]( RfGraph::Edge_Descriptor localEdge) -> RfGraph::Edge_Descriptor
  {
    
    RfGraph::Vertex_Descriptor localSource = boost::source(localEdge, optionalFormation);
    RfGraph::Vertex_Descriptor localTarget = boost::target(localEdge, optionalFormation);
    
    std::pair< RfGraph::Edge_Descriptor, bool> pair =
      boost::edge(/*source*/ globalRfVertex(localSource)
                 ,/*target*/ globalRfVertex(localTarget)
                 ,/*graph*/ rfGraph);
    Q_ASSERT( pair.second );
      
    return pair.first;
  };
  
  std::multimap<double, RfGraph::Graph> scoredFormations;
  
  for( auto ingressPair : ingressData )
  {
    LocationId bifurcationId = ingressPair.first;
    std::list<LocationId> ingressLocations = ingressPair.second;
    
    // iterate over all loops that match the endpoint bifurcation vertex of the 
    // ingress path with their fixed start location
    for( auto loopLocations : loopData[bifurcationId] )
    {
      optionalFormation.clear();
      
      // build the formation
      {  
        using namespace boost;
      
        RfGraph::Vertex_Index_Map::value_type vIndex = 0;
        RfGraph::Edge_Index_Map::value_type eIndex = 0;
        RfGraph::Vertex_Descriptor last_v = RfGraph::Graph::null_vertex();
        
        
        // add the ingress path to the optional formation
        for( LocationId locId : ingressLocations )
        {
          RfGraph::Vertex_Descriptor v = add_vertex(optionalFormation);
          
          put(vertex_index, optionalFormation, v, vIndex++);
          put(vertex_locationId, optionalFormation, v, locId);

          put(vertex_id, optionalFormation, v,
              get(vertex_id, rfGraph, locationId_to_rfVertex.at(locId) ));
          
          if( vIndex > 1 )
          {
            auto pair = add_edge(v,last_v, optionalFormation);
            
            Q_ASSERT(pair.second);
            
            RfGraph::Edge_Descriptor e = pair.first;
            
            RfGraph::Edge_Descriptor globalEdge = globalRfEdge(e);
            
            put(edge_index, optionalFormation, e, eIndex++);
            put(edge_weight, optionalFormation, e, 
                get(edge_weight, rfGraph, globalEdge));
            put(edge_quality, optionalFormation, e, 
                get(edge_quality, rfGraph, globalEdge));
          }
          
          last_v = v;
        }
        
//         { // DEBUG
//           Q_Q(Node); Q_ASSERT(q);
//           qDebug() << q->qualifiedName() << ": added" << vIndex << "ingress vertices.";
//         }
//         auto vIndex_interim = vIndex; //DEBUG
        
        RfGraph::Vertex_Descriptor bifurcation_v = last_v;
        Q_ASSERT( get(vertex_locationId, optionalFormation, bifurcation_v) == bifurcationId );
        

        // add the loop to the optional formation
        for( auto locId_it = ++loopLocations.begin(); //NOTE: the ++ increment is there to start at the second vertex in the loop
            locId_it != loopLocations.end();
            ++locId_it)
        {
          RfGraph::Vertex_Descriptor v = add_vertex(optionalFormation);
          
          put(vertex_index, optionalFormation, v, vIndex++);
          put(vertex_locationId, optionalFormation, v, *locId_it);
          
          
          auto pair = add_edge(v,last_v, optionalFormation);
            
          Q_ASSERT(pair.second);
            
          RfGraph::Edge_Descriptor e = pair.first;
            
          RfGraph::Edge_Descriptor globalEdge = globalRfEdge(e);
            
          put(edge_index, optionalFormation, e, eIndex++);
          put(edge_weight, optionalFormation, e, 
              get(edge_weight, rfGraph, globalEdge));
          put(edge_quality, optionalFormation, e, 
              get(edge_quality, rfGraph, globalEdge));
          
          last_v = v;
        }
        
//         { // DEBUG
//           Q_Q(Node); Q_ASSERT(q);
//           qDebug() << q->qualifiedName() << ": added" << vIndex-vIndex_interim << "loop vertices.";
//         }
        
        { // the edge closing the loop at the bifurcation vertex
          auto pair = add_edge(last_v,bifurcation_v, optionalFormation);
            
          Q_ASSERT(pair.second);
            
          RfGraph::Edge_Descriptor e = pair.first;
            
          RfGraph::Edge_Descriptor globalEdge = globalRfEdge(e);
            
          put(edge_index, optionalFormation, e, eIndex++);
          put(edge_weight, optionalFormation, e, 
              get(edge_weight, rfGraph, globalEdge));
          put(edge_quality, optionalFormation, e, 
              get(edge_quality, rfGraph, globalEdge)); 
        }
        
      }
      
      // score the formation
      double score = computeFormationScore(optionalFormation);     
      
      scoredFormations.emplace(score, optionalFormation);
      
      Q_Q( Node ); Q_ASSERT(q);
      qDebug() << q->qualifiedName() << ": created an optional R/F formation to" 
       << obstacleData[ownshipBlip->targetId()].objectName() << "with" 
       << boost::num_vertices(optionalFormation) << "vertices and a score of" 
       << score;
       
        
       
      /** \todo HACK: break after the first loop, for now that is sufficient
       * as the first loop (should) be one of the loops with the smallest number
       * of hops for that bifurcation point and as that is currently all that 
       * goes into the score, any other loop cannot be better... */
      qWarning() << q->qualifiedName() << ": HACK: Stopped the creating of optional formation graphs as with the current simple scoring (hop count), no better solution will be added.";
      break; 
    }
  }    
  
  return scoredFormations;
}

double NodePrivate::computeFormationScore(RfGraph::Graph formation)
{
 return boost::num_vertices(formation); 
}


std::unordered_map< LocationId, std::list<LocationId>> 
  NodePrivate::minimumHopCountComRouteToTargetCell(NodeId const& source) const
{
  Q_ASSERT(dynamicRfVertex.count(source)==1);
  Q_ASSERT(dynamicRfVertex.at(source) != RfGraph::Graph::null_vertex());
  
  
  Q_Q( const Node ); Q_ASSERT(q);
  if( isRfGraphValid == false )
  { 
    qCritical() << q->qualifiedName() << ": R/F graph isn't valid, cannot compute route.";
    return std::unordered_map< LocationId, std::list<LocationId>> ();
  }
  
  using namespace boost;    
  using RfVertex = RfGraph::Vertex_Descriptor;
  
   
  //
  // Get the intersection points of the target cell
  // ==============================================
  // Find the intersection points (bifurcation nodes) of the target cell
  // and get all the paths from there to the ownship.
  //

  // the list of RfVertices that are intersection points around the target
  std::list<RfVertex> targetCellBifurcationPoints;

  for( BifurcationGraph::Vertex_Descriptor const& biVertex 
        : obstacleBifurcationVertexData.at(ownshipBlip->targetId()) ) // cannot use operator[] for const correctnes
  {
    // check that the intersection vertices are within the COM corridor. They
    // can be outside for large obstacle cells that have an obstacle fairly 
    // offset from the cell's centroid.
    
    LocationId locId = get(vertex_locationId, bifurcationGraph, biVertex);
    if( geometry::covered_by(locationData.at(locId), comCorridor) )
    { 
      RfVertex rfVertex = locationId_to_rfVertex.at(locId);
      targetCellBifurcationPoints.push_back(rfVertex);
    }
  }
  
  qDebug() << q->qualifiedName() << ": Determined" << targetCellBifurcationPoints.size() 
    << "intersections around" << obstacleData.at(ownshipBlip->targetId()).objectName();
    
    
    
    
    
    
  //
  // Breadth first search through the visibility graph
  // =================================================
  // Prepare the predecessor lookup map to get the minimum-hop paths through 
  // the rfGraph.
  //

  RfVertex const & sourceVertex = dynamicRfVertex.at(source);
    
  std::map<RfVertex,RfVertex> predecessors;
  predecessors[sourceVertex] = sourceVertex;
    
  associative_property_map< std::map<RfVertex,RfVertex> > rfVertex_predecessor_map(predecessors);
  breadth_first_search(rfGraph, sourceVertex
    ,visitor(make_bfs_visitor(record_predecessors(rfVertex_predecessor_map, on_tree_edge())))
  );
  

  //
  // Extract the minimum hop paths
  // =============================
  //
  /** \todo If the overall target formation graph selection shall be based on 
   * some sort of metric, the same metric needs to be applied here when 
   * selecting the ingress part. */
  
  // the list of routes to get from the ownship RfVertex to the intersection 
  // points of the target cell.
  std::unordered_map< LocationId, std::list<LocationId>> comRoutesToTargetCell;  

  auto rfLocationIdMap = boost::get(boost::vertex_locationId, rfGraph);
//   auto rfLocationMap = boost::make_compose_property_map(location_pMap, rfLocationIdMap);
    
  for( RfVertex intersectionVertex : targetCellBifurcationPoints )
  {
    // the route to the current intersection vertex
    std::list<LocationId> minHopRoute;

    for( RfVertex v = intersectionVertex
      ; /* empty */ 
      ; v = predecessors[v])
    {
      LocationId locId = get(rfLocationIdMap,v);
      minHopRoute.push_front( locId ); // put in the front so the list is from ownship to intersection
      if( v == predecessors[v] )
      { break; }
    }    
    
    comRoutesToTargetCell[get(rfLocationIdMap, intersectionVertex)]
      = minHopRoute;
  }
  
  qDebug() << q->qualifiedName() << ": Computed" << comRoutesToTargetCell.size() 
    << "routes to the target cell.";


#if defined WITH_GRAPHICS && defined DEBUG_GRAPHICS
{ // DEBUG GRAPHICS
   
  
  delete ingressGraphics;
  ingressGraphics = new QGraphicsItemGroup(/*datumItem*/); // parent is set explicitely below
  
  for ( auto pair : comRoutesToTargetCell )
  {
    std::list<LocationId>& route = pair.second;
    
    int i = 0;
    Geometry::Point_2D lastPos;
    for( LocationId locId : route ) 
    {
      Geometry::Point_2D vPos;
      if( locId != NULL_LOCATION_ID )
      { vPos = locationData.at(locId); }
      else
      { vPos = blipData.at(source)->position2D(); }
      auto vGraphics = new Graphics_2D::SimpleVertexItem(vPos.x(),vPos.y(),2, ingressGraphics);
      vGraphics->setPen(Graphics_2D::Pens::Outline(Qt::red));
      
      if( i++ > 0 )
      {
        auto line = new QGraphicsLineItem(vPos.x(),vPos.y(), lastPos.x(), lastPos.y(), ingressGraphics );
        line->setPen(Graphics_2D::Pens::DashedOutline(Qt::red));
      }
      lastPos = vPos;
    }
  }
  
  ingressGraphics->setZValue(NodePrivate::IngressGraphicZLevel);
  ingressGraphics->setParentItem(datumItem);
  scene->update();
  qApp->processEvents();
}
#endif // WITH_GRAPHICS && DEBUG_GRAPHICS
  
  return comRoutesToTargetCell;

};


QList<LocationId> NodePrivate::getLocations(Geometry::Polygon_2D const& region)
{
  QList<LocationId> coveredLocations;
    
  for( auto& pair  : locationData )
  {
    LocationId id = pair.first;
    Location loc = pair.second;
    
    if( boost::geometry::covered_by(loc,region) ) 
    { coveredLocations.append(id); }
  }
   
  return coveredLocations;
};

QList<LocationId> NodePrivate::getRouteLocations(Geometry::Polygon_2D const& region)
{
  QList<LocationId> locations = getLocations(region);
  for( QList<LocationId>::iterator it = locations.begin();
       it!=locations.end();
       /* empty */ )
  {
    Location const& loc = locationData[*it];
    if( loc.isRouteGraphVertexValid() )
    { ++it; }
    else
    { it = locations.erase(it); }
  }
  return locations;
};

QList<LocationId> NodePrivate::getBifurcationLocations(Geometry::Polygon_2D const& region)
{
  QList<LocationId> locations = getLocations(region);
  for( QList<LocationId>::iterator it = locations.begin();
       it!=locations.end();
       /* empty */ )
  {
    Location const& loc = locationData[*it];
    if( loc.isBifurcationGraphVertexValid() )
    { ++it; }
    else
    { it = locations.erase(it); }
  }
  return locations;
};

QList<LocationId> NodePrivate::getObstacleLocations(Geometry::Polygon_2D const& region)
{
  QList<LocationId> locations = getLocations(region);
  for( QList<LocationId>::iterator it = locations.begin();
       it!=locations.end();
       /* empty */ )
  {
    Location const& loc = locationData[*it];
    if( loc.isObstacleGraphVertexValid() )
    { ++it; }
    else
    { it = locations.erase(it); }
  }
  return locations;
};




void NodePrivate::computeRfGraph( Geometry::Polygon_2D const& environment )
{
  
  rfGraph.clear();
  locationId_to_rfVertex.clear();
  
  //
  // Add vertices to the graph 
  // =============================
  // This would be all the route graph vertices inside the environment plus an
  // extra vertex for the ownship.
  //
  {
    using namespace boost;
    
    unsigned int vIndex = 0;

    //
    // dynamic vertices (i.e. for Blips)
    //
    
    for( auto& pair : dynamicRfVertex )
    {
      NodeId id = pair.first;
      RfGraph::Vertex_Descriptor& v = pair.second;
      
      Q_ASSERT( blipExists(id) );
      
      // check a precondition
      if( boost::geometry::covered_by(blipData[id]->position2D(),environment) )
      {
        // dynamic vertex (i.e. for a Blip)
        v = add_vertex(rfGraph);
        put(vertex_index, rfGraph, v, vIndex++);
        put(vertex_locationId, rfGraph, v, NULL_LOCATION_ID); //TODO: Blips don't have locationIds.... :/   
        put(vertex_id, rfGraph, v, id);
        
        locationId_to_rfVertex[NULL_LOCATION_ID] = v;
      }
    }
    
    // TODO: As the COM formation stuff relies on the LocationId being a unique 
    // identifier for a rfGraph vertex, the current implementation using 
    // NULL_LOCATION_ID for dynamic vertices only works with a maximum of 1
    // dynamic vertex!
    Q_ASSERT(dynamicRfVertex.size() <=1 ); 
    
    //
    // static vertices (i.e. for locations)
    //
    
    RfGraph::Vertex_Descriptor v;
    
    QList<LocationId> coveredLocations = getRouteLocations(environment);
    for( LocationId id : coveredLocations )
    {
      // routeGraph vertex
      v = add_vertex(rfGraph);
      put(vertex_index, rfGraph, v, vIndex++);
      put(vertex_locationId, rfGraph, v, id);
      put(vertex_id, rfGraph, v, NULL_NODE_ID);
      
      locationId_to_rfVertex[id]=v;
    }    
  }
  
  
//   if(false) //DEBUG
//   {
//     using namespace boost;
//     RfGraph::Vertex_Iterator vi, vi_end;
//     for( tie(vi,vi_end)=vertices(rfGraph); vi != vi_end; ++vi)
//     {
//       static int i =0;
//      qDebug() << "Vertex" << i++ 
//       << "{Index =" << get(vertex_index, rfGraph, *vi)
//       << ", LocId =" << get(vertex_locationId, rfGraph, *vi)
//       << ", NodeId =" << get(vertex_id, rfGraph, *vi) 
//       <<"}";
//     }
//     
//     for( auto pair : dynamicRfVertex )
//     {
//       static int i=0;
//       qDebug() << "Dynmic Vertex" << i++ 
//         << "{Key =" << pair.first 
//         << ", NodeId =" << boost::get(boost::vertex_id, rfGraph, pair.second)
//         << "}";
//     }
//   }
    
  

  auto locationIdMap = boost::get(boost::vertex_locationId, rfGraph);
  auto locationMap = boost::make_compose_property_map(location_pMap, locationIdMap);
  
  //
  // Build the graph based on range limited visibility
  // =================================================
  // Check for range limited visibility amongst all vertices and connect the 
  // vertices that can see each other.
  //
  {
    using namespace boost;
    
    unsigned int edgeIndex = 0;
    
    VisiLibity::Environment visEnv = VisilibityInterface::createEnvironment(environment);
    visEnv.enforce_standard_form();  
    Q_ASSERT( visEnv.is_valid() ); //FIXME: This should not create any errors.... :/
    
    // the added vertices for the dynamic vertices isn't a location and has been
    // given the NULL_LOCATION_ID. Hence we need to circumvent accessing 
    // the Location database in order to cheat a little bit...
    // HACK: This clearly doesn't help when plotting this graph... :/
    auto vertexPosition = [&](RfGraph::Vertex_Descriptor v)->Geometry::Point_2D
    {
      Location loc = locationMap[v];
      if( loc.id() == NULL_LOCATION_ID )
      {
        Blip const* blip = blipData.at(get(vertex_id, rfGraph, v));
        return blip->position2D();  
      }
      return loc;
    };
        
    
    RfGraph::Vertex_Iterator vOuter, vOuter_end;
    tie(vOuter, vOuter_end) = vertices(rfGraph);
    
    RfGraph::Vertex_Iterator const vFirst = vOuter;
    
    // NOTE: this starts at the 'second' vertex as it is sufficient to check 
    // only a diagonal matrix of vertices (the graph is undirected)
    for( vOuter++ ; vOuter != vOuter_end; ++vOuter)
    {
      // get the assoc. visible area of the outer vertex
      
      VisiLibity::Point vOuterPos = vertexPosition( *vOuter );
      VisiLibity::Visibility_Polygon visPoly(vOuterPos,visEnv,VisilibityInterface::epsilon); 
      auto visibleArea = VisilibityInterface::makeRing<Geometry::Polygon_2D::ring_type>(visPoly);
      
      
      RfGraph::Vertex_Iterator vInner;
      for( vInner = vFirst; vInner != vOuter; vInner++ )
      {   
//         Location vInnerLoc = locationMap[*vInner];


        Geometry::Point_2D vInnerPos = vertexPosition(*vInner);
        // check distance as a "cheap" inital check so that we can skip the 
        // computation of the deltaDisc and the clipping of the visibleArea
        
        auto edgeLength = distance(vOuterPos, vInnerPos);
        
        auto rfZone = correspondingRfZone( edgeLength );  
        if( rfZone == RfZone::White ) 
        { continue; } // distance is outside R/F limits, i.e. not in range   
       
        // if that clears, check for inclusion by the visible area.
        // "covered_by" is used (as opposed to "within") to include the boundary
        // of the area as some route graph vertices will be located there
      
        if( geometry::covered_by(vInnerPos, visibleArea) )
        {
            RfGraph::Edge_Descriptor edge;
            bool edgeIsNew;
            tie(edge, edgeIsNew) = add_edge(/*from/source*/*vOuter, /*to/target*/*vInner, rfGraph);
            put(edge_index, rfGraph, edge, edgeIndex++);
            
            put(edge_quality, rfGraph, edge, static_cast<uint>(rfZone));
            put(edge_weight, rfGraph, edge, edgeLength);
        }
      }
    }
  }

  
  //
  // Create (DEBUG) graphics for the R/F graph
  // =========================================
  // The problem here is the ownship as it's location is stored inside the graph 
  // and not accessible via a LocationID.  
#if defined WITH_GRAPHICS && defined DEBUG_GRAPHICS
{
    using namespace boost;
  
    // building a position map that covers all vertices    
    std::vector<Geometry::Point_2D> pointData(num_vertices(rfGraph));
    
    RfGraph::Vertex_Index_Map vIndexMap = get(vertex_index, rfGraph);
    
    RfGraph::Vertex_Iterator vi, vi_end;
    for( tie(vi,vi_end)=vertices(rfGraph); vi!=vi_end; ++vi )
    {
      unsigned int vIndex = vIndexMap[*vi];
      
      if( vIndex == 0 ) // the ownship
      { pointData[0] = ownshipBlip->position2D(); }
      else // a route graph vertex
      { pointData[vIndex] = locationMap[*vi]; }         
    }
      
    auto positionMap = make_iterator_property_map(pointData.begin(),vIndexMap);
 
    { 
      Q_Q(Node); Q_ASSERT(q);
      
      bool visible = false;
      if( rfGraphGraphics != nullptr )
      { visible = rfGraphGraphics->isVisible(); }

      delete rfGraphGraphics;  
      
      rfGraphGraphics = new Graphics_2D::GraphItem(q,datumItem);
      rfGraphGraphics->generate(rfGraph, positionMap);
      rfGraphGraphics->setEdgePen(Graphics_2D::Pens::Outline(Qt::yellow));
//       rfGraphGraphics->setVertexDiameter(1.0);
      rfGraphGraphics->setVertexPen(Graphics_2D::Pens::Outline(Qt::darkGreen));
      rfGraphGraphics->setZValue(NodePrivate::RfGraphGraphicZLevel);
      rfGraphGraphics->setVisible(visible);
    }

}
#endif // WITH_GRAPHICS && DEBUG_GRAPHICS
  
  isRfGraphValid = true;
};

void NodePrivate::initializePropertyMapInterfaces()
{
  location_pMap = boost::make_assoc_property_map(locationData);
  blip_pMap     = boost::make_assoc_property_map(blipData);
  obstacle_pMap = boost::make_assoc_property_map(obstacleData);
}

void NodePrivate::computeComIngressToTargetCell(PathComputationData* data) const
{ 
  data->mutex.lock();
    NodeId source = data->source;
  data->mutex.unlock();
  
  PathComputationData::IngressMap ingress = 
    minimumHopCountComRouteToTargetCell(source);
    
  data->mutex.lock();
    data->ingress = ingress;
  data->mutex.unlock(); 
}


bool NodePrivate::proposeComFormation(const ElementId& target, const NodeId& source)
{
  if( computingComFormationProposal or comFormationData != nullptr )
  {
    Q_Q(Node); Q_ASSERT(q);
    qWarning() << q->qualifiedName() << ": An old COM formation process didn't quit cleanly. Skipping this process.";
    return false;
  }
  
  computingComFormationProposal = true;
  comFormationData = new PathComputationData;
  
  comFormationData->mutex.lock();
    comFormationData->target = target;
    comFormationData->source = source;
    Node::connect(&(comFormationData->watcher), &QFutureWatcher<void>::finished
           , [=](){processComFormationProposal(comFormationData);});
  comFormationData->mutex.unlock();
  
  dynamicRfVertex.clear();
  dynamicRfVertex[source] = RfGraph::Graph::null_vertex();
  
  computeRfGraph(comCorridor);  
  
  computeComIngressToTargetCell(comFormationData);
  
  
  /** \todo Adapt the max hop count available to build loops wrt. the numbers of
   * available (or active) nodes as after the computation of the ingress, the 
   * max. number of UAVs that are left to be used is known at this point.
   * This information could be used to make a feasible/infeasible decision. */
  
  /** \todo FIXME: when the target is unset before the formation graph is 
   * computed, the worker thread isn't canceled and setting a new target 
   * creates a segfault... :/ */
   
  QFuture< void > future = 
    QtConcurrent::run<void>(this, &NodePrivate::computeComLoopsAroundTargetObstacle
                           ,comFormationData );
  
  comFormationData->mutex.lock();  
    comFormationData->watcher.setFuture(future);  
  comFormationData->mutex.unlock();
   
  return true;
}

bool NodePrivate::processComFormationProposal(PathComputationData*& data)
{
  Q_Q(Node); Q_ASSERT(q);
  
  qDebug() << q->qualifiedName() << ": processing COM formation.";
  
  data->mutex.lock();
    ElementId target = data->target;
    NodeId source = data->source;
    PathComputationData::IngressMap ingress = data->ingress;
    PathComputationData::LoopMap loops = data->loops;
  data->mutex.unlock();
 
  
  if( target != q->targetId() )
  {
    delete data;
    data=nullptr;

    computingComFormationProposal = false;
    
    return false;
  }
  
  
  
  
  std::multimap<double, RfGraph::Graph> scoredFormations =
    combineIngressAndLoops(ingress, loops);
      
  auto proposedFormation = *scoredFormations.begin();
  
  formationGraph = proposedFormation.second;
  isFormationGraphValid = true;

  emit q->comFormationProposed(data->source,data->target);
  


#if defined WITH_GRAPHICS && defined DEBUG_GRAPHICS
{
  using namespace boost;
  
  auto locationIdMap = get(vertex_locationId, data->directedRfGraph);
  auto locationMap = make_compose_property_map(location_pMap, locationIdMap);
  
  Q_Q(Node); Q_ASSERT(q);
  
  bool visible = false;
  if( comLoopGraphGraphics != nullptr )
  { visible = comLoopGraphGraphics->isVisible(); }
  
  delete comLoopGraphGraphics;  
  
  comLoopGraphGraphics = new Graphics_2D::GraphItem(q,datumItem);
  comLoopGraphGraphics->generate(data->directedRfGraph, locationMap);
  comLoopGraphGraphics->setEdgePen(Graphics_2D::Pens::Outline(Qt::darkYellow));
//   comLoopGraphGraphics->setVertexDiameter(1.0);
  comLoopGraphGraphics->setVertexPen(Graphics_2D::Pens::Outline(Qt::darkGreen));
  comLoopGraphGraphics->setZValue(NodePrivate::ComLoopGraphGraphicZLevel);
  comLoopGraphGraphics->setVisible(visible);
  
}
#endif // DEBUG_GRAPHICS  
  
#ifdef WITH_GRAPHICS
{ // draw the proposed Formation
    
  using namespace boost;

  auto locationIdMap = get(vertex_locationId, formationGraph);
  auto locationMap = make_compose_property_map(location_pMap, locationIdMap);
  
  // 
  // building a position map that covers all vertices    
  //  
  
  std::vector<Geometry::Point_2D> pointData(num_vertices(formationGraph));
  
  RfGraph::Vertex_Index_Map vIndexMap = get(vertex_index, formationGraph);
  RfGraph::Vertex_Index_Map::value_type vIndex = 1;
  
  RfGraph::Vertex_Iterator vi, vi_end;
  for( tie(vi,vi_end)=vertices(formationGraph); vi!=vi_end; ++vi )
  {
    unsigned int vLocId = locationIdMap[*vi];
    
    if( vLocId == NULL_LOCATION_ID ) // the ownship
    { 
      vIndexMap[*vi] = 0;
      pointData[0] = blipData[source]->position2D();
    }
    else // a route graph vertex
    { 
      vIndexMap[*vi] = vIndex;
      pointData[vIndex] = locationMap[*vi]; 
      ++vIndex;
    }         
  }

#ifdef DEBUG_GRAPHICS
  /** \internal Delete/hide the ingress graphics once the formation graph is 
   * ready. This is simply done to remove visual clutter. */
  ingressGraphics->hide();
#endif // DEBUG_GRAPHICS
  
  
  using PosMap_t = boost::iterator_property_map<std::vector<Geometry::Point_2D>::iterator, RfGraph::Vertex_Index_Map>;
  PosMap_t positionMap = make_iterator_property_map(pointData.begin(),vIndexMap);
  
  
  {
    bool visible = false;
    if( formationGraphGraphics != nullptr )
    { visible = formationGraphGraphics->isVisible(); }
        
    QPen edgePen = QPen(Graphics_2D::Brushes::FloodFill(Qt::cyan,30)
      ,4,Qt::SolidLine,Qt::RoundCap); //FIXME: Magic Numbers: thickness of formation graph line.
    
    
    delete formationGraphGraphics; 

    formationGraphGraphics = new Graphics_2D::GraphItem(q,datumItem);
    formationGraphGraphics->generate(formationGraph, positionMap);
    formationGraphGraphics->setEdgePen(edgePen);
    formationGraphGraphics->setVertexPen(Graphics_2D::Pens::Outline(Qt::darkBlue));
    formationGraphGraphics->setZValue(NodePrivate::FormationGraphGraphicZLevel);
    formationGraphGraphics->setVisible(visible);

    
    // Actually, we just computed it, so we might as well look at it.
    q->showFormationGraph();
  }   
  
}
#endif // WITH_GRAPHICS  



  delete data;
  data=nullptr;

  computingComFormationProposal = false;
  
  return true;
}



void NodePrivate::unsetComFormation()
{
  Q_Q(Node); Q_ASSERT(q);
  isFormationGraphValid = false;
  
#ifdef WITH_GRAPHICS
  delete comLoopGraphGraphics;
  comLoopGraphGraphics = nullptr;

  delete ingressGraphics;
  ingressGraphics = nullptr;
  q->hideFormationGraph(); 
  delete formationGraphGraphics;
  formationGraphGraphics = nullptr;

#ifdef DEBUG_GRAPHICS
  delete rfGraphGraphics;
  rfGraphGraphics = nullptr;
#endif // DEBUG_GRAPHICS  

#endif // WITH_GRAPHICS
}


std::list<Blip*> NodePrivate::getBlipsByMode(IDL::Data::StateDescriptorId mode) const
{
  std::list<Blip*> list;

  for( std::pair<NodeId,Blip*> pair : blipData )
  {
    Blip* blip = pair.second;
    if( blip->mode().contains(mode) )
    { list.push_back(blip); }
  }
 
 return list;
}


//############################################################################//
//                                                                            //
//  NodePrivate::OwnshipBlip                                                  //
//                                                                            //
//############################################################################//

void NodePrivate::OwnshipBlip::setPosition3D ( const AMG::Vector& position )
{
  AMG::CoordinateTupel coordinates = position.absoluteCoordsIn(AMG::CoSy::getDatumFrame());

  // update the State topic message data
  parentNodePrivate->stateMessage->position.x = coordinates(0);
  parentNodePrivate->stateMessage->position.y = coordinates(1);
  parentNodePrivate->stateMessage->position.z = coordinates(2);
    
  // call the corresponding Blip function to take care of Blip-related business
  Blip::setPosition3D(position);
  
};

void NodePrivate::OwnshipBlip::setAttitude ( const AMG::EulerAngleTupel& eulerAngles )
{ 
  // update the State topic message data
  parentNodePrivate->stateMessage->attitude.phi   = eulerAngles[0];
  parentNodePrivate->stateMessage->attitude.theta = eulerAngles[1];
  parentNodePrivate->stateMessage->attitude.psi   = eulerAngles[2];
  
  // call the corresponding Blip function to take care of Blip-related business
  Blip::setAttitude(eulerAngles);
};

void NodePrivate::OwnshipBlip::setVelocity ( const AMG::Vector& velocity )
{
  AMG::CoordinateTupel speeds = velocity.coordsIn(AMG::CoSy::getDatumFrame());
  
  // update the State topic message data
  parentNodePrivate->stateMessage->velocity.x_dot = speeds(0);
  parentNodePrivate->stateMessage->velocity.y_dot = speeds(1);
  parentNodePrivate->stateMessage->velocity.z_dot = speeds(2);
    
  // call the corresponding Blip function to take care of Blip-related business
  Blip::setVelocity(velocity);
};

bool NodePrivate::OwnshipBlip::setFlightplan ( IDL::Data::Flightplan const & idlPlan)
{
  // update the Flightplan topic message data
  parentNodePrivate->flightplanMessage->plan = idlPlan;
  parentNodePrivate->flightplanMessage->currentWaypoint = 0;
  
  // call the corresponding Blip function to take care of Blip-related business
  if( Blip::setFlightplan(idlPlan) )
  {    
    // send the related flightplanMessage
    parentNodePrivate->flightplanTopic->send();
    return true;
  }
  else
  { return false; }
}

bool NodePrivate::OwnshipBlip::setCurrentWaypointIndex(unsigned int const & index)
{
  // update the Flightplan topic message data
  parentNodePrivate->flightplanMessage->currentWaypoint = index;
  
  // call the corresponding Blip function to take care of Blip-related business
  if( Blip::setCurrentWaypointIndex(index) )
  {
    // send the related flightplanMessage
    parentNodePrivate->flightplanTopic->send();
    return true;
  }
  else
  { return false; }
}


void NodePrivate::OwnshipBlip::clearFlightplan()
{
  qDebug() << parent()->qualifiedName() << ": Clearing Flightplan for ownship.";  
  setFlightplan(IDL::Data::Flightplan()); 
}


void NodePrivate::OwnshipBlip::setTargetId(ElementId const & id)
{
  bool targetWillChange = id!=targetId();
  Blip::setTargetId(id);
  if( targetWillChange )
  { emit parentNodePrivate->q_func()->targetChanged(id); }
}

#ifdef WITH_GRAPHICS
void NodePrivate::OwnshipBlip::setDatumItem(QGraphicsItem* const datumItem)
{
  Blip::setDatumItem(datumItem); 
  
  Q_ASSERT(d_ptr); //HACK: well, technically we shouldn't have access to BlipPrivate... but d_prt is only 'protected' so we can cheat.
  d_ptr->rfPolygonItem->setZValue(GraphicItemZLevels::OwnshipBlipRfPolygonZLevel);
  d_ptr->rfPolygonCenterItem->setZValue(GraphicItemZLevels::OwnshipBlipRfPolygonCenterZLevel);
}
#endif // WITH_GRAPHCIS

// RfGraph::Vertex_Descriptor NodePrivate::OwnshipBlip::rfGraphVertex() const
// {
//   return rfVertex;
// }

// void NodePrivate::OwnshipBlip::setRfGraphVertex(RfGraph::Vertex_Descriptor const & descriptor)
// {
//   rfVertex = descriptor;
// }





//############################################################################//
//                                                                            //
//  NodePrivate::Pallets                                                      //
//                                                                            //
//############################################################################//

#ifdef WITH_GRAPHICS
NodePrivate::Palette::Palette()
{
  using namespace Graphics_2D;
  
  blip.fovPoly_joystick.brush = Qt::NoBrush;
  blip.fovPoly_joystick.pen = Pens::Outline(Qt::black);
  
  blip.fovPoly_nonJoystick.brush = Qt::NoBrush;
  blip.fovPoly_nonJoystick.pen = Pens::Outline(Qt::lightGray);

  /** \internal The possible "states" for a palette should be the same as (or a 
   * subset of) the reportedStates of a Node. That way everything that is 
   * reported can/could be represented in graphics.
   * 
   * \note This constructor has some involved interplay with 
   * the private (Node/Vehicle) variable \c reportedStates as well as with 
   * \c statesToDraw in BlipPrivate::updatePalette()
   *
   * \internal Blip::updatePalette() is the method that actually
   * makes use of these pallets, i.e. that function can overrule any pallett 
   * setting.
   */
  

  Utensils relayBlip_rfPoly_A;
    relayBlip_rfPoly_A.brush = Brushes::FloodFill(Qt::darkGreen,30);
    relayBlip_rfPoly_A.pen   = Qt::NoPen;

  Utensils relayBlip_rfPoly_B;
    relayBlip_rfPoly_B.brush = Brushes::FloodFill(QColor(245,245,245),30);
    relayBlip_rfPoly_B.pen   = Qt::NoPen;
    
  Utensils relayBlip_rfPoly_C;
//     relayBlip_rfPoly_C.brush = Brushes::FloodFill(Qt::green,30);
    relayBlip_rfPoly_C.brush = Brushes::FloodFill(Qt::darkGreen,30);
    relayBlip_rfPoly_C.pen   = Qt::NoPen;

    
  Utensils nonRelayBlip_rfPoly_A;
    nonRelayBlip_rfPoly_A.brush = Qt::NoBrush;
    nonRelayBlip_rfPoly_A.pen   = Pens::Outline(Qt::darkGreen);

  Utensils nonRelayBlip_rfPoly_B;
    nonRelayBlip_rfPoly_B.brush = Qt::NoBrush;
    nonRelayBlip_rfPoly_B.pen   = Pens::Outline(Qt::lightGray);

  Utensils nonRelayBlip_rfPoly_C;
    nonRelayBlip_rfPoly_C.brush = Qt::NoBrush;
    nonRelayBlip_rfPoly_C.pen   = Pens::Outline(Qt::green);

    
    
  Utensils blip_rfPoly_NoLink;
    blip_rfPoly_NoLink.brush = Qt::NoBrush;
    blip_rfPoly_NoLink.pen   = Qt::NoPen;
  
 /* 
  * reported states for the Datalink
  */
    
  blip.rfPoly_relay[IDL::Data::STATE_ID_D_NO_LINK]    = blip_rfPoly_NoLink;

  blip.rfPoly_nonRelay[IDL::Data::STATE_ID_D_NO_LINK] = blip_rfPoly_NoLink;

  /* 
   * reported states of a Node (see NodePrivate constructor) 
   */   
  
  blip.rfPoly_relay[IDL::Data::STATE_ID_N_STANDBY]     = relayBlip_rfPoly_B;
  blip.rfPoly_relay[IDL::Data::STATE_ID_N_JOINING]     = relayBlip_rfPoly_A;
  blip.rfPoly_relay[IDL::Data::STATE_ID_N_LEAVING]     = relayBlip_rfPoly_A;
  blip.rfPoly_relay[IDL::Data::STATE_ID_N_OPERATIONAL] = relayBlip_rfPoly_A;
  
  blip.rfPoly_nonRelay[IDL::Data::STATE_ID_N_STANDBY]     = nonRelayBlip_rfPoly_B;
  blip.rfPoly_nonRelay[IDL::Data::STATE_ID_N_JOINING]     = nonRelayBlip_rfPoly_A;
  blip.rfPoly_nonRelay[IDL::Data::STATE_ID_N_LEAVING]     = nonRelayBlip_rfPoly_A;
  blip.rfPoly_nonRelay[IDL::Data::STATE_ID_N_OPERATIONAL] = nonRelayBlip_rfPoly_A;



  /* 
   * reported states of a Vehicle (see VehiclePrivate constructor) 
   */
  
//   blip.rfPoly_relay[IDL::Data::STATE_ID_V_AUTONOMOUS] = relayBlip_rfPoly_relayA;
  blip.rfPoly_relay[IDL::Data::STATE_ID_V_PILOTED]    = relayBlip_rfPoly_A;
  blip.rfPoly_relay[IDL::Data::STATE_ID_V_JOYSTICK]   = relayBlip_rfPoly_C; //NOTE: "Joystick" additionally has FOV enabled and uses fovPoly_joystick.
  
//   blip.rfPoly_nonRelay[IDL::Data::STATE_ID_V_AUTONOMOUS]  = relayBlip_rfPoly_nonRelayA;
  blip.rfPoly_nonRelay[IDL::Data::STATE_ID_V_PILOTED]     = nonRelayBlip_rfPoly_A;  
  blip.rfPoly_nonRelay[IDL::Data::STATE_ID_V_JOYSTICK]    = nonRelayBlip_rfPoly_C; //NOTE: "Joystick" additionally has FOV enabled and uses fovPoly_joystick.
  

  /* 
   * ownshipBlip
   */
    
  
  Utensils ownshipBlip_rfPolyDefault;
  ownshipBlip_rfPolyDefault.brush = Qt::NoBrush; 
  ownshipBlip_rfPolyDefault.pen   = Pens::Outline(Qt::darkBlue);
  
  Utensils ownshipBlip_rfPolyNoLink;
  ownshipBlip_rfPolyNoLink.brush = Qt::NoBrush; 
  ownshipBlip_rfPolyNoLink.pen   = Pens::Outline(Qt::darkRed);
  
  blip.rfPoly_ownship[IDL::Data::STATE_ID_D_NO_LINK]      = ownshipBlip_rfPolyNoLink;
  
  blip.rfPoly_ownship[IDL::Data::STATE_ID_N_STANDBY]      = ownshipBlip_rfPolyDefault;
  blip.rfPoly_ownship[IDL::Data::STATE_ID_N_JOINING]      = ownshipBlip_rfPolyDefault;
  blip.rfPoly_ownship[IDL::Data::STATE_ID_N_LEAVING]      = ownshipBlip_rfPolyDefault;
  blip.rfPoly_ownship[IDL::Data::STATE_ID_N_OPERATIONAL]  = ownshipBlip_rfPolyDefault;

  //   blip.rfPoly_ownship[IDL::Data::STATE_ID_V_AUTONOMOUS]   = ownshipBlip_rfPolyDefault;
  blip.rfPoly_ownship[IDL::Data::STATE_ID_V_PILOTED]      = ownshipBlip_rfPolyDefault;
  blip.rfPoly_ownship[IDL::Data::STATE_ID_V_JOYSTICK]     = ownshipBlip_rfPolyDefault; //NOTE: "Joystick" additionally has FOV enabled and uses fovPoly_joystick.
     
}
#endif // WITH_GRAPHICS



//############################################################################//
//                                                                            //
//  Node                                                                      //
//                                                                            //
//############################################################################//

Node::Type Node::type() const
{ return IDL::Data::NODE_TYPE_BASIC; }

/** \note This constructor does NOT do any work!
 * All Node level construction needs to happen in  
 * Node(NodePrivate& dd, ...)!
 */
Node::Node(Node::Settings* const settings
           /*,SimItem * parent */)
  :Node(*new NodePrivate(this,settings/*,parent*/)/*,parent*/)
{}

Node::Node(NodePrivate& dd
           /*,SimItem * parent */)
  : SimItem(dd, /*parent=*/nullptr) // Nodes don't have parents
{
  Q_D(Node); Q_ASSERT(d);
  
  if( d->xmlSettings->dds().present() )
  { d->dds = new Datalink(&(d->xmlSettings->dds().get()), this); }
  else
  {
    QString ddsName = tr("Datalink"); // FIXME: Magic Numbers
    d->ddsSettings = new Datalink::Settings( ddsName.toStdString().c_str());
    d->dds = new Datalink(d->ddsSettings,this);
  }
  
  /** \internal 
   * This will only indicate the status of the Node, any derived classes
   * will not be constructed at this point!
   */
  d->setStatus(Status::Constructed); 
  
  connect(this, &Node::comGraphChanged,
          [d](){d->updateRfConnectedArea();} );
}

Node::~Node()
{
//   qDebug() << qualifiedName() << ": ~Node()";
  Q_D(Node); Q_ASSERT(d);
  d->unInitializeNode();
  
  // dds deosn't need to be deleted, as there is a parent/child relationship
  // that takes care of that
    
  delete d->ddsSettings; // this might delete a nullptr, but that's ok...
  d->ddsSettings = nullptr;
}

const Node::Settings* Node::settings() const
{
  Q_D(const Node); Q_ASSERT(d);
  return d->xmlSettings;
}

void Node::timerEvent(QTimerEvent* event)
{
  const int timerId = event->timerId();
  Q_D(Node); Q_ASSERT(d);
  
  if( timerId  == d->metronome[10] )
  { // 100 Hz stuff
//     d->randomWalk();
  }
  else if( timerId == d->metronome[100] )
  { // 10 Hz stuff
    d->stateTopic->send();
  }
  else if( timerId == d->metronome[1000] )
  { // 1 Hz stuff    
    d->updateStatusMessage();
    d->statusTopic->send();
  }
}

QMutex* Node::mutex()
{
  Q_D(Node); Q_ASSERT(d);
  return &(d->mutex);
}

void Node::initializeItem()
{
  SimItem::initializeItem();
  Q_D(Node); Q_ASSERT(d);
   
#ifdef WITH_GRAPHICS
  d->initializeGraphics();
#endif // WITH_GRAPHICS
  d->initializeDatalink(); //NOTE: has to be done before any usage of Node::id()
  d->initializePropertyMapInterfaces();
  d->initializeRouteGraph();
  d->initializeDerivedGraphs();
//   d->initializeVisibilityGraph();
  d->initializeOwnshipBlip();
  d->initializePosition();
  d->initializeStates();
//   d->initializeComGraph();
#ifdef WITH_GRAPHICS
  d->initializeModels();
#endif // WITH_GRAPHICS
  
  { /// Register some basic metronome timers
    metronome(10);    //   10 ms period -> 100 Hz
    metronome(100);   //  100 ms period ->  10 Hz
    metronome(1000);  // 1000 ms period ->   1 Hz
  }
    
}

void Node::startItem()
{
  Q_D(Node); Q_ASSERT(d);
  SimItem::startItem();
  
  d->dds->run();
  
  { /// Start all metronome timers
    QHash<unsigned int, int>::iterator it;
    for( it = d->metronome.begin(); it != d->metronome.end(); ++it )
    {
      it.value() = startTimer(it.key());
    }
  }
  
}

void Node::stopItem()
{
  Q_D(Node); Q_ASSERT(d);
  
  
  { /// Kill all metronome timers
    QHash<unsigned int, int>::iterator it;
    for( it = d->metronome.begin(); it != d->metronome.end(); ++it )
    {
      killTimer(it.value());
      it.value() = 0;
    }
  }
  
  d->dds->halt();
  
  SimItem::stopItem();
}

void Node::unInitializeItem()
{
//   qDebug() << qualifiedName() << ": Node::unInitializeItem()";
  Q_D(Node); Q_ASSERT(d);
  d->unInitializeNode();
  SimItem::unInitializeItem();
}

NodeId Node::id() const
{
  Q_D(const Node); Q_ASSERT(d);
  return d->ownshipId;
}

ElementId Node::targetId() const
{
  return ownship().targetId();
}


void Node::setTargetId(const ElementId& id)
{
  if( id == ownship().targetId() )
  { return; }
  
  Q_D(Node); Q_ASSERT(d);  
  
  unsetTarget();
 
  if( !d->obstacleGraphIsCurrent )
  { d->updateDerivedGraphs(); } 
 
  ownship().setTargetId(id);
  
  d->createTargetEnvironment(id);
  
#ifdef WITH_GRAPHICS
  // update the graphical represenation for all obstacles as a result from the 
  // target (un-)selection.
  for( auto& pair : d->obstacleData )
  {
    if( pair.first == id )
    { pair.second.markAsTarget(); }
    else
    { pair.second.markAsNonTarget(); }
  }  
  showTargetEnvironment();
#endif // WITH_GRAPHICS  
}



void Node::unsetTarget()
{
  Q_D(Node); Q_ASSERT(d);
  
#ifdef WITH_GRAPHICS
  for( auto& pair : d->obstacleData )
  { pair.second.removeTargetMarker(); }
#endif // WITH_GRAPHICS

  ownship().setTargetId(NULL_ELEMENT_ID);

#ifdef WITH_GRAPHICS
  hideTargetEnvironment();
#endif // WITH_GRAPHICS
  
  d->unsetTargetEnvironment(); 
}


void Node::setObjectName ( const QString& name )
{
  if( isInitialized() )
  { 
    qCritical("\"%s\" : The name of an initialized Node cannot be changed.",
      qualifiedName().toStdString().c_str()
    );
    return;
  }
  
  return SimItem::setObjectName( name );
}

Blip const& Node::blip ( const NodeId& id ) const
{
  Q_D(const Node); Q_ASSERT(d);
  Q_ASSERT( d->blipExists(id) );
  return *(d->blipData.at(id));
}

Blip& Node::blip ( const NodeId& id )
{
  Q_D(Node); Q_ASSERT(d);
  Q_ASSERT( d->blipExists(id) );
  return *(d->blipData.at(id));
}

bool Node::blipExists ( const NodeId& id ) const
{
  Q_D(const Node); Q_ASSERT(d);
  return d->blipExists(id);
}


bool Node::obstacleExists(const ElementId& id) const
{
  Q_D(const Node); Q_ASSERT(d);
  
  if( d->obstacleData.count(id) > 0)
  { return true; }
  else
  { return false;}
}


const Obstacle& Node::obstacle(const ElementId& id) const
{
  Q_D(const Node); Q_ASSERT(d);
  return d->obstacleData.at(id);
}

Obstacle& Node::obstacle(const ElementId& id)
{
  Q_D(Node); Q_ASSERT(d);
  return d->obstacleData.at(id);
}



Blip& Node::ownship()
{
  return blip(id());
}

Blip const & Node::ownship() const
{
  return blip(id());
}


bool Node::updateRfPoly(NodeId const & id, double const & minDistance)
{
  Q_D( Node ); Q_ASSERT(d);
    
  if( blipExists(id) )
  { return d->updateRfPoly(d->blipData.at(id),minDistance); }
  else
  { 
    qWarning() << qualifiedName() << ": Cannot update the R/F polygon for a"
      << "nonexisting Blip. (NodeId:" << id << ")";
    return false;
  }
}

bool Node::updateFovPoly(NodeId const & id, double const & minDistance)
{
  Q_D( Node ); Q_ASSERT(d);
    
  if( blipExists(id) )
  { return d->updateFovPoly(d->blipData.at(id),minDistance); }
  else
  { 
    qWarning() << qualifiedName() << ": Cannot update the FOV polygon for a"
      << "nonexisting Blip. (NodeId:" << id << ")";
    return false;
  }
}



Blip* Node::addNewBlip( const NodeId& id, const Node::Type& type
                      , const QString& name, QStringList propertyList
                      )
{
  Q_D(Node); Q_ASSERT(d);
  
  if( blipExists(id) )
  { return &blip(id); }
    
  d->initializeBlip(id, type, name, propertyList); 
  emit blipAdded(id);
  
  return &blip(id);
}

void Node::deleteBlip ( const NodeId& id )
{
  Q_D(Node); Q_ASSERT(d);
  if(blipExists(id) )
  { 
    emit aboutToDeleteBlip(id); 
    d->deleteBlip(id);
#ifdef WITH_GRAPHICS
    {
      d->updateComGraphGraphics(); 
    }
#endif // WITH_GRAPHICS
  }
  else
  { qDebug() << qualifiedName() <<": Cannot delete nonexisting blip" << id; }
}

#ifdef WITH_GRAPHICS

bool Node::isComGraphVisible() const
{
  Q_D(const Node); Q_ASSERT(d);
  if( d->comGraphGraphics != nullptr )
  { return d->comGraphGraphics->isVisible(); }
  else
  { 
    qWarning() << qualifiedName() << ": trying to access non-existing graphic item.";
    return false;
  }
}

void Node::setComGraphVisibility(bool const & visibility)
{
  Q_D(Node); Q_ASSERT(d);
  if( d->comGraphGraphics != nullptr 
      and visibility != isComGraphVisible() )
  {
    d->comGraphGraphics->setVisible(visibility);
    emit comGraphVisibilityChanged(visibility);
  }
}

void Node::showComGraph()
{ setComGraphVisibility(true); }

void Node::hideComGraph()
{ setComGraphVisibility(false); }



bool Node::isRouteGraphVisible() const
{
  Q_D(const Node); Q_ASSERT(d);
  if( d->routeGraphGraphics != nullptr )
  { return d->routeGraphGraphics->isVisible(); }
  else
  { 
    qWarning() << qualifiedName() << ": trying to access non-existing graphic item.";
    return false;
  }
}

void Node::setRouteGraphVisibility(bool const & visibility)
{
  Q_D(Node); Q_ASSERT(d);
  if( d->routeGraphGraphics != nullptr 
      and visibility != isRouteGraphVisible() )
  {
    d->routeGraphGraphics->setVisible(visibility);
    emit routeGraphVisibilityChanged(visibility);
  }
}

void Node::showRouteGraph()
{ setRouteGraphVisibility(true); }

void Node::hideRouteGraph()
{ setRouteGraphVisibility(false); }




bool Node::isObstacleGraphVisible() const
{
  Q_D(const Node); Q_ASSERT(d);
  if(d->obstacleGraphGraphics != nullptr)
  { return d->obstacleGraphGraphics->isVisible(); }
  else
  { 
    qWarning() << qualifiedName() << ": trying to access non-existing graphic item.";
    return false;
  }
}

void Node::setObstacleGraphVisibility(bool const & visibility)
{
  Q_D(Node); Q_ASSERT(d);
  if( d->obstacleGraphGraphics != nullptr 
      and visibility != isObstacleGraphVisible() )
  {
    d->obstacleGraphGraphics->setVisible(visibility);
    emit comGraphVisibilityChanged(visibility);
  }
}

void Node::showObstacleGraph()
{ setObstacleGraphVisibility(true); }

void Node::hideObstacleGraph()
{ setObstacleGraphVisibility(false); }






bool Node::isFormationGraphVisible() const
{
  Q_D(const Node); Q_ASSERT(d);
  if(d->formationGraphGraphics!=nullptr)
  { return d->formationGraphGraphics->isVisible(); }
  else
  { 
    qWarning() << qualifiedName() << ": trying to access non-existing graphic item.";
    return false;
  }
}

void Node::setFormationGraphVisibility(bool const & visibility)
{
  Q_D(Node); Q_ASSERT(d);
  if( d->formationGraphGraphics != nullptr 
      and visibility != isFormationGraphVisible() )
  {
    d->formationGraphGraphics->setVisible(visibility);
    emit formationGraphVisibilityChanged(visibility);
  }
}

void Node::showFormationGraph()
{ setFormationGraphVisibility(true); }

void Node::hideFormationGraph()
{ setFormationGraphVisibility(false); }






bool Node::areObstacleCellsVisible() const
{
  Q_D(const Node); Q_ASSERT(d);
  if(d->obstacleCellGraphics!= nullptr)
  { return d->obstacleCellGraphics->isVisible(); }
  else
  { return false; }
}

void Node::setObstacleCellVisibility(bool const & visibility)
{
  Q_D(Node); Q_ASSERT(d);
  if( d->obstacleCellGraphics != nullptr 
      and visibility != areObstacleCellsVisible() )
  {
    d->obstacleCellGraphics->setVisible(visibility);
    emit obstacleCellVisibilityChanged(visibility);
  }
}

void Node::showObstacleCells()
{ setObstacleCellVisibility(true); }

void Node::hideObstacleCells()
{ setObstacleCellVisibility(false); }


bool Node::isTargetEnvironmentVisible() const
{
  Q_D(const Node); Q_ASSERT(d);
  if(d->targetEnvironmentGraphics != nullptr)
  { return d->targetEnvironmentGraphics->isVisible(); }
  else 
  { return false; }
}

void Node::setTargetEnvironmentVisibility(bool const & visibility)
{
  Q_D(Node); Q_ASSERT(d);
  if( d->targetEnvironmentGraphics != nullptr 
     and visibility != isTargetEnvironmentVisible() )
  {
    d->targetEnvironmentGraphics->setVisible(visibility);
    emit targetEnvironmentVisibilityChanged(visibility);
  }
}

void Node::showTargetEnvironment()
{ setTargetEnvironmentVisibility(true); }

void Node::hideTargetEnvironment()
{ setTargetEnvironmentVisibility(false); }


bool Node::isComCorridorVisible() const
{
  Q_D(const Node); Q_ASSERT(d);
  if(d->comCorridorGraphics != nullptr)
  { return d->comCorridorGraphics->isVisible(); }
  else 
  { return false; }
}

void Node::setComCorridorVisibility(bool const & visibility)
{
  Q_D(Node); Q_ASSERT(d);
  if( d->comCorridorGraphics != nullptr 
     and visibility != isComCorridorVisible() )
  {
    d->comCorridorGraphics->setVisible(visibility);
    emit comCorridorVisibilityChanged(visibility);
  }
}

void Node::showComCorridor()
{ setComCorridorVisibility(true); }

void Node::hideComCorridor()
{ setComCorridorVisibility(false); }


#if defined DEBUG_GRAPHICS

bool Node::isVroniGraphVisible() const
{
  Q_D(const Node); Q_ASSERT(d);
  if(d->vroniGraphGraphics != nullptr)
  { return d->vroniGraphGraphics->isVisible(); }
  else 
  { return false; }
}

void Node::setVroniGraphVisibility(bool const & visibility)
{
  Q_D(Node); Q_ASSERT(d);
  if( d->vroniGraphGraphics != nullptr 
     and visibility != isVroniGraphVisible() )
  {
    d->vroniGraphGraphics->setVisible(visibility);
    emit vroniGraphVisibilityChanged(visibility);
  }
}

void Node::showVroniGraph()
{ setVroniGraphVisibility(true); }

void Node::hideVroniGraph()
{ setVroniGraphVisibility(false); }



bool Node::isFaceGraphVisible() const
{
  Q_D(const Node); Q_ASSERT(d);
  if(d->faceGraphGraphics != nullptr)
  { return d->faceGraphGraphics->isVisible(); }
  else 
  { return false; }
}

void Node::setFaceGraphVisibility(bool const & visibility)
{
  Q_D(Node); Q_ASSERT(d);
  if( d->faceGraphGraphics != nullptr 
     and visibility != isFaceGraphVisible() )
  {
    d->faceGraphGraphics->setVisible(visibility);
    emit faceGraphVisibilityChanged(visibility);
  }
}

void Node::showFaceGraph()
{ setVroniGraphVisibility(true); }

void Node::hideFaceGraph()
{ setVroniGraphVisibility(false); }




bool Node::isFaceAssociationVisible() const
{
  Q_D(const Node); Q_ASSERT(d);
  if(d->faceToObstacleCorrespondence != nullptr)
  { return d->faceToObstacleCorrespondence->isVisible(); }
  else 
  { return false; }
}

void Node::setFaceAssociationVisibility(bool const & visibility)
{
  Q_D(Node); Q_ASSERT(d);
  if( d->faceToObstacleCorrespondence != nullptr 
     and visibility != isFaceAssociationVisible() )
  {
    d->faceToObstacleCorrespondence->setVisible(visibility);
    emit faceAssociationVisibilityChanged(visibility);
  }
}

void Node::showFaceAssociation()
{ setFaceAssociationVisibility(true); }

void Node::hideFaceAssociation()
{ setFaceAssociationVisibility(false); }



bool Node::isBifurcationAssociationVisible() const
{
  Q_D(const Node); Q_ASSERT(d);
  if(d->biEdgeAssociationGraphics != nullptr)
  { return d->biEdgeAssociationGraphics->isVisible(); }
  else 
  { return false; }
}

void Node::setBifurcationAssociationVisibility(bool const & visibility)
{
  Q_D(Node); Q_ASSERT(d);
  if( d->biEdgeAssociationGraphics != nullptr 
     and visibility != isBifurcationAssociationVisible() )
  {
    d->biEdgeAssociationGraphics->setVisible(visibility);
    emit faceAssociationVisibilityChanged(visibility);
  }
}

void Node::showBifurcationAssociation()
{ setBifurcationAssociationVisibility(true); }

void Node::hideBifurcationAssociation()
{ setBifurcationAssociationVisibility(false); }




bool Node::isRfGraphVisible() const
{
  Q_D(const Node); Q_ASSERT(d);
  if(d->rfGraphGraphics!=nullptr)
  { return d->rfGraphGraphics->isVisible(); }
  else
  { 
    qWarning() << qualifiedName() << ": trying to access non-existing graphic item.";
    return false;
  }
}

void Node::setRfGraphVisibility(bool const & visibility)
{
  Q_D(Node); Q_ASSERT(d);
  if( d->rfGraphGraphics != nullptr 
      and visibility != isRfGraphVisible() )
  {
    d->rfGraphGraphics->setVisible(visibility);
    emit rfGraphVisibilityChanged(visibility);
  }
}

void Node::showRfGraph()
{ setRfGraphVisibility(true); }

void Node::hideRfGraph()
{ setRfGraphVisibility(false); }



bool Node::isDirectedRfGraphVisible() const
{
  Q_D(const Node); Q_ASSERT(d);
  if(d->comLoopGraphGraphics!=nullptr)
  { return d->comLoopGraphGraphics->isVisible(); }
  else
  { 
    qWarning() << qualifiedName() << ": trying to access non-existing graphic item.";
    return false;
  }
}

void Node::setDirectedRfGraphVisibility(bool const & visibility)
{
  Q_D(Node); Q_ASSERT(d);
  if( d->comLoopGraphGraphics != nullptr 
      and visibility != isDirectedRfGraphVisible() )
  {
    d->comLoopGraphGraphics->setVisible(visibility);
    emit directedRfGraphVisibilityChanged(visibility);
  }
}

void Node::showDirectedRfGraph()
{ setDirectedRfGraphVisibility(true); }

void Node::hideDirectedRfGraph()
{ setDirectedRfGraphVisibility(false); }




bool Node::isBifurcationGraphVisible() const
{
  Q_D(const Node); Q_ASSERT(d);
  if(d->bifurcationGraphGraphics != nullptr)
  { return d->bifurcationGraphGraphics->isVisible(); }
  else
  { 
    qWarning() << qualifiedName() << ": trying to access non-existing graphic item.";
    return false;
  }
}

void Node::setBifurcationGraphVisibility(bool const & visibility)
{
  Q_D(Node); Q_ASSERT(d);
  if( d->bifurcationGraphGraphics != nullptr 
      and visibility != isBifurcationGraphVisible() )
  {
    d->bifurcationGraphGraphics->setVisible(visibility);
    emit bifurcationGraphVisibilityChanged(visibility);
  }
}

void Node::showBifurcationGraph()
{ setBifurcationGraphVisibility(true); }

void Node::hideBifurcationGraph()
{ setBifurcationGraphVisibility(false); }




bool Node::isObstacleEdgeAssociationVisible() const
{
  Q_D(const Node); Q_ASSERT(d);
  if(d->obstacleBiEdgeAssocGraphics!=nullptr)
  { return d->obstacleBiEdgeAssocGraphics->isVisible(); }
  else
  { 
    qWarning() << qualifiedName() << ": trying to access non-existing graphic item.";
    return false;
  }
}

void Node::setObstacleEdgeAssociationVisibility(bool const & visibility)
{
  Q_D(Node); Q_ASSERT(d);
  if( d->obstacleBiEdgeAssocGraphics != nullptr 
      and visibility != isObstacleEdgeAssociationVisible() )
  {
    d->obstacleBiEdgeAssocGraphics->setVisible(visibility);
    emit obstacleEdgeAssociationVisibilityChanged(visibility);
  }
}

void Node::showObstacleEdgeAssociation()
{ setObstacleEdgeAssociationVisibility(true); }

void Node::hideObstacleEdgeAssociation()
{ setObstacleEdgeAssociationVisibility(false); }


bool Node::isObstacleVertexAssociationVisible() const
{
  Q_D(const Node); Q_ASSERT(d);
  if(d->obstacleRouteVertexAssocGraphics!=nullptr)
  { return d->obstacleRouteVertexAssocGraphics->isVisible(); }
  else
  { 
    qWarning() << qualifiedName() << ": trying to access non-existing graphic item.";
    return false;
  }
}

void Node::setObstacleVertexAssociationVisibility(bool const & visibility)
{
  Q_D(Node); Q_ASSERT(d);
  if( d->obstacleRouteVertexAssocGraphics != nullptr 
      and visibility != isObstacleVertexAssociationVisible() )
  {
    d->obstacleRouteVertexAssocGraphics->setVisible(visibility);
    emit obstacleVertexAssociationVisibilityChanged(visibility);
  }
}

void Node::showObstacleVertexAssociation()
{ setObstacleVertexAssociationVisibility(true); }

void Node::hideObstacleVertexAssociation()
{ setObstacleVertexAssociationVisibility(false); }






#endif // DEBUG_GRAPHICS



void Node::buildBlipContextMenu(QMenu* parentMenu, const NodeId& id)
{ 
  if( !blipExists(id) )
  { return; }
  
  // 
  // persistent menu actions
  //
  if( parentMenu->actions().size() > 0 )
  { parentMenu->addSeparator(); }
  
  parentMenu->addAction(blip(id).menuAction());
  
  //
  // temporary menu actions
  //
  
  QAction* action = nullptr;
  if( id == this->id() )
  {
    //TODO: maybe add entries for self from the "debug menu" ?
  }
  else
  {
#ifdef DEBUG    
    action = new QAction(parentMenu);
    action->setText(Node::tr("&Delete"));
    
    parentMenu->addAction(action);
    
    Node::connect(action, &QAction::triggered,
      [=](){ deleteBlip(id); } );
#endif // DEBUG
  }
  
}

void Node::buildElementContextMenu(QMenu* parentMenu, const ElementId& id)
{
  if( !obstacleExists(id) )
  { return; }

  if( parentMenu->actions().size() > 0 )
  { parentMenu->addSeparator(); }
  
  // 
  // persistent menu actions
  //
  
  parentMenu->addAction( obstacle(id).menuAction());
  
  //
  // temporary menu actions
  //
  
  QAction* action = nullptr;
  
  action = new QAction(parentMenu);
  action->setText(Node::tr("Is &Target"));
  action->setCheckable(true);
  action->setChecked(targetId() == id);
  
  parentMenu->addAction(action);
  
  
  Node::connect(action, &QAction::triggered,
    [=](bool checked)
    { 
      if( checked )
      { setTargetId(id); }
      else
      { unsetTarget();}
    } );
  
  
  Q_D(Node); Q_ASSERT(d); 
  
  if( d->obstacleDelaunayFaceGraphics.count(id) == 1 )
  {
    action = new QAction(parentMenu);
    action->setText(Node::tr("Highlight &Cell"));
    action->setCheckable(true);
    action->setChecked(d->obstacleDelaunayFaceGraphics[id]->isVisible());
    
    action->setDisabled(!areObstacleCellsVisible());
    
    parentMenu->addAction(action);
    
    Node::connect(action, &QAction::triggered,
      [=](bool checked)
      { d->obstacleDelaunayFaceGraphics[id]->setVisible(checked); }
    );
  }
  else
  { qWarning() << "No cell graphic for obstacle" << obstacle(id).objectName(); }
  
}


void Node::buildFormationContextMenu(QMenu* parentMenu)
{
  Q_UNUSED(parentMenu);
}


#endif // WITH_GRAPHICS




void Node::joinSwarm()
{
  /** \todo Check that joining doesn't break the COM link... */
  
  Q_D(Node); Q_ASSERT(d);
  d->postCommand(IDL::Commands::N_JOIN_NETWORK);
}

void Node::leaveSwarm()
{
  Q_D(Node); Q_ASSERT(d);
  d->postCommand(IDL::Commands::N_LEAVE_NETWORK);
}


void Node::processReceivedMessage(const NodeId& sender, const IDL::Messages::ModeChangeCommand& message)
{ 
  if( NodeId(message.addressee) == id())
  {
    using namespace IDL::Commands;
    switch( message.command )
    {
      case NULL_COMMAND    : /* has no effect */  break;
      case N_JOIN_NETWORK  : joinSwarm();         break;
      case N_LEAVE_NETWORK : leaveSwarm();        break;
      default: /* do nothing */ break;
    }
  }
}



void Node::processReceivedMessage ( const NodeId& sender, const IDL::Messages::State& message )
{
  if( QThread::currentThread() != this->thread() )
  {
    QMetaObject::invokeMethod(this,"processReceivedMessage",Qt::BlockingQueuedConnection,
          Q_ARG(NodeId,   sender),
          Q_ARG(IDL::Messages::State, message )               
        );
    return;
  }

  
  Q_D(Node); Q_ASSERT(d);
  QMutexLocker locker(&(d->mutex));
   
  AMG::FrameOfReference* blipDatum = AMG::CoSy::getDatumFrame();//TODO: this assumes all nodes use the same datum
  
  auto& senderBlip = blip(sender);
//   qDebug() << qualifiedName() << ": Processing a STATE message from" << senderBlip.name();
  
  //
  // update the primary data 
  //
  
  AMG::Vector position3D(message.position.x,
                         message.position.y,
                         message.position.z,
                         blipDatum ); 
  
  senderBlip.setPosition3D(position3D);
  
  AMG::Vector velocity(message.velocity.x_dot,
                       message.velocity.y_dot,
                       message.velocity.z_dot,
                       blipDatum);
  
  senderBlip.setVelocity(velocity);
  
  AMG::EulerAngleTupel eulerAngles;
  eulerAngles[0] = message.attitude.phi;
  eulerAngles[1] = message.attitude.theta;
  eulerAngles[2] = message.attitude.psi;
  
  senderBlip.setAttitude(eulerAngles);
  
  d->updateTertiaryBlipData(&senderBlip);
}

void Node::processReceivedMessage ( const NodeId& sender, const IDL::Messages::Status& message )
{
  if( QThread::currentThread() != this->thread() )
  {
    QMetaObject::invokeMethod(this,"processReceivedMessage",Qt::BlockingQueuedConnection,
          Q_ARG(NodeId,   sender),
          Q_ARG(IDL::Messages::Status, message )               
        );
    return;
  }
  
  //   qDebug() << qualifiedName() << ": received a 'Status' message from" << blip(sender).name();
  Q_D(Node); Q_ASSERT(d);
  QMutexLocker locker(&(d->mutex));


  
//   try
  if( blipExists(sender) )
  { 
    Blip& b = blip(sender);
    b.setMode(message.mode);
    b.setClosestObstacleId(message.closestObstacle);
    b.setTargetId(message.target);
  }
//   catch(std::out_of_range const& e)
  else
  {
    QString message(qualifiedName());
    message.append(QString(" : Blip(%1) doesn't exists.").arg(sender.toQString()));
    qCritical("%s",message.toStdString().c_str());
  }
  
  
}

void Node::processReceivedMessage ( const NodeId& sender, const IDL::Messages::Datum& message )
{
  if( QThread::currentThread() != this->thread() )
  {
    QMetaObject::invokeMethod(this,"processReceivedMessage",Qt::BlockingQueuedConnection,
          Q_ARG(NodeId,   sender),
          Q_ARG(IDL::Messages::Datum, message )               
        );
    return;
  }
  
  Q_D(Node); Q_ASSERT(d);
  QMutexLocker locker(&(d->mutex));
  
  qDebug() << qualifiedName() << ": received a 'Datum' message from" << blip(sender).name();
  
}

void Node::processReceivedMessage ( const NodeId& sender, const IDL::Messages::Flightplan& message )
{
  if( QThread::currentThread() != this->thread() )
  {
    QMetaObject::invokeMethod(this,"processReceivedMessage",Qt::BlockingQueuedConnection,
          Q_ARG(NodeId,   sender),
          Q_ARG(IDL::Messages::Flightplan, message )               
        );
    return;
  }
  
  
  Q_D(Node); Q_ASSERT(d);
  QMutexLocker locker(&(d->mutex));
  
//   qDebug() << qualifiedName() << ": received a 'Flightplan' message from" << blip(sender).name();
  
  if( blipExists(sender) )
  {
    blip(sender).setFlightplan(message.plan);
    
    if( message.currentWaypoint > message.plan.waypoints.length() )
    {
      qWarning() << qualifiedName() <<": Received a 'Flightplan' message from"
      << blip(sender).name() << "with missmatching waypoint information."
      << "(Setting currentWaypoint =" << message.plan.waypoints.length() <<", the last one.)";
      
      blip(sender).setCurrentWaypointIndex( message.plan.waypoints.length() );
    }
    else
    { blip(sender).setCurrentWaypointIndex(message.currentWaypoint); }
    
    
//     qDebug() << qualifiedName() << ":" << blip(sender).name() << "current WP:"
//       << message.currentWaypoint << "/" << message.plan.waypoints.length() ;
  }
  else
  {
    QString message(qualifiedName());
    message.append(QString(" : Blip(%1) doesn't exists. Cannot process Flightplan.").arg(sender.toQString()));
    qCritical("%s",message.toStdString().c_str());
  }
  
}

Geometry::Polygon_2D& Node::arena()
{
  Q_D(Node); Q_ASSERT(d);
  return d->environment2D;
}

Geometry::Polygon_2D Node::arena() const
{
  Q_D(const Node); Q_ASSERT(d);
  return d->environment2D;
}


/** \internal The addition of a new obstacle also insertes a new vertex into the
 * obstacle graph. The vertex is assigned an index matching the position of the
 * obstacle's data in obstacleData. The vertex is positioned the the centroid
 * of the obstacle's shape, it is not connected to any other vertices in the 
 * obstacle graph.
 */
void Node::addObstacle(const Obstacle& obstacle)
{
  Q_D(Node); Q_ASSERT(d);
  
  /** \todo Before inserting a potentially new obstacle, do not only do some 
   * checks for uniqueness (i.e. confirm the obstacle is indeed new) but also 
   * for colissions (i.e. the obstacle doesn't interfere with other obstacles). */
  if( obstacleExists(obstacle.id()) )
  { 
    qDebug() << qualifiedName() << ": Won't add the new obstacle" << obstacle.objectName()
      << "as the already known obstacle" << d->obstacleData[obstacle.id()].objectName()
      << "has the same ElementId (" << obstacle.id() << ")";
    return;
  }
     
  // store a copy of the obstacle data in the database
  /** \todo Is that really what is happening here? I found some online docu
   * mentioning that one \e cannot copy a QObject... */
  d->obstacleData[obstacle.id()] = obstacle;
//   d->obstacleData[obstacle.id(),obstacle);
  
  Obstacle& newObstacle = d->obstacleData[obstacle.id()];
  
  // reparent the obstacle to this Node
  newObstacle.setParent(this);
  
  { // deal with the obstacleGraph
    using namespace boost;
        
    // create a location at the obstacle centroid and store it
    Location& location = d->makeLocation( newObstacle.centroid() );
   
    // add a new vertex to the obstacle graph...
    ObstacleGraph::Vertex_Descriptor elementVertex = boost::add_vertex(d->obstacleGraph);
  
    // ... and create the reverse lookup(s)
    newObstacle.setObstacleGraphVertex( elementVertex );  
    location.setObstacleGraphVertex( elementVertex );
    
    /** \todo The method to use the size of the obstacleData container might
     * currently work, but it won't work any more once the deletion of obstacles
     * is allowed as that would allow to take out any obstacle, not necessarily 
     * the last one added, hence creating obstacle index duplicates... :/ */
    // store the index as an internal property 
    put(vertex_index, d->obstacleGraph, elementVertex, d->obstacleData.size()); ///FIXME: See todo above.
    
    // store the ElementId of the obstacel as the vertex_id
    put(vertex_id, d->obstacleGraph, elementVertex, newObstacle.id() );
       
    // associate the obstacle graph vertex with that location
    put(vertex_locationId, d->obstacleGraph, elementVertex, location.id() );
  }
    
#ifdef WITH_GRAPHICS
  if( isInitialized() ) //NOTE: otherwise d->datum item is 0x0...
  {
    newObstacle.setParentItem(d->datumItem);
    newObstacle.setZValue(NodePrivate::GraphicItemZLevels::ObstacleGraphicZLevel);
  }
#endif // WITH_GRAPHICS
  
  // the insertion of a new obstacle invalidates the 2D environment and the 
  // related graphs
  d->environmentObstaclesAreCurrent = false;
  d->obstacleGraphIsCurrent = false;
  d->routeGraphIsCurrent = false;
  
  
//   { // debug
//     qDebug() << objectName() << ": Added an Obstacle. Current count is" 
//       << d->obstacleData.size() <<".";
// //     qDebug() << newObstacle;
//   }
  
}

/** \internal
 * todo:
 * - emit some sort of signal (aboutToDelete...)
 * - remove from location db
 * - remove from obstacle db
 * - trigger an update of the route and derived graphs.
 */
void Node::removeObstacle(ElementId const& elementId)
{
  qFatal("This method is not yet implemented.");
}


void Node::setOperationalBoundary(const Geometry::Polygon_2D& polygon)
{
  setOperationalBoundary(polygon.outer());
}

/** \todo Oh well, lots: technically we need two boundaries: a motion 
 * boundary to limit autmonomous motion (which this boundary actually is) \e and
 * a R/F visibility boundary. The motion boundary is needed to limit VRONI,
 * the R/F boundary is needed to limit VisiLibity.
 * A (the?) correct approch would be to take the motion boundary (i.e. this one)
 * and offset it to the outside by the R/F radius. (However, offsetting polygons
 * isn't that trivial. Boost::Geometry doesn't seem to be able to do it yet.)
 * 
 * \todo Update libvroni to enable polygon offset computation.
 */
void Node::setOperationalBoundary(const Geometry::Polygon_2D::ring_type& ring)
{
  // Check for self-intersections
  Q_ASSERT( !boost::geometry::intersects(ring) );
  
  Q_D(Node); Q_ASSERT(d);
  
  d->environmentBoundaryIsCurrent = false;
  d->routeGraphIsCurrent = false;

  //NOTE: I am actually not sure about this one. Changing the boundary currently
  // doesn't add or remove any obstacles and as such the neighboring situation 
  // of the obstacles (which is essentially what the obstacle graph represents)
  // doesn't change...
//   d->obstacleGraphIsCurrent = false; 

#ifdef WITH_GRAPHICS
  if(d->motionBoundaryItem != nullptr)
  {  
    Geometry::Polygon_2D tempPoly;
    boost::geometry::assign(tempPoly.outer(),ring);
    boost::geometry::correct(tempPoly);
    d->motionBoundaryItem->setGeometry(tempPoly);
  }
  else
  {
    /** \internal During intialization this part is called before the graphics
     * are initialized, i.e. the motionBoundaryItem doesn't exist yet. */
    boost::geometry::assign(d->motionBoundary.outer(),ring);
    boost::geometry::correct(d->motionBoundary);
  }
#else //WITH_GRAPHICS
  boost::geometry::assign(d->motionBoundary.outer(),ring);
  boost::geometry::correct(d->motionBoundary);
#endif //WITH_GRAPHICS


  // create a location for the boundary
  Geometry::Point_2D centroid;
  boost::geometry::centroid(d->motionBoundary,centroid);
  
  if( d->motionBoundaryLocationId != NULL_LOCATION_ID )
  { 
    { // deal with obstacle graph 
    using namespace boost;
    ObstacleGraph::Vertex_Descriptor boundaryVertex 
      = d->locationData[d->motionBoundaryLocationId].obstacleGraphVertex();
    clear_vertex(boundaryVertex, d->obstacleGraph);  
    remove_vertex(boundaryVertex, d->obstacleGraph);
    }
    
    d->locationData.erase(d->motionBoundaryLocationId);
  }

  Location& boundaryLoc = d->makeLocation(centroid);
  d->motionBoundaryLocationId = boundaryLoc.id();
  
  { // deal with obstacle graph 
  using namespace boost;
  ObstacleGraph::Vertex_Descriptor boundaryVertex = add_vertex(d->obstacleGraph);
//   put(vertex_index, d->obstacleGraph, boundaryVertex, 0); /// \todo FIXME This shouldn't be 0 as there are valid obstacles with that index... :(
  put(vertex_index, d->obstacleGraph, boundaryVertex, d->motionBoundaryObstacleVertexIndex); 
  put(vertex_id, d->obstacleGraph, boundaryVertex, d->motionBoundaryElementId);
  put(vertex_locationId, d->obstacleGraph, boundaryVertex, boundaryLoc.id());
  
  boundaryLoc.setObstacleGraphVertex(boundaryVertex);
  }
  
}


bool Node::isArenaCurrent() const
{
  Q_D(const Node);Q_ASSERT(d);
  return d->environmentBoundaryIsCurrent and d->environmentObstaclesAreCurrent;
}

/** \todo For larger environments, it might be necessary to select the Obstacles 
 * that are to be inluded in the Arena (as holes). Maybe the same strategy than
 * in the COM slot can be used? */
void Node::updateArena()
{
  Q_D(Node); Q_ASSERT(d);
  
  Geometry::Polygon_2D newArena = arena();
  
  if( !d->environmentBoundaryIsCurrent )
  {
    Geometry::Polygon_2D::ring_type& outer = newArena.outer();
    boost::geometry::assign(outer,d->motionBoundary);
  }
  
  if( !d->environmentObstaclesAreCurrent )
  {
    
    Geometry::Polygon_2D::inner_container_type& holes = newArena.inners();
    holes.clear();
    holes.resize( d->obstacleData.size() );

    int obstacleCounter = 0;
    for( auto& pair : d->obstacleData )
    {
      Obstacle const & obstacle = pair.second;
      
      /** \todo Currently boost::geometry doesn't have a within(Polygon,Polygon) 
      * function which could be used to only add holes from the environment that 
      * are within the outer boundary. */
      if( boost::geometry::within(obstacle.shape(),newArena) ) //FIXME: see todo
      {
        Geometry::Polygon_2D::ring_type& newHole = holes.at(obstacleCounter++);
        boost::geometry::assign(newHole,obstacle.shape());
//         qDebug() << qualifiedName() << ":" << obstacle.objectName() << "area  :" << boost::geometry::area(obstacle.shape()) << "m²";
      }
      else
      {
        /** \todo Do something about intersecting Obstacles. */
        
        /** \internal Note that the skipped obstacle will still be represented
         * in the VRONI environment. VRONI aparently performs some checks 
         * internally and doesn't seem to care about intersections. 
         * This has the charming benefit that althgouh an obstacle is skipped,
         * the route graph will still consider it.
         * \sa updateRouteGraph() */
        
        qWarning() << qualifiedName() << ": Obstacle intersections while updating arena."
          << "Skipping "<< obstacle.objectName() << "(ElementId:" << pair.first << ")."; 
      }
    }
  }
  
  boost::geometry::correct(newArena); // the orientation of the obstacle/hole needs to be reversed
  

  d->setEnvironment(newArena); /// This also triggers the graph updates
  d->environmentBoundaryIsCurrent = true;
  d->environmentObstaclesAreCurrent = true;
  
//   qDebug() << qualifiedName() << ": Updated the arena:" << arena().inners().size() << "obstacles.";
  
  emit arenaChanged();
}



ComGraph::Graph_t Node::comGraph() const
{
  Q_D(const Node);Q_ASSERT(d);
  return d->comGraph;
}

RouteGraph::Graph Node::routeGraph() const
{
  Q_D(const Node);Q_ASSERT(d);
  return d->routeGraph;
}

#ifdef WITH_RFSIMULATION
/** \todo : The whole WITH_RFSIMULATION is actually wrong, well, at least when it 
 * comes to the comGraph: A node can only \e not simulate the connection(s) of
 * the ownship. Any (potential) connections outside the singel hop distance 
 * have to be deduced, i.e. simulated... As such a comGraph per se is based 
 * either on a simulation \e OR a message send by each node whom that node is 
 * connected to... but that opens the door to routing ;) (like spanning trees,
 * etc.)
 * \internal This would be a great place to put a hook in for \tt ns-3, the network 
 * simulator (http://www.nsnam.org/).
 */
template<>
bool Node::isMessageReceivable(NodeId const & senderId,
                               IDL::Messages::State const & message)
{
  // avoid erroneous error messages during startup
  if(!isRunning())
  { return false; }
  
  // messages from self are always receivable
  if( senderId == ownship().id() )
  { return true; }
  
  
  bool messageIsReceivable = false;
   
  if( blipExists(senderId) )
  {
    /** \internal
    * ok, this is rather redundant, but the main work in this function is the 
    * computation of the R/F polygon which we need to be current in order to 
    * determine the connectedness. The same function will be called
    * if the connectedness has been determined to be true -- however, the 
    * compuationally costly R/F computation shouldn't be repeated as the 
    * minDistance criterion isn't met the next time around when it is called 
    * in case this message should turn out to be receivable
    * (i.e. the only cost is a double assignment of the primary data...)
    */
    processReceivedMessage(senderId,message); //NOTE: watch out, this is just asking for thread deadlock...
    
    
    Q_D(Node); Q_ASSERT(d);
    QMutexLocker locker(&(d->mutex));//TODO: Is this mutex needed?
    
    /** Determine if the message is actually receivable or not, based on the
     * connectivity of the sender. */
    messageIsReceivable = blip(senderId).isConnectedToHost(); 
  }
  else
  { // sender blip doesn't exist in the receiving nodes blip database
      QString message(qualifiedName());
      message.append(QString(": WARNING: received a %1 message from the unknown Node %2.")
                      .arg(QString(IDL::Messages::State::TypeSupport::get_type_name()))
                      .arg(senderId.toQString()));
      qWarning("%s",message.toStdString().c_str());
  };
 
  return messageIsReceivable;
}

#endif // WITH_RFSIMULATION

int Node::metronome ( unsigned int period_ms )
{
  Q_D(Node); Q_ASSERT(d);
  
  if( d->metronome.contains(period_ms) )
  { // a metronome with that period already exists
    return d->metronome[period_ms];
  }
  else if( isRunning() )
  { // create and immediately start a new timer
    int id = startTimer(period_ms);
    d->metronome[period_ms] = id;
    return id;
  }
  else
  { // just create an entry in the hash to be started later
    return d->metronome[period_ms];
  }
}

#ifdef WITH_GRAPHICS

QGraphicsScene* Node::scene() const
{
  Q_D(const Node); Q_ASSERT(d);
//   return d->datumItem->scene();
  return d->scene;
}

void Node::setScene ( QGraphicsScene*const scene )
{
  Q_D(Node); Q_ASSERT(d); 
  d->scene = scene;
  scene->addItem(d->datumItem);
  
//   scene->addItem(d->environmentItem);
}


QAbstractItemModel* Node::blipModel()
{
  Q_D(Node); Q_ASSERT(d);
  return d->blipModel;
}


#endif // WITH_GRAPHICS

#include "node.moc"