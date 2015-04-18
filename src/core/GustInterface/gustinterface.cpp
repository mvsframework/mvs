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

#include "GustInterface/gustinterface.h"
#include "GustInterface/gustinterface_p.h"

#include <QtCore/QFile>
#include <QtCore/QDir>

#include "XSD/xmltools.h"
#include "StateMachine/transitions.h"

//############################################################################//
//                                                                            //
//  GustInterfacePrivate                                                        //
//                                                                            //
//############################################################################//


GustInterfacePrivate::GustInterfacePrivate ( GustInterface* q
                                            ,GustInterface::Settings* const settings
                                            ,Node* parent)
  :SimItemPrivate(q,settings)
  ,q_ptr ( q )
  ,xmlSettings(settings)
{ //NOTE: Do _NOT_ use q_ptr or q in here, it _WILL_ break things!
}

GustInterfacePrivate::~GustInterfacePrivate()
{
}


void GustInterfacePrivate::initializeNetwork()
{
  Q_Q(GustInterface);Q_ASSERT(q);
  
  qDebug() << q->qualifiedName() << ": Network configuration: remote port at"
    << QString("%1:%2").arg( xmlSettings->gustIp().c_str() ).arg(xmlSettings->gustUdpPort() )
    << ", local port at"
    << QString("%1:%2").arg( xmlSettings->localIp().c_str() ).arg( xmlSettings->localUdpPort() )  ;
  
  //
  // process the XML
  // ===============
  // NOTE: the next four entries are guaranteed to be in the XML struct as 
  // otherwise the XML parser would not have been able to validate against the
  // XSD
  gustHostAddress = XML::qString(xmlSettings->gustIp());
  gustHostUdpPort = xmlSettings->gustUdpPort(); 
  localHostAddress = XML::qString(xmlSettings->localIp());
  localUdpPort = xmlSettings->localUdpPort();
  
  // 
  // initialize the interface
  //
  
  udpSocketSend = new QUdpSocket(q);
  udpSocketReceive = new QUdpSocket(q);
  gustConnectionTimer = new QTimer(q);
  
}

void GustInterfacePrivate::initializeStates()
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
    
    connectionInactive = new State(IDL::Data::STATE_ID_GUST_INACTIVE,running_itemRegion);
    connectionActive   = new State(IDL::Data::STATE_ID_GUST_ACTIVE, QState::ParallelStates, running_itemRegion);
    
    trajectoryStatus_gustInterfaceRegion = new State(IDL::Data::STATE_ID_GUST_TRAJ_REGION, connectionActive);;
    
    trajectoryStatusInitial = new State(IDL::Data::STATE_ID_GUST_TRAJ_INITIAL, trajectoryStatus_gustInterfaceRegion);
    trajectoryStatusInitial->setDebugPrinting(false);
    trajectoryStatusCruise  = new State(IDL::Data::STATE_ID_GUST_TRAJ_CRUISE,  trajectoryStatus_gustInterfaceRegion);
    trajectoryStatusCruise->setDebugPrinting(false);
    trajectoryStatusStop    = new State(IDL::Data::STATE_ID_GUST_TRAJ_STOP,    trajectoryStatus_gustInterfaceRegion);
    trajectoryStatusStop->setDebugPrinting(false);
    
    
    guidanceStatus_gustInterfaceRegion = new State(IDL::Data::STATE_ID_GUST_GUIDANCE_REGION, connectionActive);
    
    guidanceStatusGo    = new State(IDL::Data::STATE_ID_GUST_GUIDANCE_GO,    guidanceStatus_gustInterfaceRegion);
        
    guidanceStatusStop  = new State(IDL::Data::STATE_ID_GUST_GUIDANCE_STOP,  guidanceStatus_gustInterfaceRegion);
  }
  
  //
  // connect some additional signals
  //
  {
    Q_Q(GustInterface); Q_ASSERT(q);
        
    GustInterface::connect(connectionInactive, &State::entered,
      [q] () {emit q->livelinessChanged(false);}  );
    
    GustInterface::connect(connectionActive, &State::entered,
      [q] () {emit q->livelinessChanged(true);} ); 
    
    GustInterface::connect(guidanceStatusGo, &State::entered,
      [q] () { emit q->guidanceModeChanged(GUST::M1_Safemode_Status::go); }  );
        
    GustInterface::connect(guidanceStatusStop, &State::entered,
      [q] () { emit q->guidanceModeChanged(GUST::M1_Safemode_Status::stop); }  );
  }
  
  //
  // set the initial (sub-)states
  // ==============================
  // Any "Region" state is "just" a Qt construct to allow parallelization at 
  // the level directly above. As such regions don't need to be set themselves 
  //   as initial states of anything as their use implies that they are children 
  // of a QState::ParallelStates compound state and Qt automatically enters all
  // parallel children of compound states upon entry.
  //
  // NOTE: Activities automatically set the "done" substate as their initial 
  // state.
  //
  {
    running_itemRegion->setInitialState(connectionInactive);
   
    trajectoryStatus_gustInterfaceRegion->setInitialState(trajectoryStatusStop);
    guidanceStatus_gustInterfaceRegion->setInitialState(guidanceStatusStop);
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
    Q_Q(GustInterface); Q_ASSERT(q);
    
    connectionInactive->addTransition(q, SIGNAL(messageReceived(int)),
                                      connectionActive);
  
    connectionActive->addTransition(gustConnectionTimer, SIGNAL(timeout()),
                                    connectionInactive );    
    
    connectionActive->addTransition(q->stateMachine(), SIGNAL(stopped()),
                                    connectionInactive );
    
    using namespace Transitions;
    
    auto trajStop_1 = new TrajectoryTransition(GUST::TRAJ_STATUS_STOP);
    trajStop_1->setTargetState(trajectoryStatusStop);
    trajectoryStatusInitial->addTransition(trajStop_1); // assumes possession of transition pointer
    trajStop_1 = nullptr; // prevent reuse
    
    auto trajStop_2 = new TrajectoryTransition(GUST::TRAJ_STATUS_STOP);
    trajStop_2->setTargetState(trajectoryStatusStop);
    trajectoryStatusCruise->addTransition(trajStop_2); // assumes possession of transition pointer
    trajStop_2 = nullptr; // prevent reuse
    
    auto trajCruise_1 = new TrajectoryTransition(GUST::TRAJ_STATUS_CRUISE);
    trajCruise_1->setTargetState(trajectoryStatusCruise);
    trajectoryStatusInitial->addTransition(trajCruise_1); // assumes possession of transition pointer
    trajCruise_1 = nullptr; // prevent reuse
    
    auto trajCruise_2 = new TrajectoryTransition(GUST::TRAJ_STATUS_CRUISE);
    trajCruise_2->setTargetState(trajectoryStatusCruise);
    trajectoryStatusStop->addTransition(trajCruise_2); // assumes possession of transition pointer
    trajCruise_2 = nullptr; // prevent reuse
    
    auto trajInitial_1 = new TrajectoryTransition(GUST::TRAJ_STATUS_INITIAL);
    trajInitial_1->setTargetState(trajectoryStatusInitial);
    trajectoryStatusCruise->addTransition(trajInitial_1); // assumes possession of transition pointer
    trajInitial_1 = nullptr; // prevent reuse
    
    auto trajInitial_2 = new TrajectoryTransition(GUST::TRAJ_STATUS_INITIAL);
    trajInitial_2->setTargetState(trajectoryStatusInitial);  
    trajectoryStatusStop->addTransition(trajInitial_2); // assumes possession of transition pointer
    trajInitial_2 = nullptr; // prevent reuse
    
    
    auto guidanceGo = new GuidanceTransition(GUST::M1_Safemode_Status::go);
    guidanceGo->setTargetState(guidanceStatusGo);
    guidanceStatusStop->addTransition(guidanceGo); // assumes posession of guidanceGo
    guidanceGo = nullptr;
      
    auto guidanceStop = new GuidanceTransition(GUST::M1_Safemode_Status::stop);
    guidanceStop->setTargetState(guidanceStatusStop);
    guidanceStatusGo->addTransition(guidanceStop); // assumes posession of guidanceStop
    guidanceStop = nullptr;

    
  }
}


void GustInterfacePrivate::unInitializeGustInterface()
{
  // undoing whatever GustInterface::initializeItem() did...
  
  delete udpSocketSend;
  udpSocketSend = nullptr;
  
  delete udpSocketReceive;
  udpSocketReceive = nullptr;
  
  delete gustConnectionTimer;
  gustConnectionTimer = nullptr;
  
}

void GustInterfacePrivate::connectSockets()
{
  Q_Q(GustInterface);Q_ASSERT(q);
  
  udpSocketSend->bind(localHostAddress,localUdpPort
    , QUdpSocket::ShareAddress | QUdpSocket::ReuseAddressHint // bind mode. //NOTE: not certain if this is necessary
  );
  udpSocketSend->connectToHost(gustHostAddress,gustHostUdpPort);
  
  udpSocketReceive->bind(localHostAddress,localUdpPort
    , QUdpSocket::ShareAddress | QUdpSocket::ReuseAddressHint // bind mode. //NOTE: not certain if this is necessary
  );
  q->connect(udpSocketReceive, SIGNAL(readyRead())  ,
             q, SLOT(readPendingDatagramms()) );
 

  gustConnectionTimer->start(gustTimeOut);
  q->connect(gustConnectionTimer, SIGNAL(timeout())    ,
             q, SLOT(processConnectionTimeOut()) );
}

void GustInterfacePrivate::disConnectSockets()
{
  Q_Q(GustInterface);Q_ASSERT(q);
  
  
  gustConnectionTimer->start();  
  q->disconnect(gustConnectionTimer, SIGNAL(timeout()) ,
                q, SLOT(readPendingDatagramms()) );
  
  udpSocketSend->close();
  udpSocketReceive->close();
  q->disconnect(udpSocketReceive, SIGNAL(readyRead())     ,
                q, SLOT(processConnectionTimeOut()) );
}


void GustInterfacePrivate::restartConnectionTimer()
{
  Q_Q(GustInterface); Q_ASSERT(q);
  if( !q->isAlive() )
    qWarning() << q->qualifiedName() << ": Datalink connection to GUST (re-)started.";
  gustConnectionTimer->start(gustTimeOut);
} 


//############################################################################//
//                                                                            //
//  GustInterface                                                             //
//                                                                            //
//############################################################################//


void GustInterface::generateSetupInputFiles ( const GustUav::Settings* settingsXml )
{
  
  GustInterface::Settings const & gustXml = settingsXml->gust();
  
  //
  // Create GUST input files to ease the set up
  //
  
  QDir inpFilePath;
  if( gustXml.inpFilePath().present() )
  { // a setting is present in the XML file, use that
   inpFilePath.setPath(XML::qString( gustXml.inpFilePath().get()));
   // deal with potential "~" to indicate the HOME path
   if( inpFilePath.isRelative() and inpFilePath.path().startsWith("~") )
     inpFilePath.setPath(inpFilePath.path().replace("~",inpFilePath.homePath()));
  }
  else
  { // no XML setting present, use a default
    inpFilePath.setPath("gustInputFiles"); //FIXME: Magic Numbers
  }
  
  if( !inpFilePath.exists() )
    inpFilePath.mkpath(inpFilePath.absolutePath());
  
  QList<QStringList> coordinates = XML::splitXmlStringList( settingsXml->coordinates() );
  QHostAddress gustHostAddress(XML::qString( gustXml.gustIp()));
//   qint16 gustHostUdpPort = settings.gustUdpPort(); // not used in this function
  QHostAddress localHostAddress(XML::qString( gustXml.localIp()));
  qint16 localUdpPort =gustXml.localUdpPort();
  
  QString nodeName =  XML::qString(settingsXml->name());
  QString setupFileName = QString("%1/%2.inp").arg(inpFilePath.path()).arg( nodeName.simplified().replace(" ","").toLower() );
  QFile setupFile(setupFileName);
  if( setupFile.exists() )
    setupFile.remove();
  if( !setupFile.open(QIODevice::WriteOnly | QIODevice::Text) )
        qDebug() << nodeName << "Can't open"<< setupFile.fileName();
  
  QTextStream setupFileOut(&setupFile);
  setupFileOut 
    << "#######################################################################\n"
    << "# THIS FILE IS AUTOGENERATED AND MIGHT BE OVERWRITTEN WITHOUT NOTICE! #\n"
    << "#######################################################################\n"
    << "\n"
    << "#\n"
    << "# Moving the Datum\n"
    << "# ================\n"
    << "#\n"
    << "navinit.datumLon = " << coordinates[0].at(0) << "\n" 
    << "navinit.datumLat = " << coordinates[0].at(1) << "\n"
    << "navinit.datumAlt = " << AMG::Units::meter2feet(coordinates[0].at(2).toDouble()) << "\n"
    << "\n" 
    << "#\n"
    << "# Adjusting the Joystick Interface\n"
    << "# ================================\n"
    << "#\n"
    << "manAugment.anom     = " << gustXml.manAugment().anom() << "\n" 
    << "manAugment.jerk     = " << gustXml.manAugment().jerk() << "\n"
    << "manAugment.vcHzGo   = " << gustXml.manAugment().vcHzGo() << "\n"
    << "manAugment.vcHzStop = " << gustXml.manAugment().vcHzStop() << "\n"
    << "manAugment.vcHzSide = " << gustXml.manAugment().vcHzSide() << "\n"
    << "\n" 
    << "#\n"
    << "# Adjusting the Aerodynamic Model\n"
    << "# ===============================\n"
    << "#\n"
    << "# This is used to reduce pitch oscillation in the model.\n" 
    << "params_mr.damp = " << gustXml.params_mr().damp() << "\n" //HACK: This is used to reduce pitch oscillation in the model.
    << "\n" 
    << "# enable the Google Maps overlay to check positioning.\n" 
    << "sceneGlobal.overlaySource = 1\n" 
    << "\n" 
    << "#\n"
    << "# Setting up the Datalink\n"
    << "# =======================\n"
    << "# This sets up the datalink to communicate with a MVS node instance acting as a GCS\n"
    << "#\n"
    << "onboardPortDatalink2.connectTo  = " << localHostAddress.toString() << "\n"
    << "onboardPortDatalink2.dataSource = 1\n" // HITL
    << "onboardPortDatalink2.portNum    = "<< localUdpPort - 2000 <<"\n" //FIXME: Magic Numbers
    << "onboardPortDatalink2.init       = 1    \n" // reinitialize the datalink
    << "\n"
    << "#\n"
    << "# Starting GUST\n"
    << "# =============\n"
    << "# Do some final preparations and start GUST\n"
    << "#\n"
    << "scene 0 close\n" // close the default scene window
    << "brwin 0 close\n" // close the default browser window
    << "onboardSet.init = 0\n" // prevent the (init-)run command to override settings.
    << "run\n";
    
  setupFile.close();  
}




/** \note This constructor does NOT do any work!
 * All GustInterface level construction needs to happen in 
 * GustInterface(GustInterfacePrivate & dd, ...)!
 */
GustInterface::GustInterface( Settings* const settings
                             ,Node* parent)
  :GustInterface(*new GustInterfacePrivate(this,settings,parent),parent)
{}

GustInterface::GustInterface( GustInterfacePrivate& dd, Node* parent )
  : SimItem(dd, parent) // GustInterfaces have SimItems as parents
{
  Q_D(GustInterface); Q_ASSERT(d);
  /** \internal 
   * This will only indicate the status of the GustInterface, any derived classes
   * will not be constructed at this point!
   */
  d->setStatus(Status::Constructed); 
}


GustInterface::~GustInterface()
{
  Q_D(GustInterface); Q_ASSERT(d);
  d->unInitializeGustInterface();
}




void GustInterface::initializeItem()
{
  SimItem::initializeItem();
  // do local work below ...
  
  Q_D(GustInterface); Q_ASSERT(d);
  d->initializeNetwork();
  d->initializeStates();
}

void GustInterface::startItem()
{
  SimItem::startItem();
  // do local work below ...
  
  Q_D(GustInterface); Q_ASSERT(d);
  d->connectSockets();
  
}

void GustInterface::stopItem()
{
  Q_D(GustInterface); Q_ASSERT(d);
  d->disConnectSockets();
  
  // do local work above...
  SimItem::stopItem();
}

void GustInterface::unInitializeItem()
{
  Q_D(GustInterface); Q_ASSERT(d);
  d->unInitializeGustInterface();
  
  // do local work above...
  SimItem::unInitializeItem();
}

void GustInterface::readPendingDatagramms()
{
  using namespace GUST;
  
  Q_D(GustInterface); Q_ASSERT(d);

  
  quint16  sendersPort;
  QHostAddress sendersAddress;

  QByteArray datagram;
  datagram.resize(d->udpSocketReceive->pendingDatagramSize()); 
  
  // read the pending datagram
  qint64 size = d->udpSocketReceive->readDatagram(datagram.data(), datagram.size(),
                                        &sendersAddress,&sendersPort);
  
  if( size != -1 )
  {
    // do some preliminary checking
    if( sendersAddress != d->gustHostAddress or sendersPort != d->gustHostUdpPort )
    { // this is not what we are expecting. discard...
      return;
    }
    
    datalinkHeader_ref* header = nullptr;
    header = reinterpret_cast<datalinkHeader_ref*>(datagram.data());
   
    // some _very_rudamentary_ error checking
    if (   header->sync1 != SYNC_BYTE_1 
        or header->sync2 != SYNC_BYTE_2
        or header->sync3 != SYNC_BYTE_3 )
    {
      qWarning() << qualifiedName() << ": Sync byte sequence does not match.";
      return;
    }
    else
    {
      /** \todo Compute and check checksum to confirm message integrity. */
    }
    
    d->restartConnectionTimer();
    
    emit messageReceived(header->messageID);
    
    // count the receipt of this message
    ++d->receivedMessageCount[header->messageID]; //NOTE: Qt automatically initializes primitive types to 0
    
    // process the message
    switch( header->messageID )
    {
      case DATALINK_MESSAGE0 :
        d->processReceivedMessage<datalinkMessage0_ref>(datagram.data());
        break;
      case DATALINK_MESSAGE1 :
        d->processReceivedMessage<datalinkMessage1_ref>(datagram.data());
        break;
      case DATALINK_MESSAGE_FLIGHTPLAN :
        d->processReceivedMessage<datalinkMessageFlightPlan_ref>(datagram.data());
        break;
      default:
        d->processReceivedMessage<datalinkHeader_ref>(datagram.data());    
    }
  }
  else
  {
    qWarning() << qualifiedName() << ": Could not read a pending Datagram.";
  }    
}

AMG::Vector GustInterface::position() const
{
  Q_D(const GustInterface); Q_ASSERT(d);
  return AMG::Vector(d->positionCoords,d->gustDatum);
}

AMG::Vector GustInterface::velocity() const
{
  Q_D(const GustInterface); Q_ASSERT(d);
  return AMG::Vector(d->velocityCoords,d->gustDatum);
}

AMG::QuaternionTupel GustInterface::attitude() const
{
  Q_D(const GustInterface); Q_ASSERT(d);
  return d->quaternions;
}

bool GustInterface::uploadFlightplan ( const Flightplan& flightplan )
{
  { /// checking that the attached GUST instance is alive
    if( !isAlive() )
    { 
      qWarning("\"%s\" : Cannot upload a flightplan as the connection is dead."
        ,qualifiedName().toStdString().c_str());
      return false;
    }
  }
  
  Q_D(GustInterface); Q_ASSERT(d);
  
  if( flightplan.waypoints.count() <=GUST::maxNumberOfWaypoints )
  {
    GUST::datalinkMessageFlightPlan_ref fpMessage;
    fpMessage.spare = GustInterfacePrivate::getNextSpareByte();
  
    // insert the waypoints into the message
    int wpIndex = 0;
//     GUST::datalinkMessageFlightPlanWaypoint_ref* gustFpWp = nullptr;
    for( Waypoint wp : flightplan.waypoints )
    {
      // get coordinates wrt. to the GUST datum
      
      AMG::Vector pos(wp.position().x,
                      wp.position().y,
                      wp.position().z,
                      AMG::CoSy::getDatumFrame()
                     );
      AMG::CoordinateTupel gustCoords = pos.absoluteCoordsIn(d->gustDatum);
      
      double datumPosX = gustCoords(0);
      double datumPosY = gustCoords(1);
      double datumPosZ = gustCoords(2);
      
      auto gustFpWp = reinterpret_cast<GUST::datalinkMessageFlightPlanWaypoint_ref*>(fpMessage.waypoint[wpIndex++]);
    
      gustFpWp->anom = 10; //FIXME: Magic Numbers
      gustFpWp->hdgAltMode = static_cast<char>(GUST::HDG_VELOCITY)+(static_cast<char>(GUST::ALT_ABS)<<4);
      gustFpWp->psi = 0;
      gustFpWp->type = GUST::MAN_CUTCORNER;  //FIXME: Magic Numbers
      gustFpWp->vnom = 15;  //FIXME: Magic Numbers
      gustFpWp->x[0] = AMG::Units::meter2feet(datumPosX); // North (wrt. esim datum) in [ft]
      gustFpWp->x[1] = AMG::Units::meter2feet(datumPosY); // East (wrt. esim datum) in [ft]
      gustFpWp->x[2] = AMG::Units::meter2feet(datumPosZ); // Down (wrt. esim datum) in [ft]
    }    
  
    {/// update some general data 
      fpMessage.messageSize = sizeof(GUST::datalinkHeader_ref) + 4*sizeof(char) 
      + flightplan.waypoints.count()*sizeof(GUST::datalinkMessageFlightPlanWaypoint_ref);
      
      fpMessage.lastIndex = --wpIndex; // subract one to compensate for the last incrementation
    }
    
    {/// alter the last waypoint
     auto lastGustWp = reinterpret_cast<GUST::datalinkMessageFlightPlanWaypoint_ref*>(fpMessage.waypoint[wpIndex]);
     lastGustWp->type = GUST::MAN_STOPAT;
    }

  
    // send the message and return
    return d->sendUDPMessage(fpMessage);
  }
  else // the flight plan has too many waypoints
  {
    qWarning("\"%s\" : Cannot upload a flight plan with %d waypoints.(The MAXIMUM is %d.)"
      ,qualifiedName().toStdString().c_str()
      ,flightplan.waypoints.count()
      ,GUST::maxNumberOfWaypoints  );
    return false;
  }
}

void GustInterface::sendCommand ( GustInterface::TrajectoryCommand command )
{
  Q_D(GustInterface); Q_ASSERT(d);
  
  GUST::datalinkMessageTrajectory_ref message(command);
  message.spare = GustInterfacePrivate::getNextSpareByte();
  
  d->sendUDPMessage(message);
}


void GustInterface::sendJoystick ( IDL::Data::JoystickData data )
{
    GUST::datalinkMessageUp0_ref up0;

    Q_D(GustInterface); Q_ASSERT(d);
    up0.spare = GustInterfacePrivate::getNextSpareByte();
    
    //
    // Axis Data
    //
    up0.pitchStick    = data.axes.pitchStick;
    up0.rollStick     = data.axes.rollStick;
    up0.rudderPedal   = data.axes.rudderPeddal;
    
    // the altitudeHold mode simply disables "throttle" commands as that results
    // in maintaining altitude ("throttle" is mapped to "vertical speed" in
    // GUST)
    if( data.actions.altitudeHold == false )
    { up0.throttleLever = data.axes.throttleLever; }
    else
    { up0.throttleLever = 0.0F;}
    
    //
    // Buttons
    //
    up0.button[GUST::GoFastButton] = data.actions.dash ? 1 : 0 ;

    
    // send the esim message
    d->sendUDPMessage(up0);
}


bool GustInterface::isAlive()
{
  Q_D(GustInterface); Q_ASSERT(d);
  bool alive = d->stateMachine->configuration().contains(d->connectionActive);
  return alive;
}



#include "gustinterface.moc"
