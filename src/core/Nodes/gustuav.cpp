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

#include "Nodes/gustuav.h"
#include "Nodes/gustuav_p.h"

#include <QtCore/qglobal.h>
#include <QtCore/QDebug>
#include <qtimer.h>

//############################################################################//
//                                                                            //
//  GustUavPrivate                                                            //
//                                                                            //
//############################################################################//

GustUavPrivate::GustUavPrivate ( GustUav* q
                                ,GustUav::Settings* const settings
                                /*,Vehicle* parent*/ )
  :VehiclePrivate(q,settings)
  ,q_ptr ( q )
  ,xmlSettings(settings)
{ //NOTE: Do _NOT_ use q_ptr or q in here, it _WILL_ break things!
}



GustUavPrivate::~GustUavPrivate()
{
}

void GustUavPrivate::unInitializeGustUav()
{
  { /** - Uninitialize the GustInterface.
     *    As gust is its own SimItem, there is nothing to be done
     */
  }
}

void GustUavPrivate::updateCurrentWaypointIndex(unsigned int const & index)
{
  ownshipBlip->setCurrentWaypointIndex(index);
}



bool GustUavPrivate::detectFinishedFlightPlan(double const& threshold_distance_m)
{
  Q_Q(GustUav); Q_ASSERT(q);
  
  auto distanceToLastWaypoint = [=]()
    { 
      // the waypoint sequence of the current flightplan
      auto wpSeq = ownshipBlip->flightplan().waypoints;
      
      // the distance to the last waypoint of the current flightplan
      double distance = Waypoint::distance(wpSeq[wpSeq.length()-1],ownshipBlip->position3D());
      
      qDebug() << q->qualifiedName() << ": Distance to last waypoint:"<< distance << "m."
      << "Threshold to declare \"last waypoint reached\":" << threshold_distance_m << "m.";
      
      return distance;
    };
  
  if( distanceToLastWaypoint() <= threshold_distance_m ) 
  {
    emit q->flightplanFinished();
    return true;
  }
  //else
  return false;
}


//############################################################################//
//                                                                            //
//  GustUav                                                                   //
//                                                                            //
//############################################################################//

/** \note This constructor does NOT do any work!
 * All GustUav level construction needs to happen in 
 * GustUavGustUavPrivate & dd, ...)!
 */
GustUav::GustUav ( Settings*const settings
                 /* ,Vehicle* parent*/ )
  :GustUav(*new GustUavPrivate(this,settings/*,parent*/)/*,parent*/)
{}

GustUav::GustUav ( GustUavPrivate& dd
                  /*,Vehicle* parent*/ )
  :Vehicle(dd/*,parent*/) 
{
  Q_D(GustUav); Q_ASSERT(d);
  d->gust = new GustInterface(&(d->xmlSettings->gust()), this);
    
  GustInterface::generateSetupInputFiles(d->xmlSettings);
}

GustUav::~GustUav()
{
  Q_D(GustUav); Q_ASSERT(d);
  d->unInitializeGustUav();
  
  // gust doesn't need to be deleted as it is a child of this, hence Qt takes
  // care of this.
}



void GustUav::initializeItem()
{
  Vehicle::initializeItem();
  
  Q_D(GustUav); Q_ASSERT(d);
  d->gust->initialize();

  connect(d->gust,&GustInterface::guidanceModeChanged,
    [=](GustInterface::GuidanceMode const & mode)
    {
      // the waypoint sequence of the current flightplan
      auto wpSeq = ownship().flightplan().waypoints;
      
      if( mode==GustInterface::GuidanceMode::stop
          and wpSeq.length() > 0
          and d->ownshipBlip->currentWaypointIndex() == 0 )
      { d->detectFinishedFlightPlan(); }
    }
  );
  
}

void GustUav::startItem()
{
  Vehicle::startItem();
  
  Q_D(GustUav); Q_ASSERT(d);
  d->gust->run();
  
  connect(d->gust,&GustInterface::stateVectorUpdateAvailable,
          this, &GustUav::synchronizeOwnshipState );
  
  connect(d->gust, &GustInterface::currentWaypointChanged,
    [d] (unsigned int index) {d->updateCurrentWaypointIndex(index);} ); 
}

void GustUav::stopItem()
{
  Q_D(GustUav); Q_ASSERT(d);
  
  disconnect(d->gust,&GustInterface::stateVectorUpdateAvailable,
          this, &GustUav::synchronizeOwnshipState );
  
  d->gust->halt();
    
  Vehicle::stopItem();
}

void GustUav::unInitializeItem()
{
  Q_D(GustUav); Q_ASSERT(d);
  d->unInitializeGustUav();
  
  Vehicle::unInitializeItem();
}

void GustUav::executeFlightplan ( const Flightplan& plan )
{
  
  /** \internal The detectFinishedFlightPlan check leads to the fact that no
   * "loop"-flightplans can be issued, i.e. flightplans that end up at the 
   * start location.
   * 
   * \todo Expand the detectFinishedFlightPlan check to allow cyclic flightplans
   * but still detect unnecessary ones that won't result in motion.
   */
  
  
  Q_D(GustUav); Q_ASSERT(d);
  if( !d->detectFinishedFlightPlan() // check whether the last WP might already be reached
      and d->gust->isAlive()
      and d->gust->uploadFlightplan( plan ) )
  {
     gust_trajGo();
  }
  else
  {
    qWarning("\"%s\" : Could not process a Flightplan."
      ,qualifiedName().toStdString().c_str() );
  }
}

void GustUav::timerEvent ( QTimerEvent* event )
{
  Node::timerEvent ( event );
  
  const int timerId = event->timerId();
  Q_D(GustUav); Q_ASSERT(d);
  
//   if( timerId == metronome(100) )
//   {
//    d->randomWalk(1.0); 
//   }
// else 
  if( timerId  == metronome(30000) )
  { // every 10 seconds stuff
    
    if (false) {              
      auto coords = d->ownshipBlip->position3D().coords();
      auto distance = d->rZeroOne() *  10;
      coords(0) += distance * std::cos(AMG::Units::degree2radian(d->ownshipBlip->yaw()));
      coords(1) += distance * std::sin(AMG::Units::degree2radian(d->ownshipBlip->yaw()));
      
      Waypoint wp;
      wp.setPosition( coords(0), coords(1), coords(2) );
      
      goTo(wp);
    }
    
    if( true) {
      

    }
     
  }

}

void GustUav::synchronizeOwnshipState()
{
  Q_D(GustUav); Q_ASSERT(d);
  
  AMG::QuaternionTupel quaternions = d->gust->attitude();
  
  // The Euler angles of the GUST instance wrt. to the GUST datum
  AMG::EulerAngleTupel euler;
    AMG::FrameOfReference::quaternions2euler<AMG::Units::degree>
    (
      quaternions[0],
      quaternions[1],
      quaternions[2],
      quaternions[3],
      euler[2], // psi,
      euler[1], // theta,
      euler[0] // phi
    );
  
  /** \todo The utilzes way of converting the Euler angles only works because I 
   * \e know that both datum frames are NED frames which are not far appart, 
   * i.e. the relative rotation is more or less neglegible. This is important
   * as the Euler angles I have are given wrt. to the GUST datum, but 
   * setOwnshipStateVector expects Euler angles wrt. to the node's datum.
   */
  d->setOwnshipStateVector( d->gust->position()
                           ,d->gust->velocity()
                           ,euler );
}

void GustUav::stopMotion()
{
//   Q_D(GustUav); Q_ASSERT(d);
//   d->gust->sendCommand(GUST::TRAJECTORY_STOP);
  gust_trajStop();
}

void GustUav::continueMotion()
{
//   Q_D(GustUav); Q_ASSERT(d);
//   d->gust->sendCommand(GUST::TRAJECTORY_GO);
  gust_trajGo();
}


void GustUav::gust_trajGo()
{
  Q_D(GustUav); Q_ASSERT(d);
  d->gust->sendCommand(GUST::TRAJECTORY_GO);
}

void GustUav::gust_trajStop()
{
  Q_D(GustUav); Q_ASSERT(d);
  d->gust->sendCommand(GUST::TRAJECTORY_STOP);
}

void GustUav::gust_trajPanicStop()
{
  Q_D(GustUav); Q_ASSERT(d);
  d->gust->sendCommand(GUST::TRAJECTORY_PANICSTOP);
}

void GustUav::processReceivedMessage ( const NodeId& sender, const IDL::Messages::JoystickCommand& message )
{
  if( NodeId(message.addressee) == id() )
  {
    Q_D(GustUav); Q_ASSERT(d);
    
    if( ! d->stateMachine->configuration().contains(d->joystickControl) )
    {
      qCritical() << qualifiedName() << ": Received a joystick command but am not in joystickControl state.";
      return;
    }
    
    if( d->gust->isAlive())
    { d->gust->sendJoystick(message.joystick); }
    else
    {
      qCritical() << qualifiedName() << ": Received a joystick command but have no active GUST interface.";
    }
  }
}
