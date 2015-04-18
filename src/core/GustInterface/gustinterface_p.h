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

#ifndef GUSTINTERFACEPRIVATE_H
#define GUSTINTERFACEPRIVATE_H

#include "simitem_p.h"

#include <QtCore/QTimer>
#include <QtCore/QDebug>
#include <QtNetwork/QUdpSocket>

#include <amg.hpp>

#include "GustInterface/datalinkmessages.h"
#include "GustInterface/debuganderrormessages.h"
#include "GustInterface/checksum.h"

#include "StateMachine/state.h"
#include "StateMachine/events.h"

class GustInterface;

class GustInterfacePrivate : public SimItemPrivate {
private:
  Q_DECLARE_PUBLIC(GustInterface);
  GustInterface* const q_ptr;  
  
public:
  GustInterfacePrivate( GustInterface* q, GustInterface::Settings*const settings, Node* parent  );
  virtual ~GustInterfacePrivate();
     
  GustInterface::Settings* const xmlSettings;
     
  void unInitializeGustInterface();
  
  //
  // The GustInterface-specific states for the SimItem's state machine.
  //
  
  State* connectionInactive = nullptr;
  State* connectionActive = nullptr;
  
  State* trajectoryStatus_gustInterfaceRegion = nullptr; ///< \brief A Qt necessity to reflect the item's orthogonal region.
  
  State* trajectoryStatusInitial = nullptr;
  State* trajectoryStatusCruise = nullptr;
  State* trajectoryStatusStop = nullptr;
  
  State* guidanceStatus_gustInterfaceRegion = nullptr; ///< \brief A Qt necessity to reflect the item's orthogonal region.
  
  State* guidanceStatusGo = nullptr;
  State* guidanceStatusStop = nullptr;
  
  //
  // network related stuff
  //
  
  
  QUdpSocket*   udpSocketSend = nullptr;    ///< \brief The Tx UDP socket of the Interface
  QUdpSocket*   udpSocketReceive = nullptr; ///< \brief The Rx UDP socket of the Interface
  
  QHostAddress  gustHostAddress; ///< \brief The address of the host providing a GUST instance
  
  /** \brief The UDP address the GUST instance is listening at
   * The GUST default is 2010.
   */
  qint16        gustHostUdpPort = 2010;  
  
  QHostAddress  localHostAddress; ///< \brief The address of the host providing this GustQInterface
  
  /** \brief The address this GustInterfacePrivate is listening at for GUST messages
   * The GUST default is 3010.
   */
  qint16        localUdpPort = 3010;
  
  //
  // other
  //
  
  /** \brief Get the next spare byte for GUST message headers.
  * \details As of SVN revision 11638 the spare byte needs to change inbetween 
  * messages as it is used to discriminate duplicate sendings of the same
  * message against a legitimate new message (which might have the same content,
  * though).
  * 
  * \return A spare byte that hasn't been used before by a GustInterface in 
  * this compilation unit.
  */ 
  static unsigned char getNextSpareByte()
  {
    static unsigned char spareByte=0;
    return ++spareByte;
  };
  
  
  /** \brief Receive counter for the different message types.
   * This is a pure data collection counter which uses int, the type of
   * datalinkHeader_ref::messageID, as a hash and holds the number of received
   * messages of that type.
   */
  QHash<int, uint32_t> receivedMessageCount; 
  
  QTimer* gustConnectionTimer = nullptr; ///< \brief A timer to check if the esim connection is still alive.
  
  /**  \brief Max. allowed time inbetween GUST messages. [ms] */
  static const int gustTimeOut = 1000; //FIXME : Magic Numbers
  
//   bool gustConnectionAlive = false; ///< \brief Status of the datalink connection to the GUST instance.

  
  /** \brief A struct holding the last received datalink messages.
   */
  struct DatalinkMessages {
    GUST::datalinkMessage0_ref m0;
    GUST::datalinkMessage1_ref m1;
    GUST::datalinkMessageFlightPlan_ref flightPlan; ///< \brief The m11 flight plan message.
  } lastReceivedMessages;
  
  AMG::FrameOfReference* gustDatum = nullptr; ///< \brief The Frame of Reference for GUST coordinates
  AMG::CoordinateTupel positionCoords; ///< \brief The last known position
  AMG::CoordinateTupel velocityCoords; ///< \brief The last known velocity
  AMG::QuaternionTupel quaternions; ///< \brief The last know attitude

  
  
  void initializeNetwork();
  void initializeStates();
  
  void connectSockets();
  void disConnectSockets();

  void restartConnectionTimer();
  
  
  /** \brief Send a message via UDP to the connected GUST instance.
   * \tparam MessageType Any of the message types in the GUST namespace
   * \param message The message struct to be send, e.g. GUST::datalinkMessageUp0_ref
   * \return The number of bytes sent if successful, -1 otherwise. 
   */
  template<typename MessageType>
  qint64 sendUDPMessage(MessageType message);
  
  template<typename MessageType>
  void processReceivedMessage(char* data);

};


//
// Template Implementations and Specializations
// ============================================
//
template<typename MessageType>
inline qint64 GustInterfacePrivate::sendUDPMessage ( MessageType message )
{
  unsigned char* data = reinterpret_cast<unsigned char*>(&message);
  
  size_t utilizedSize = std::min(sizeof(MessageType),
                                  static_cast<size_t>(message.messageSize));
  
  
  if( utilizedSize > 512 )
  {
    //NOTE: This warning message is related to the Qt documentation of
    //  udpSocket::writeDatagram(...).
    qWarning("\"%s\" : Trying to send a UDP datagram of size %lu. Qt documentation: \"Sending datagrams larger than 512 bytes is generally disadvised,[...]\""
      ,"GustInterface"
      ,utilizedSize);
  }
  
  GUST::datalinkCheckSumEncode(data, utilizedSize);
  
  char* qtData = reinterpret_cast<char*>(&message);
  size_t bytesSent = udpSocketSend->writeDatagram(qtData,utilizedSize,gustHostAddress,gustHostUdpPort);
  
  if( bytesSent != utilizedSize )
  { qCritical("\"%s\" : Failed to send datagram correctly.", "Gust Interface"); }
  
  return bytesSent;
}


/** \brief Primary Message Processing Template: no processing. */
template<typename MessageType>
inline void GustInterfacePrivate::processReceivedMessage(char* data)
{
  Q_Q(GustInterface); Q_ASSERT(q);
  
  auto m = reinterpret_cast<MessageType*>(data);
  qDebug() << q->qualifiedName() << ": Received a message via UDP (MessageID =" 
    << QString("%1").arg(m->messageID,3) << "). No further processing.";
}

//NOTE: the inline is necessary to avoid a multiple definition problem during the linking stage.
/** \brief Process a received m0 message. */
template<>
inline void GustInterfacePrivate::processReceivedMessage<GUST::datalinkMessage0_ref>(char* data)
{
  auto m0 = reinterpret_cast<GUST::datalinkMessage0_ref*>(data);
//   qDebug("Processing a Message0...");

  for( int i=0; i<3; ++i)
  {
    positionCoords[i] = AMG::Units::feet2meter(m0->pos[i]);
    velocityCoords[i] = AMG::Units::feet2meter(m0->vel[i]); // implicit fps -> m/s conversion
    quaternions[i] = m0->q[i];
  }
  
  // and the 4th quaternion

  quaternions[3] = m0->q[3];
  
  // update the messages-received buffer
  lastReceivedMessages.m0 = *m0;
  
  if( gustDatum != nullptr)
  {
    Q_Q(GustInterface); Q_ASSERT(q);
    emit q->stateVectorUpdateAvailable(); // only emit the signal if a datum to make sense of the coordinates is known!
  }
}

//NOTE: the inline is necessary to avoid a multiple definition problem during the linking stage.
/** \brief Process a received m1 message. */
template<>
inline void GustInterfacePrivate::processReceivedMessage<GUST::datalinkMessage1_ref>(char* data)
{
  using AMG::Units::feet2meter;
  using AMG::Units::degree2radian;
  
  
  auto newData = reinterpret_cast<GUST::datalinkMessage1_ref*>(data);
//   qDebug("Processing a Message1...");
  
  // grab a copy of the old current data for comparison...
  auto oldData = lastReceivedMessages.m1;
  // ... update the current data with the one just received so that the 
  // subsequent emits (i.e. the connected slots) can access the new data already
  lastReceivedMessages.m1= *newData;
  
  
  //
  // Process Datum changes...
  // 
  Q_Q(GustInterface); Q_ASSERT(q);
  if( nullptr == gustDatum )
  {
    // create the gustDatum as a NED on the WGS84...
    gustDatum = AMG::CoSy::newNorthEastDown<AMG::Units::degree>(newData->datumLon,newData->datumLat);
    // ... and move it "up" (i.e.  minus "down" direction) to match the GUST datum
    gustDatum->translate(0.0,0.0,-1*feet2meter(newData->datumAlt));
    
    qDebug() << q->qualifiedName() << ": Created an interface internal frame of reference matching the GUST datum location.";
    
    emit q->datumUpdateAvailable();
  }
  else if( oldData.datumLat != newData->datumLat or
           oldData.datumLon != newData->datumLon or
           oldData.datumAlt != newData->datumAlt    )
  {
    // get the origin of GUST's datum frame as an AMG::Vector
    double x,y,z;
    AMG::GIS::geodetic2ecef(degree2radian(newData->datumLon),
                            degree2radian(newData->datumLat),
                            feet2meter(newData->datumAlt),
                            x,y,z);
    AMG::Vector gustDatumOrigin(x,y,z,AMG::CoSy::getECEF());
        
    // reposition the EsimInterface::datum if they don't match
    gustDatum->setPositionOfOrigin(gustDatumOrigin-gustDatum->positionOfOrigin()); 
    
    qDebug() << q->qualifiedName() << ": Repositioned the interface internal datum to match the GUST datum location.";
    emit q->datumUpdateAvailable();
  }
  
  //
  // Process trajectory status
  // =========================
  // NOTE: unfortunately   
  //   stateMachine->postEvent(new StateMachine::Events::GustTrajectoryEvent(newData->traj_status));
  //   wont work because of a char-ENUM conversion issue...
  //   Hence all the trouble with the switch.
  // TODO: maybe the new class enum could solve this?
  
  if( oldData.traj_status != newData->traj_status )
  {
    switch( newData->traj_status )
    {
      case GUST::TRAJ_STATUS_CRUISE :
        stateMachine->postEvent(new Events::GustTrajectory(GUST::TRAJ_STATUS_CRUISE));
        break;
      case GUST::TRAJ_STATUS_STOP :
        stateMachine->postEvent(new Events::GustTrajectory(GUST::TRAJ_STATUS_STOP));
        break;
      case GUST::TRAJ_STATUS_INITIAL :
        stateMachine->postEvent(new Events::GustTrajectory(GUST::TRAJ_STATUS_INITIAL));
        break;
      default:
        qCritical() << q->qualifiedName() << ": detected an unkown \"traj_status=\" in the GUST datalink messages:" << static_cast<int>(newData->traj_status);
        break;
    }
  }
  
  if( oldData.safemode != newData->safemode )
  { 
    using namespace GUST;
    switch( newData->safemode )
    {
      case M1_Safemode_Status::go :     
      case M1_Safemode_Status::go2 :    
      case M1_Safemode_Status::slow :   
      case M1_Safemode_Status::gcas :   
      case M1_Safemode_Status::gcas2 :  
      case M1_Safemode_Status::gcasNoe :
      case M1_Safemode_Status::panic :
      case M1_Safemode_Status::avoid :  
        stateMachine->postEvent(new Events::GustGuidance(M1_Safemode_Status::go));
        break;
      case M1_Safemode_Status::stop :   
      case M1_Safemode_Status::holding :
        stateMachine->postEvent(new Events::GustGuidance(M1_Safemode_Status::stop));
        break;
      default:
        qCritical() << q->qualifiedName() << ": detected an unkown \"safemode=\" in the GUST datalink messages:" << static_cast<int>(newData->traj_status);
        break;
    }  
  }
  
  if( oldData.traj_manIndex != newData->traj_manIndex )
  { emit q->currentWaypointChanged(static_cast<unsigned int>(newData->traj_manIndex)); }
  
}


template<>
inline void GustInterfacePrivate::processReceivedMessage<GUST::datalinkMessageFlightPlan_ref>(char* data)
{
  auto m11 = reinterpret_cast<GUST::datalinkMessageFlightPlan_ref*>(data);
  
  lastReceivedMessages.flightPlan = *m11;
  
  Q_Q(GustInterface); Q_ASSERT(q);
//   qDebug() << q->qualifiedName() << ": Received a flight plan:\n" << lastReceivedMessages.flightPlan;
  
  emit q->flightPlanReceived();
}



#endif // GUSTINTERFACEPRIVATE_H
