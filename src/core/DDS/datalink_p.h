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

#ifndef DATALINKPRIVATE_H
#define DATALINKPRIVATE_H

#include "simitem_p.h"

#include <QtCore/QDebug>
#include <QtCore/QHash>
#include <QtCore/QString>


#include <ndds/ndds_cpp.h>

#include "IDL/messages.h"
#include "IDL/messagesSupport.h"
#include "DDS/ddssupport.h"
#include "DDS/validatingdatareaderlistener.h"
#include "DDS/subscriberlistener.h"


#include "datalink.h"
#include "Nodes/node.h"
#include "Nodes/node_p.h"
// class Node;

class Datalink;

class DatalinkPrivate : public SimItemPrivate
{
private:
  Q_DECLARE_PUBLIC(Datalink);
  Datalink* const q_ptr;
  
  static DDS_DomainId_t const  domainId = 0; //FIXME: Magic Numbers.
 
public:
  DatalinkPrivate(Datalink* q, Datalink::Settings* const settings, Node* parent);
  virtual ~DatalinkPrivate();

  Datalink::Settings * const xmlSettings; // initialized in construction list
  
  /** \brief The DDS participant.
   * DomainParticipant objects enable an application to exchange messages within 
   * domains. An application must have a DomainParticipant for every domain in 
   * which the application will communicate.(Unless your application is a bridging
   * application, it will typically participate in only one domain and have only 
   * one DomainParticipant.) 
   * DomainParticipants are used to create Topics, Publishers, DataWriters, 
   * Subscribers, and DataReaders in the corresponding domain.  
   */
  DDSDomainParticipant*     participant = nullptr;
  
  /** \brief A DDS Publisher. */
  DDSPublisher*             publisher = nullptr;
  
  /** \brief A DDS Subscriber. */
  DDSSubscriber*            subscriber = nullptr;    
  
  /** \brief A custom listener attached to the subscriber. */
  SubscriberListener*       subscriberListener = nullptr;
  
  /** \brief The Quality-of-Service settings of the participant.
   * \sa DDSDomainParticipant::get_qos();
   */
  DDS_DomainParticipantQos participant_qos;
    
  /** \brief The globally unique Id of a datalink participant. */
  DDS_GUID_t*               guid = nullptr;

  void initializeDatalink();
  
  void unInitializeDatalink();

  
  
  /** \brief A listener detecting Subscriptions by other participants. */
  NodePrivate::BuiltInTopic_Subscriptions_Listener*  builtInTopic_Subscriptions_Listener = nullptr;
  
  /** \brief A listener detecting Publications by other participants. */
  NodePrivate::BuiltInTopic_Publications_Listener*   builtInTopic_Publications_Listener  = nullptr;
  
  /** \brief A listener detecting the presence of other participants. */
  NodePrivate::BuiltInTopic_Participant_Listener*    builtInTopic_Participant_Listener  = nullptr;
  
  /** \brief A listener directly attached to the domain participant. */
  NodePrivate::Participant_Listener* participant_listener = nullptr;
  
  void attachToBuiltInTopic(DDSDataReaderListener* listener,
                            char const * topicName,                            
                            DDS_StatusMask mask = DDS_DATA_AVAILABLE_STATUS );
   
  


};


#endif // DATALINKPRIVATE