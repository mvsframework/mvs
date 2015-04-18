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

#ifndef ADDRESSEDTOPIC_H
#define ADDRESSEDTOPIC_H

#include <utility>
#include <unordered_map>

#include <QtCore/QMutex>

#include "DDS/topic.h"

#include "IDL/dataPlugin.h"
#include "IDL/dataSupport.h"

#include "IDL/messagesPlugin.h"
#include "IDL/messagesSupport.h"



/** \brief A topic container for adressed messages.
 * 
 * \pre The Message type must be a subclass of IDL::Messages::AddressedMessage.
 * 
 * \tparam Message The IDL::Messages type this addressed topic is using. 
 * \tparam ParentNodeType The Node derived class that introduces this topic.
 * \tparam options The TopicOptions for this Topic.
 */
template< typename Message
        , typename ParentNodeType
        , TopicOptions options
        >
class AddressedTopic : public Topic<Message,ParentNodeType,options>
{
public:
  /** \brief Constructor.
   * 
   * \param[in] name The name of the topic.
   * \param[in] datalink The DDS datalink to use.
   * \param[in] callbackNode The Node reacting to received topic messages.
   */
  AddressedTopic(QString const & name, Datalink* const  datalink, 
                 ParentNodeType* const callbackNode)
    :Topic<Message,ParentNodeType,options>(name, datalink, callbackNode)
  {};
  
  /** \brief Destructor. */
  virtual ~AddressedTopic() = default;

  /** \brief Get access to the message struct holding the data to be send. */
  Message * message()
  { return &messageStruct; };
  
 
  /** \brief Register an addressee and get the assoc. message instance handle. */
  DDS_InstanceHandle_t registerAddressee(NodeId const & id)
  {
    if( messageInstanceHandles.count(id) == 0 )
    { 
      DDS_InstanceHandle_t handle = Topic<Message,ParentNodeType,options>::writer()->register_instance(messageStruct);
      messageInstanceHandles.emplace(std::make_pair(id,handle));
      return handle;
    }
    // else
    return messageInstanceHandles.at(id);
  };
  
  /** \brief Register an addressee and delete the assoc. message instance handle. */
  DDS_ReturnCode_t unregisterAddressee(NodeId const &id)
  {
    // This is default return code when stuff goes wrong in unregister_instance.
    // See http://community.rti.com/rti-doc/500/ndds.5.0.0/doc/html/api_cpp/structFooDataWriter.html#aec82ad5e0bd1803708a48b9f1759822d
    DDS_ReturnCode_t retcode = DDS_RETCODE_BAD_PARAMETER;
    if( messageInstanceHandles.count(id) > 0 )
    {
      DDS_InstanceHandle_t handle = messageInstanceHandles.at(id);
      retcode = Topic<Message,ParentNodeType,options>::writer()->unregister_instance(messageStruct,handle); 
      if( DDS_RETCODE_OK == retcode)
      { messageInstanceHandles.erase(id); }
    }
    return retcode;
  };
    
  /** \brief Send the currently held message (keeping the current addressee). */
  virtual bool send()
  {
    QMutexLocker locker(&mutex);
    DDS_InstanceHandle_t handle = registerAddressee(messageStruct.addressee);
    
    return Topic<Message,ParentNodeType,options>::send( messageStruct, handle );
  };
  

    
  
private:
  QMutex mutex;
  Message messageStruct;
  DDS_InstanceHandle_t messageHandle;
  
  /** \brief A hash to keep the registered message handles. */
  std::unordered_map<NodeId, DDS_InstanceHandle_t> messageInstanceHandles;
  
 };


#endif // ADDRESSEDTOPIC_H