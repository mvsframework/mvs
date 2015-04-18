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

#ifndef BROADCASTTOPIC_H
#define BROADCASTTOPIC_H

#include "DDS/topic.h"

/** \brief A topic container for broadcast messages.
 * 
 * \tparam Message The IDL::Messages type this broadcast topic is using.
 * \tparam ParentNodeType The Node derived class that introduces this topic.
 * \tparam options The TopicOptions for this Topic.
 */
template<typename Message
        ,typename ParentNodeType
        ,TopicOptions options
        >
class BroadcastTopic : public Topic<Message,ParentNodeType,options>
{
public:
  /** \brief Constructor.
   * 
   * \param[in] name The name of the topic.
   * \param[in] datalink The DDS datalink to use.
   * \param[in] callbackNode The Node reacting to received topic messages.
   */
  BroadcastTopic(QString const & name, Datalink* const  datalink, 
                 ParentNodeType* const callbackNode)
    :Topic<Message,ParentNodeType,options>(name, datalink, callbackNode)
  {};
  
  /** \brief Destructor. */
  virtual ~BroadcastTopic() = default;

  /** \brief Get access to the message struct holding the data to be send. */
  Message * message()
  { return &defaultMessageStruct; }
  
  /** \brief Send the currently held message. */
  bool send()
  {
    // these messages _shouldn't_ have keys, hence the NIL handle is OK.
    // TODO: check for the absence of any keys?
    return Topic<Message,ParentNodeType,options>::send(defaultMessageStruct, DDS_HANDLE_NIL );
  };
    
  
private:
  Message defaultMessageStruct;
};


#endif // BROADCASTTOPIC_H