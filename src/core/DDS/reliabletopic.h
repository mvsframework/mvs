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

#ifndef RELIABLETOPIC_H
#define RELIABLETOPIC_H

#include "DDS/addressedtopic.h"


/** \brief A topic container for relably delivered messages.
 * 
 * This topic is used (can be used) for messages that need to make it to the
 * addressee(s); e.g. commands.
 * 
 * The setup follows section 10.3 of the RTI Users Manual.
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
class ReliableTopic : public AddressedTopic<Message,ParentNodeType,options>
{
public:
  /** \brief Constructor.
   * 
   * \param[in] name The name of the topic.
   * \param[in] datalink The DDS datalink to use.
   * \param[in] callbackNode The Node reacting to received topic messages.
   */
  ReliableTopic(QString const & name, Datalink* const  datalink, 
                 ParentNodeType* const callbackNode)
    :AddressedTopic<Message,ParentNodeType,options>(name, datalink, callbackNode)
  {};
 
  
private:
  using AT = AddressedTopic<Message,ParentNodeType,options>;
  
  virtual void modifyWriterQos(DDS_DataWriterQos & qos)
  {
    qos.reliability.kind = DDS_RELIABLE_RELIABILITY_QOS;
            
    // reliability.max_blocking_time defaults to 100 ms
    qos.reliability.max_blocking_time  = DDS_Duration_t::from_millis(200);
    
    // history.depth defaults to 1 (and it needs to be kept in sync with
    // DDS_ReseourceLimitQosPolicy::max_samples_per_instance, see API 
    // description)
//       qos.history.depth = 10;
  };
  
  virtual void modifyReaderQos(DDS_DataReaderQos & qos)
  {
    qos.reliability.kind = DDS_RELIABLE_RELIABILITY_QOS;
                      
    // reliability.max_blocking_time defaults to 100 ms
    qos.reliability.max_blocking_time  = DDS_Duration_t::from_millis(200);
    
    // history.depth defaults to 1 (and it needs to be kept in sync with
    // DDS_ReseourceLimitQosPolicy::max_samples_per_instance, see API 
    // description)
//       qos.history.depth = 10;  
  };
  
  
};


#endif // RELIABLETOPIC_H