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

#ifndef SUBSCRIBERLISTENER_H
#define SUBSCRIBERLISTENER_H

#include <ndds/ndds_cpp.h>



/** \brief Subscriber Listener.
 * \note The simple presence of the callbacks doesn't mean that they indeed are
 * triggered. Check the DDS_StatusMask associated with each listener to ensure 
 * that all callbacks implemented are actually triggered, too.
 * This normally is handled by a reader->set_listener(listener,mask) routine.
 */
class SubscriberListener : public DDSSubscriberListener {
public:
  
  /** \brief A status mask capturing all implemented callback routines. */
  static const DDS_StatusMask implementedCallbacks = 0
    | DDS_DATA_ON_READERS_STATUS
    | DDS_DATA_AVAILABLE_STATUS
    ;
  
  SubscriberListener() = default;
  ~SubscriberListener() = default;
  virtual void on_data_on_readers ( DDSSubscriber* sub )
  {
    /** \todo I would be great if the actual message receivabiliyt could be 
     * solved in here, but so far I couldn't figure out a way to get access to
     * the sample data (mainly the sample type and the sender key) and hence I
     * couldn't do anything... :(
     */
    sub->notify_datareaders(); // this _should_ trigger a call to on_data_available()...
  };
  
  virtual void on_data_available ( DDSDataReader* reader ) {};
};

#endif // SUBSCRIBERLISTENER_H
