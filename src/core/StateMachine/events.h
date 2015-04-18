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

#ifndef EVENTS_H
#define EVENTS_H

#include <QtCore/QEvent>

#include "IDL/data.h"
#include "GustInterface/datalinkmessages.h"

namespace Events {
  
/** \brief Custom Events for use with the StateMachine
  * In order ot avoid magic numbers, this enum records the custom event types
  * necessary for subclassing QEvent.
  * \sa QEvent::registerEventType()
  */
enum class Types {
   CommandType        = QEvent::User+1 ///< \brief An event encapsulating (user) commands.
  ,GustTrajectoryType = QEvent::User+2 ///< \brief A GUST TrajectoryStatus change.
  ,GustGuidanceType   = QEvent::User+3 ///< \brief A change in the GUST guidance mode.
                                       ///<  Which is indicated in GUST::M1SafemodeStatus.
};


struct UserCommand : public QEvent
{
  UserCommand(IDL::Commands::CommandTypes const & command)
    :QEvent(QEvent::Type(Types::CommandType))
    ,command(command)
  {};
  
  IDL::Commands::CommandTypes  command;
};

struct GustTrajectory : public QEvent 
{
  GustTrajectory(GUST::TrajectoryStatus const & trajStatus)
    :QEvent(QEvent::Type(Types::GustTrajectoryType)) 
    ,status(trajStatus)
  {};
  
  GUST::TrajectoryStatus status;
};

struct GustGuidance : public QEvent
{
  GustGuidance(GUST::M1_Safemode_Status const & guidanceStatus)
    :QEvent(QEvent::Type(Types::GustGuidanceType))
    ,status(guidanceStatus)
  {};
  
  GUST::M1_Safemode_Status status;
};

} // end namespace Events

#endif // EVENTS_H