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

#ifndef ACTIVITY_H
#define ACTIVITY_H

#include "state.h"

#include <QtCore/QFinalState>

/** \brief The Activity-State class.
 * An Activity is a State that mainly encapsulates a state of activity.
 * An Activity is something that "does work", or something that has some sort
 * of process, i.e. something that starts, is beeing worked on, but also can 
 * have an end and might need some non-trivial time to be completed.
 * 
 * Activities should actually \e do something to the object whose state machine
 * reaches an Activity state. This "work" is most likely attached to the
 * Activity through either of the following two methods:
 * 
 * - A Qt SIGNAL/SLOT connection to an "Activity Work Slots" (a protected slot)
 * - Subclassing Activity and (re-)implementing Activity::onEntry()
 * 
 * \note Maybe inserting a function pointer during instantiation might be a 
 *   third option? (But would that be much more elegant than the signal-slot
 *   method?)
 */
class Activity : public State
{
public:
    explicit Activity(StateId const stateId, State* parent);
    virtual ~Activity() = default;

    /** \brief The final state of the activity.
     * Reaching this state will trigger an emit(finished()).
     */
    QFinalState* done;
};

#endif // ACTIVITY_H
