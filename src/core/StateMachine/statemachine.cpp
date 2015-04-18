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
#include "statemachine.h"
#include "statemachine_p.h"

#include "simitem.h"
#include "IDL/idlsupport.h"

//############################################################################//
//                                                                            //
//  StateMachinePrivate                                                       //
//                                                                            //
//############################################################################//

StateMachinePrivate::StateMachinePrivate(StateMachine* q)
  :q_ptr(q)
{//NOTE: Do _NOT_ use the q_ptr in here, it _WILL_ break things!
}

StateMachinePrivate::~StateMachinePrivate()
{
}

//############################################################################//
//                                                                            //
//  StateMachine                                                              //
//                                                                            //
//############################################################################//

/** \note This constructor does NOT do any work!
 * All StateMachine level construction needs to happen in 
 * StateMachine( StateMachinePrivate& dd, ...)!
 */
StateMachine::StateMachine(SimItem* parent)
  :StateMachine(*new StateMachinePrivate(this),parent)
{}

StateMachine::StateMachine(StateMachinePrivate& dd, SimItem* parent)
  :QStateMachine(parent)
  ,d_ptr(&dd)
{

}


StateMachine::~StateMachine()
{
}

SimItem* StateMachine::parent()
{
  return static_cast<SimItem*>(QObject::parent());
}

QAbstractState* StateMachine::state ( const IDL::Data::StateDescriptorId& id )
{
  return findChild<QAbstractState*>(enumToQString(id));
}



#include "statemachine.moc"
