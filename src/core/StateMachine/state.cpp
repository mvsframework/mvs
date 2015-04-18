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


#include "StateMachine/state.h"

#ifndef QT_NO_DEBUG_OUTPUT 
  #include <QtCore/QDebug>
  #include <QtCore/QStateMachine>
#endif

// #include "StateMachine/statemachine.h"
#include "simitem.h"
#include "IDL/idlsupport.h"


//############################################################################//
//                                                                            //
//  local stuff (anonymous namespace )                                        //
//                                                                            //
//############################################################################//
namespace {



} // end anonymous namespace



//############################################################################//
//                                                                            //
//  State                                                                     //
//                                                                            //
//############################################################################//



State::State(const State::StateId stateId,
             State* parent)
  :State(stateId,QState::ExclusiveStates,parent)
{
}

State::State(const State::StateId stateId,
             QState::ChildMode childMode,
             State* parent)
  :QState(childMode,parent)
  ,m_stateId(stateId)
{
  setObjectName(stateId);
//   qDebug() << objectName() << ": my parentState is" << parentState() ;
}

//NOTE: the idea was to use machine()->parent()->qualifiedName()... but that
// is always a root item, hence this is a useless restriciton...
//
// StateMachine* State::machine()
// {
//   StateMachine* m = dynamic_cast<StateMachine*>(QState::machine());
//   Q_ASSERT(m);
//   return m;
// }


void State::setObjectName ( State::StateId stateId )
{
  QState::setObjectName(enumToQString(stateId));
}


void State::setDebugPrinting ( const bool& flag )
{
  printDebugOutput = flag;
}


void State::onEntry(QEvent* event)
{
  Q_UNUSED( event );
  
  //NOTE: The parent() is injected as all these messages by default come from
  // the state machines of the respecitve parent of the state machine.
  // Hence it wouldn't make much sense to print name()+"'s StateMachine" ...
  // Also note the although "qualifiedName()" could be used (if the parent
  // is a SimItem) machine()->parent() by design is always a root item as 
  // children use their parent's machine....
  if( printDebugOutput )
  {
    qDebug() << machine()->parent()->objectName() 
    << ": Entering state" << nestedStateName(this) << " <==";
  }
}

void State::onExit(QEvent* event)
{
  Q_UNUSED( event );
  
    //NOTE: The parent() is injected as all these messages by default come from
  // the state machines of the respecitve parent of the state machine.
  // Hence it wouldn't make much sense to print name()+"'s StateMachine" ....
  if( printDebugOutput )
  {
    qDebug() << machine()->parent()->objectName() 
    << ": Exiting state " << nestedStateName(this) <<" ==>";
  }
  
}

QString State::nestedStateName ( const State* s )
{
  Q_ASSERT(s);
  
  // Print out the state we are entering and it's parents
  QString nestedName = s->objectName();
  
  State* ps =  s->parentState() ;
  
  while( ps != nullptr)
  { 
    nestedName = ps->objectName() + "->" + nestedName; 
    ps =  ps->parentState();
  }
  
  return nestedName;   
}



QString State::qualifiedName() const
{
  return nestedStateName(this);
}

#include "state.moc"
