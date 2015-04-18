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

#ifndef STATEMACHINE_H
#define STATEMACHINE_H

#include <QtCore/QStateMachine>
#include "IDL/data.h"

class SimItem; // forward declaration
class StateMachinePrivate; // forward declaration

class StateMachine : public QStateMachine
{
  Q_OBJECT
  
public:
  StateMachine(SimItem* parent = nullptr);
  virtual ~StateMachine();
  
  SimItem* parent();
  
  /** \brief Get the state with the corresponding id.
   * \note This function currently uses Qt's \e findChild and it's execution 
   * might be (computationally) costly.
   */
  QAbstractState* state(IDL::Data::StateDescriptorId const & id);
  
protected:
  QScopedPointer<StateMachinePrivate> const d_ptr;
  
  /** \brief Constructor optimized for d-pointer inheritance.
   * See http://qt-project.org/wiki/Dpointer for more explanations.
   */
  StateMachine(StateMachinePrivate& dd, SimItem* parent = nullptr);

private:
  Q_DISABLE_COPY(StateMachine)
  Q_DECLARE_PRIVATE(StateMachine)
};

#endif // STATEMACHINE_H
