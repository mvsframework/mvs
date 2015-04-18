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

#ifndef STATEMACHINEPRIVATE_H
#define STATEMACHINEPRIVATE_H

class StateMachine; // forward declaration

class StateMachinePrivate 
{
public:
  StateMachinePrivate(StateMachine* q);
  virtual ~StateMachinePrivate();
  
private:
  Q_DECLARE_PUBLIC(StateMachine);
  StateMachine* const q_ptr;
};

#endif // STATEMACHINEPRIVATE_H
