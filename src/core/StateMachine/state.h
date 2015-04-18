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

#ifndef STATE_H
#define STATE_H

#include <QtCore/QState>

#include "IDL/data.h"

class StateMachine; // forward declaration

/** \brief A custom State class for StateMachines.
 * This QState-derived class mainly adds some debug printouts on state entry and
 * exit.
 * 
 * \internal Q: Is the m_stateId really necessary? So far it is only used to set 
 * the State-object's name during construction.
 * 
 * \internal Q: Also, QState provides a QStatePrivate also, so maybe inherit 
 * this? A: No, Q...Private is not guaranteed to be consistend throughout Qt 
 * releases and hence compatability is not guaranteed. (Which is the whole point
 * of PIMPL--binary compatability throughout releases.)
 * 
 */
class State : public QState
{
  Q_OBJECT
  Q_PROPERTY(StateId m_stateId READ stateId)
  Q_ENUMS(StateId)
  
public:
  using StateId = IDL::Data::StateDescriptorId ;

  
  /** \brief Get the name of a state and all its parents.
   * 
   * \param[in] s The state whose name is wanted.
   * \return The fully nested name in the form of 
   *  [(parent's name)-> ... ->](state's name)
   */
  static QString nestedStateName(State const *  s);
  
public:
  explicit State(StateId const stateId, State * parent);
  explicit State(StateId const stateId, QState::ChildMode childMode, State* parent);
  virtual ~State() = default;
  
  State* parentState() const {return dynamic_cast<State*>(QState::parentState());}
  
  /** \brief Get the state identifyer of this state.
   * \sa IDL::Data::StateDescriptorId
   */
  StateId stateId() const {return m_stateId; };
  
  using QState::setObjectName;
  
  /** \brief Set the name of the State.
   * \overload
   * Translates the Id of the state into a name for it.
   * \param[in] stateId The ID of the State
   */
  void setObjectName(State::StateId stateId);
 
  /** \brief Enable or disable the printing of enty and exit debug messages. */
  void setDebugPrinting(bool const & flag);
    
  /** \brief Get the fully qualified name of this state.
   * \return State::nestedStateName(this);
   */
  QString qualifiedName() const;
  
protected:
  virtual void onEntry(QEvent* event);
  virtual void onExit(QEvent* event);
    
  
  StateId const m_stateId; ///< \brief The type id of this State.
  
private :
  bool printDebugOutput = true;

};

#endif // STATE_H
