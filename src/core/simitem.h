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

#ifndef SIMITEM_H
#define SIMITEM_H

#include <iostream>

#include <QtCore/QObject>
#include <QtCore/QString>
#include <QtCore/QDebug>
#include <XSD/sai.hxx>

#include "StateMachine/statemachine.h"
#include "StateMachine/state.h"

class SimItemPrivate; // Forward declaration.

/** \brief The SimItem.
 * 
 * \internal
 * This class is trying to follow the "PIMPL" (Private 
 * IMPLementation) scheme for C++. Here are some related references:
 * * http://qt-project.org/wiki/Dpointer
 * * http://techbase.kde.org/Policies/Library_Code_Policy#D-Pointers
 * 
 * \todo Mutlithreading: Would it be beneficial to move every SimItem into it's 
 * own thread? How about at least the ones that don't have a parent? Can the 
 * SimItem::run() method be used as it naturally overlaps with QThread::run()?
 * 
 */
class SimItem : public QObject
{
  Q_OBJECT
  Q_PROPERTY(Status status READ status NOTIFY statusChanged)
  Q_ENUMS(Status)

public:
  /** \brief A collection of status levels SimItems can have. */
  enum class Status { 
    Undefined   = 0,  ///< \brief Initial status upon instantiation of a SimItem.
    Constructed = 1,  ///< \brief Item is fully constructed, ready for initialization.
    Initialized = 2,  ///< \brief Item is initialized, ready to be started or re-initilized. Can be started.
    Running     = 3   ///< \brief Item is actively running in the sim. Can be stopped.
  }; //NOTE: When changing this enum, update enumToQString() below
  
  using Settings = XML::SAI::SimItem_t;
  
Q_SIGNALS:
  /** \brief This signal is emitted when the status of a SimItem has changed. */
  void statusChanged(Status);

public Q_SLOTS:  
  
  /** \brief (Re-)Initialize the SimItem.
   * This (slot) function at the SimItem level triggers the complete 
   * initialization of any (derived) SimItem.
   * 
   * This is done through calling the virtual protected function 
   * initializeItem(). Calling this virtual function results in calling the
   * function object in the furtherst derived class of the SimItem, making it
   * necessary that this function internally always calls the initializeItem()
   * of it's base class first. This combination achieves a top-down 
   * initialization, triggered through a call to this slot.
   */
  void initialize();  
  
  /** \brief Start the simulation of the SimItem.
   * 
   * SimItem::run() primarily results in starting the state machine of the 
   * item.
   * 
   * \note The QStateMachine  machine will reset its configuration and 
   * transition to the initial state. 
   * 
   * \todo What does "starting" actually mean in the context of a distributed 
   *  simulation?
   */
  void run();    
  
  /** \brief Stop the simulation of the SimItem.  
   * 
   * SimItem::halt() primarily results in stoping the state machine of the 
   * item.
   * 
   * \todo What does "stoping" actually mean in the context of a distrubuted 
   *  simulation? The (sim-)clock of the other nodes keeps on ticking...
   */
  void halt();        
  
public:
  
  /** \brief Get the SimItem parent */
  SimItem* parent() const;        

  /** \brief Get the fully qualified name of this item.
   * 
   * This function gets the name of the item prepended with the name of the
   * parent items, i.e. something like
   * 
   * parent->objectName() + "/" + ... + this->objectName();
   * 
   * \sa QObject::objectName()
   */
  QString qualifiedName() const;
  
  /** \brief Get the current status of the item. */
  Status status() const;
  
  /** \brief Check whether or not the SimItem has been initialized.
   * \return TRUE is the SimItem has been initialized, FALSE otherwise.
   */
  bool isInitialized();
  
  /** \brief Check whether or not the SimItem is currently running.
   * \return TRUE is the SimItem is currently running, FALSE otherwise.
   */
  bool isRunning();
  
  /** \brief Get the state machine used by the SimItem.
   * If the SimItem is a child of another SimItem, the parent's state machine
   * is returned.
   */
  StateMachine*        stateMachine();
//   StateMachine const * stateMachine() const;

protected:
  QScopedPointer<SimItemPrivate> const d_ptr;
  
  /** \brief Default (empty) constructor. */
  SimItem(Settings * const Settings, SimItem* parent=nullptr);

  /** \brief Destructor. */
  virtual ~SimItem();
  
  /** \brief Constructor optimized for d-pointer inheritance.
   * See http://qt-project.org/wiki/Dpointer for more explanations.
   */
  explicit SimItem(SimItemPrivate & dd, SimItem* parent=nullptr);
  
  /** \brief Performs the work necessary to initialize a SimItem.
   * \note When reimplementing this function in a derived class, make sure to 
   * call the base class version \e before you do the derived class' work.
   * \sa initialize()
   */
  virtual void initializeItem();
  
  /** \brief Performs the work necessary to start a SimItem.
   * \note When reimplementing this function in a derived class, make sure to 
   * call the base class version \e before you do the derived class' work.
   * \sa run()
   */
  virtual void startItem();
  
  /** \brief Performs the work necessary to stop a SimItem.
   * \note When reimplementing this function in a derived class, make sure to 
   * call the base class version \e after you do the derived class' work.
   * \sa halt()
   */
  virtual void stopItem();
  
  /** \brief Performs the work necessary to un-initialize a SimItem.
   * \note When reimplementing this function in a derived class, make sure to 
   * call the base class version \e after you do the derived class' work.
   */
  virtual void unInitializeItem();
  
private:
  Q_DISABLE_COPY(SimItem)
  Q_DECLARE_PRIVATE(SimItem)
  
  /** \brief Private and undefined to dissallow usage.
   * As SimItems potentially share stateMachines with their children, changing
   * parents during runtime could be harmfull. 
   */
  void setParent(SimItem * parent);

};

//############################################################################//
//                                                                            //
//  I/O Support                                                               //
//                                                                            //
//############################################################################//

std::ostream& operator<<(std::ostream& out, SimItem::Status const & status);
QDebug operator<<(QDebug dbg,  SimItem::Status const & status);



#endif // SIMITEM_H
