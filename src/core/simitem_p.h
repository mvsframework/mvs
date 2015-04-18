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

#ifndef SIMITEMPRIVATE_H
#define SIMITEMPRIVATE_H

#include "simitem.h"

#include <QtCore/QObject>

class SimItem;

class SimItemPrivate  {

private:
  Q_DECLARE_PUBLIC(SimItem);
  SimItem* const q_ptr;  
  
public:
  SimItemPrivate(SimItem* q, SimItem::Settings* settings);
  virtual ~SimItemPrivate();
  
  SimItem::Settings* const xmlSettings; // initialized in construction list
   
  SimItem::Status currentStatus = SimItem::Status::Undefined;
  
  StateMachine* stateMachine = nullptr;
  
  
  State * running = nullptr; ///< \brief The absolute root state (ParallelStates)
  State * running_itemRegion = nullptr; ///< \brief A Qt necessity to reflect the item's orthogonal region.

  /** \brief Initialize the state machine of the SimItem.
   * Each SimItem provides a state machine for added functionality of the item.
   * However, when a SimItem is a child of another item, the child item uses 
   * the state machine of the parent, making the parent automatically aware of 
   * the states (modes) of all it's childs.
   */
  void initializeStateMachine();
    
  /** \brief Initialize the SimItem specific states of the state machine. */
  void initializeStates();
  
  /** \brief Check whether a chance from currentStatus to newStatus is permissible.
   * In addition to identity operations, the following status changes from the
   * current status to a new status are possible:
   * 
   * - Undefined   -> Constructed  
   * - Constructed -> Initialized  : \sa SimItem::initialize();
   * - Initialized -> Running      : \sa SimItem::run();
   * - Running     -> Initialized  : \sa SimItem::halt();
   *
   * \param[in] newStatus The desired new status to check for.
   */
  bool isStatusChangePossible(const SimItem::Status& newStatus) const;
  
  /** \brief Change the item's status.
   * Sets the currentStatus to a new status and emits 
   * SimItem::changedStatus(newStatus).
   * 
   * \note The function does \e not perfrom any test on the status change. Use
   * isStatusChangePossible() to do that before calling setStatus()
   * 
   * \param[in] newStatus The value currentStatus is set to.
   */
  void setStatus(SimItem::Status const & newStatus);
  
  /** \brief Un-initialize the SimItem. */
  void unInitializeSimItem();
  
//   struct TrackedQtConnections {
//   } qtConnections;

};

#endif // SIMITEMPRIVATE_H
