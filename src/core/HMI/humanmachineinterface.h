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

#ifndef HUMANMACHINEINTERFACE_H
#define HUMANMACHINEINTERFACE_H


#include "simitem.h"

#include "IDL/data.h"

class HumanMachineInterfacePrivate; // forward declaration

/** \brief SimItem to interface with a (human) operator.
 * 
 * This class is intended to be the main interface to all mechanisms to ineract 
 * with a human operator (besides the keyboard and the screen).
 * 
 * So for now, this is just a glorified joystick interface.
 */
class HumanMachineInterface : public SimItem{
Q_OBJECT
  
public:
  using Settings = XML::SAI::HumanMachineInterface_t; //NOTE: Change to something like XML::SAI:HumanMachineInterface_t

Q_SIGNALS:
  /** \brief Indicating a change in the joystick, axis values or buttons. 
   * 
   * This signal is emitted whenever the joystick configuration has changed,
   * i.e. an axis moved or a button has been pressed or released. This "push"
   * approach hides the continuous polling of the joystick from the user but 
   * maintains a fairly low delay between physical action and (potential) 
   * reaction in software.
   */
  void joystickChanged(IDL::Data::JoystickData /*currentJoystickData*/);
  
public:
  explicit HumanMachineInterface( Settings* const settings
                       ,SimItem* parent);
  virtual ~HumanMachineInterface();
  
  /** \brief Check whether or not a Joystick is currently loaded.
   * \return TRUE if a joystick is loaded an usable; FALSE otherwise.
   */
  bool isJoystickLoaded() const;

protected:
  explicit HumanMachineInterface ( HumanMachineInterfacePrivate& dd, SimItem* parent = nullptr );
  
  virtual void initializeItem();
  virtual void startItem();
  virtual void stopItem();
  virtual void unInitializeItem();

private:
  Q_DISABLE_COPY(HumanMachineInterface)
  Q_DECLARE_PRIVATE(HumanMachineInterface);

};

#endif // HUMANMACHINEINTERFACE_H
