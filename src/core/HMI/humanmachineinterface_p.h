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

#ifndef HUMANMACHINEINTERFACE_PRIVATE_H
#define HUMANMACHINEINTERFACE_PRIVATE_H

#include "simitem_p.h"


#include "HMI/joystick.h"
#include "IDL/data.h"

class HumanMachineInterface;

/** \todo Remove the QObject depdency and replace the connections with lambdas.
 */
class HumanMachineInterfacePrivate : public SimItemPrivate {
  
private:
  Q_DECLARE_PUBLIC(HumanMachineInterface);
  HumanMachineInterface* const q_ptr;
  
public:
  HumanMachineInterfacePrivate( HumanMachineInterface* q
                     ,HumanMachineInterface::Settings* const settings
                     ,SimItem* parent  );
  virtual ~HumanMachineInterfacePrivate();
     
  XML::SAI::HumanMachineInterface_t* const xmlSettings;
  XML::SAI::HumanMachineInterface_t::JoystickType* activeJoystick = nullptr;
  
  
  IDL::Data::JoystickData currentJoystickData;
  
  static const int num_aux_channels = 2; //FIXME: Magic Numbers. See IDL::Data::JoystickAxes.aux and Joystick_t in sai.xsd
  static const int num_buttons = 16; //FIXME: Magic Numbers.  See Joystick_t maxOccurs in sai.xsd
  bool oldButtonValue[num_buttons] = {false}; //FIXME: initialize the bool array with the initial values of the joystick.
  
  QHash<XML::SAI::ButtonAction_t::Value,int> buttonMap;
    
  /** \brief A Qt wrapper for the raw SDL joystick interface. */
  Joystick sdlJoystick;
  
  /** \brief True if the interpolation is bounded by [min,max]. */
  bool limitJoystickToBounds;
  
  /** \brief A struct to keep track of the extremes of the raw joystick outputs. */
  struct RawJoystickExtremes{
    struct MinMax {
      int min = 0;
      int max = 0;
    };
    MinMax roll;
    MinMax pitch;
    MinMax yaw;
    MinMax throttle;
  } extremes;

  
  
  void unInitializeHumanMachineInterface();

  /** \brief Load and initialize the connected joystick(s). */
  void loadJoystick();
  
  /** \brief Unload the current joystick. */
  void unloadJoystick();
  
  void updateJoystickAxisValue( const uint axis, const int value );
  void updateJoystickButtonValue(const int button, const bool value);
  void updateJoystickHatValue(const int hat, const int value);
  void updateJoystickTrackballValue(const int trackball, const int deltaX, const int deltaY);
  
  
  /** \brief A helper function to interpolate the axis values.
   * \tparam InType
   * \tparam OutType (optional)
   * \param[in] value
   * \param[in] map
   * \param[in] maxOut (optional)
   * \param[in] minOut (optional)
   * \return The interpolated value between minOut and maxOut
   */
  template<typename InType, typename OutType=float >
  OutType interpolate(InType value, XML::SAI::AxisMapping_t& map,OutType maxOut=1.0, OutType minOut=-1.0)
  {
    OutType centerOut = minOut+(maxOut-minOut)/static_cast<OutType>(2);
    OutType offset;
    
    XML::SAI::RawAxisValue_t mapCenter = 0; //NOTE: 
    
    if( map.center().present() )
    { mapCenter = map.center().get(); }
    
    // Do the interpolation
    if( value >= mapCenter )
    {
      offset =  (value-mapCenter)/static_cast<OutType>( map.max()-mapCenter );
      offset = centerOut + offset*(maxOut-centerOut) ;
      if( limitJoystickToBounds )
      { offset = std::min(maxOut,offset); }
    }
    else
    {
      offset =  (value-map.min())/static_cast<OutType>( mapCenter-map.min() );
      offset =  minOut + offset*(centerOut-minOut);
      if( limitJoystickToBounds )
      { offset = std::max(minOut,offset); }
    }
    
    // consider the reversal
    if( map.reverse().present() and map.reverse().get() == true)
    { offset *= static_cast<OutType>(-1); }
    
    return offset;
  };

};

#endif // HUMANMACHINEINTERFACE_PRIVATE_H
