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

#include "humanmachineinterface.h"
#include "humanmachineinterface_p.h"

//############################################################################//
//                                                                            //
//  HumanMachineInterfacePrivate                                                        //
//                                                                            //
//############################################################################//


HumanMachineInterfacePrivate::HumanMachineInterfacePrivate ( HumanMachineInterface* q
                                        ,HumanMachineInterface::Settings* const settings
                                        ,SimItem* parent)
  :SimItemPrivate(q,settings)
  ,q_ptr ( q )
  ,xmlSettings(settings)
{ //NOTE: Do _NOT_ use q_ptr or q in here, it _WILL_ break things!

}

HumanMachineInterfacePrivate::~HumanMachineInterfacePrivate()
{
}

void HumanMachineInterfacePrivate::unInitializeHumanMachineInterface()
{
  // undo whatever HumanMachineInterface::initializeItem() did...
}

void HumanMachineInterfacePrivate::loadJoystick()
{
//   Q_ASSERT(NULL != settings.configuredJoysticks);
  
  Q_Q(HumanMachineInterface); Q_ASSERT(q);
  
  using JoystickType = HumanMachineInterface::Settings::JoystickType;
  
  // get all the Joystick names from SDL
  qDebug() << q->qualifiedName()<< ": Detected the following connected joysticks:" << sdlJoystick.joystickNames;
  
  // iterate through all Joysticks configured in the settings.
  for( JoystickType& xmlJoystick : xmlSettings->joystick() )  //NOTE: this has to be reference as its address is taken below (and we don't want to take the address of a temp, do we?)
  {   
    int stick = 0;
    // iterate through all the joysticks that SDL detected
    for( QString sdlJoystickName : sdlJoystick.joystickNames )
    {      
      if( xmlJoystick.name() == sdlJoystickName.toStdString() )
      {
        qDebug() << q->qualifiedName()<< ": Using" << sdlJoystickName << "as joystick.";
        
        if( sdlJoystick.open(stick) )
        {  // successfully opened a joystick.
          activeJoystick = &(xmlJoystick); 
        
          // update some general joystick settings
          if( activeJoystick->autoRepeatDelay().present() )
          {
            sdlJoystick.autoRepeat = true;
            sdlJoystick.autoRepeatDelay = activeJoystick->autoRepeatDelay().get();
          }
          else
          {
            sdlJoystick.autoRepeat = false;
            sdlJoystick.autoRepeatDelay = SDL_JOYSTICK_DEFAULT_AUTOREPEAT_DELAY ;
          }
        }
        else
        {
          qCritical("\"%s\" : Can't open joystick %s.",
                    q->qualifiedName().toStdString().c_str(),
                    xmlJoystick.name().c_str());
        }
        
        break;
      }
      ++stick;
    }
   
    if( activeJoystick )
    { // successfully opened a joystick.
      
      // configure the button mapping
      qDebug() << q->qualifiedName()<< ": Joystick Button Mapping:";

      // iterate over all mapped buttons
      for( JoystickType::ButtonType button : activeJoystick->button() )
      { 
        buttonMap[button.action()] = button;
        
        if( button.action() == XML::SAI::ButtonAction_t::Value::extremes )
        { qDebug() << q->qualifiedName()<< ":\t"<< button << "≜ get the recorded axis extremes."; } //FIXME: Magic Numbers (text)
        if( button.action() == XML::SAI::ButtonAction_t::Value::dash )
        { qDebug() << q->qualifiedName()<< ":\t"<< button << "≜ engage DASH mode."; } //FIXME: Magic Numbers (text)
        if( button.action() == XML::SAI::ButtonAction_t::Value::altitudeHold )
        { qDebug() << q->qualifiedName()<< ":\t"<<button << "≜ altitude hold."; } //FIXME: Magic Numbers (text)
      }
     
      // connect the Qt signals to process joystick changes
      QObject::connect(&sdlJoystick, &Joystick::axisValueChanged ,
                       [this](int axis, int value){updateJoystickAxisValue(axis, value);} );
//                        this, &HumanMachineInterfacePrivate::updateJoystickAxisValue    );
      QObject::connect(&sdlJoystick, &Joystick::buttonValueChanged ,
                       [this](int button, int value){updateJoystickButtonValue(button, value);}  );
//                        this, &HumanMachineInterfacePrivate::updateJoystickButtonValue  );
      QObject::connect(&sdlJoystick, &Joystick::hatValueChanged ,
                       [this](int hat, int value){updateJoystickHatValue(hat, value);}  );
//                        this, &HumanMachineInterfacePrivate::updateJoystickHatValue  );
      QObject::connect(&sdlJoystick, &Joystick::trackballValueChanged ,
                       [this](int trackball, int deltaX, int deltaY){updateJoystickTrackballValue(trackball,deltaX,deltaY);} );
//                        this, &HumanMachineInterfacePrivate::updateJoystickTrackballValue );
      
      return;
    }
  }
}

void HumanMachineInterfacePrivate::updateJoystickAxisValue(const uint axis, const int value)
{
  Q_Q(HumanMachineInterface); Q_ASSERT(q);
  if( axis ==activeJoystick->roll().axis())
  {
    // record the raw extremes
    extremes.roll.max = std::max(extremes.roll.max, value); 
    extremes.roll.min = std::min(extremes.roll.min, value);
    // interpolate to a double
    currentJoystickData.axes.rollStick = interpolate(value,activeJoystick->roll());
//     qDebug("Roll: %5d -> %1.4f",value,currentJoystickData.axes.rollStick);
  }
  else if(axis ==activeJoystick->pitch().axis())
  {
    // record the raw extremes
    extremes.pitch.max = std::max(extremes.pitch.max, value); 
    extremes.pitch.min = std::min(extremes.pitch.min, value);
    // interpolate to a double
    currentJoystickData.axes.pitchStick = interpolate(value,activeJoystick->pitch());
//     qDebug("Pitch: %5d -> %1.4f",value,currentJoystickData.axes.pitchStick);
  }
  else if(axis ==activeJoystick->yaw().axis())
  {
    // record the raw extremes
    extremes.yaw.max = std::max(extremes.yaw.max, value); 
    extremes.yaw.min = std::min(extremes.yaw.min, value);
    // interpolate to a double
    currentJoystickData.axes.rudderPeddal = interpolate(value,activeJoystick->yaw());
//     qDebug("Yaw: %5d -> %1.4f",value,currentJoystickData.axes.rudderPeddal);
  }
  else if(axis ==activeJoystick->throttle().axis())
  {
    // record the raw extremes
    extremes.throttle.max = std::max(extremes.throttle.max, value); 
    extremes.throttle.min = std::min(extremes.throttle.min, value);
    // interpolate to a double
    currentJoystickData.axes.throttleLever = interpolate(value,activeJoystick->throttle());
//     qDebug("Throttle: %5d -> %1.4f",value,currentJoystickData.axes.throttleLever);
  }  
  else
  {
//     qDebug("Axis %d: %5d",axis,value);
  }

//   qDebug() << q->qualifiedName() << ": Joystick data changed.";
  emit q->joystickChanged(currentJoystickData);
}

void HumanMachineInterfacePrivate::updateJoystickButtonValue(const int button, const bool value)
{
  Q_ASSERT( button <= num_buttons );
  Q_Q(HumanMachineInterface); Q_ASSERT(q);
 
  if( value != oldButtonValue[button] )
  {
  
    if ( button == buttonMap[XML::SAI::ButtonAction_t::Value::dash] )
    {
      currentJoystickData.actions.dash = value;
      qDebug("\"%s\" : DASH mode %s.",
            q->qualifiedName().toStdString().c_str(),
            value ? "ON" : "OFF");
    }
    else if ( button == buttonMap[XML::SAI::ButtonAction_t::Value::altitudeHold] )
    {
      currentJoystickData.actions.altitudeHold = value;
      qDebug("\"%s\" : ALTITUDE HOLD %s.",
            q->qualifiedName().toStdString().c_str(),
            value ? "ON" : "OFF");
    }
    else if ( button == buttonMap[XML::SAI::ButtonAction_t::Value::extremes] )
    {
      if( value )
      {
        qDebug() << q->qualifiedName() << ": Recorded raw joystick extremes:\n"
          << "Roll    :" << extremes.roll.min << "–" << extremes.roll.max << "\n"
          << "Pitch   :" << extremes.pitch.min << "–" << extremes.pitch.max << "\n"
          << "Yaw     :" << extremes.yaw.min << "–" << extremes.yaw.max << "\n"
          << "Throttle:" << extremes.throttle.min << "–" << extremes.throttle.max ;
      }
    }
    else // button == buttonMap[XML::SAI::ButtonAction_t::Value::inop]
    {
      qDebug() << q->qualifiedName() << ": Button" << button << "has no assigned action.";
      oldButtonValue[button] = value;
    }
  
    oldButtonValue[button] = value;
    emit q->joystickChanged(currentJoystickData);
  }
}

void HumanMachineInterfacePrivate::updateJoystickHatValue(const int hat, const int value)
{
  Q_Q(HumanMachineInterface); Q_ASSERT(q);
  qDebug() << q->qualifiedName() << ": No hat switches for joysticks yet.";
}

void HumanMachineInterfacePrivate::updateJoystickTrackballValue(const int trackball, const int deltaX, const int deltaY)
{
  Q_Q(HumanMachineInterface); Q_ASSERT(q);
  qDebug() << q->qualifiedName() << ": No track ball interface yet.";
}

void HumanMachineInterfacePrivate::unloadJoystick()
{
  Q_Q(HumanMachineInterface); Q_ASSERT(q);
  if( nullptr != activeJoystick )
  { 
    qDebug() << q->qualifiedName() << ": Unloading" << QString(std::string(activeJoystick->name()).c_str());
    activeJoystick = nullptr;
  }
  
  //NOTE: Technically all these operations below should only make sense if there 
  // indeed is an active joystick. But since they _should_ be safe even if there
  // isn't one, it doesn't hurt performing these actions in case the 
  // settings.activeJoystick pointer simply isn't set correctly...
  
  buttonMap.clear();   
  sdlJoystick.close();
}


//############################################################################//
//                                                                            //
//  HumanMachineInterface                                                               //
//                                                                            //
//############################################################################//

/** \note This constructor does NOT do any work!
 * All HumanMachineInterface level construction needs to happen in 
 * HumanMachineInterface(HumanMachineInterfacePrivate & dd, ...)!
 */
HumanMachineInterface::HumanMachineInterface( Settings* const settings
                         ,SimItem* parent)
  :HumanMachineInterface(*new HumanMachineInterfacePrivate(this,settings,parent),parent)
{}

HumanMachineInterface::HumanMachineInterface( HumanMachineInterfacePrivate& dd
                         ,SimItem* parent )
  : SimItem(dd, parent) // HumanMachineInterfaces have SimItems as parents
{
  Q_D(HumanMachineInterface); Q_ASSERT(d);
  /** \internal 
   * This will only indicate the status of the HumanMachineInterface, any derived classes
   * will not be constructed at this point!
   */
  d->setStatus(Status::Constructed); 
}

HumanMachineInterface::~HumanMachineInterface()
{
  Q_D(HumanMachineInterface); Q_ASSERT(d);
  d->unInitializeHumanMachineInterface();
}

void HumanMachineInterface::initializeItem()
{
  SimItem::initializeItem();
  // do local work below ...
  
  Q_D(HumanMachineInterface); Q_ASSERT(d);
  for( int i=0; i<d->num_aux_channels; ++i)
  { d->currentJoystickData.axes.aux[i]      = 0; }
  d->currentJoystickData.axes.pitchStick    = 0;
  d->currentJoystickData.axes.rollStick     = 0;
  d->currentJoystickData.axes.rudderPeddal  = 0;
  d->currentJoystickData.axes.throttleLever = 0;
  d->currentJoystickData.actions.dash = false;
  d->currentJoystickData.actions.altitudeHold = true;
  
}

void HumanMachineInterface::startItem()
{
  SimItem::startItem();
  // do local work below ...
  
  Q_D(HumanMachineInterface); Q_ASSERT(d);
  d->loadJoystick();
}

void HumanMachineInterface::stopItem()
{
  Q_D(HumanMachineInterface); Q_ASSERT(d);
  d->unloadJoystick();
  
  // do local work above...
  SimItem::stopItem();
}

void HumanMachineInterface::unInitializeItem()
{
  Q_D(HumanMachineInterface); Q_ASSERT(d);
  d->unInitializeHumanMachineInterface();
  
  // do local work above...
  SimItem::unInitializeItem();
}

bool HumanMachineInterface::isJoystickLoaded() const
{
  Q_D(const HumanMachineInterface); Q_ASSERT(d);
  return (nullptr != d->activeJoystick);
}

#include "humanmachineinterface.moc"
