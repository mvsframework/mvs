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
//  ----
// 
//  This is a blunt copy-past from http://www.batcom-it.net/?p=59, 
//  posted in 2008 by René Reucher 
//  
//  

#include "joystick.h"

Joystick::Joystick( QObject* parent, bool doAutoRepeat, int autoRepeatDelay, int joystickEventTimeout )
  : QObject(parent)
//   ,numAxes(0)
  ,numberOfAxes(numAxes)
//   ,numButtons(0)
  ,numberOfButtons(numButtons)
//   ,numHats(0)
  ,numberOfHats(numHats)
//   ,numTrackballs(0)
  ,numberOfTrackballs(numTrackballs)
  ,eventTimeout(joystickEventTimeout)
  ,autoRepeat(doAutoRepeat)
  ,autoRepeatDelay( autoRepeatDelay )
{
  if ( SDL_Init(SDL_INIT_JOYSTICK) == 0 ) {
    for (int i = 0; i < SDL_NumJoysticks(); i++)
      joystickNames.append(SDL_JoystickName(i));
    connect(&joystickTimer, SIGNAL(timeout()), this, SLOT(processEvents()));
  } else {
    qCritical("ERROR: couldn't initialize SDL joystick support");
  }
  
}

Joystick::~Joystick()
{
  if ( isOpen() )
    close();

  SDL_Quit();
}

bool Joystick::open(int stick)
{
  if ( isOpen() )
    close();

  joystick = SDL_JoystickOpen(stick);
  if ( joystick ) {
    numAxes = SDL_JoystickNumAxes(joystick);
    numButtons = SDL_JoystickNumButtons(joystick);
    numHats = SDL_JoystickNumHats(joystick);
    numTrackballs = SDL_JoystickNumBalls(joystick);
    joystickTimer.start(eventTimeout);
    processEvents();
    return true;
  } else {
    qCritical("ERROR: couldn't open SDL joystick #%d", stick);
    return false;
  }
}

void Joystick::close()
{
  joystickTimer.stop();
  if ( joystick )
    SDL_JoystickClose(joystick);
  joystick = NULL;
  numAxes = numButtons = numHats = numTrackballs = 0;
}

void Joystick::processEvents()
{
  if ( !isOpen() )
    return;

  SDL_JoystickUpdate();

  int i;
  for (i = 0; i < numAxes; i++) {
    Sint16 moved = SDL_JoystickGetAxis(joystick, i);
    if ( abs(moved) >= deadzones[i] ) {
      if ( (moved != axes[i]) ) {
        int deltaMoved = abs(axes[i] - moved);
        if ( deltaMoved >= sensitivities[i] )
          emit axisValueChanged(i, moved);
        axes[i] = moved;
        axisRepeatTimers[i].restart();
      } else if (autoRepeat && moved != 0) {
        if ( axisRepeatTimers[i].elapsed() >= autoRepeatDelay ) {
          emit axisValueChanged(i, moved);
          axes[i] = moved;
        }
      } else
        axisRepeatTimers[i].restart();
    } /*else
      emit axisValueChanged(i, 0);*/
  }
  
  for (i = 0; i < numButtons; i++) {
    Uint8 changed = SDL_JoystickGetButton(joystick, i);
    if ((changed != buttons[i]) ) {
      emit buttonValueChanged(i, (bool) changed);
      buttons[i] = changed;
      buttonRepeatTimers[i].restart();
    } else if (autoRepeat && changed != 0) {
      if ( buttonRepeatTimers[i].elapsed() >= autoRepeatDelay ) {
        emit buttonValueChanged(i, (bool) changed);
        buttons[i] = changed;
      }
    } else
      buttonRepeatTimers[i].restart();
  }
  
  for (i = 0; i < numHats; i++) {
    Uint8 changed = SDL_JoystickGetHat(joystick, i);
    if ( (changed != hats[i]) ) {
      emit hatValueChanged(i, changed);
      hats[i] = changed;
      hatRepeatTimers[i].restart();
    } else if (autoRepeat && changed != 0) {
      if ( hatRepeatTimers[i].elapsed() >= autoRepeatDelay ) {
        emit hatValueChanged(i, changed);
        hats[i] = changed;
      }
    } else
      hatRepeatTimers[i].restart();
  }

  for (i = 0; i < numTrackballs; i++) {
    int dx, dy;
    SDL_JoystickGetBall(joystick, i, &dx, &dy);
    if ( dx != 0 || dy != 0 )
      emit trackballValueChanged(i, dx, dy);
  }
}

int Joystick::getAxisValue(int axis)
{
  if ( isOpen() ) {
    SDL_JoystickUpdate();
    return SDL_JoystickGetAxis(joystick, axis);
  } else
    return 0;
}






// #include "joystick.moc"


