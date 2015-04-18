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

#include "uav.h"

Uav::Uav(Settings* const settings)
  :GustUav(settings)
{

}

Uav::~Uav()
{

}


void Uav::initializeItem()
{
  GustUav::initializeItem(); 
#ifdef WITH_MATLABENGINE
  { initializeMatlabEngine(); }
#endif // WITH_MATLABENGINE
}

void Uav::startItem()
{
  GustUav::startItem();
}

void Uav::stopItem()
{
  GustUav::stopItem();
}

void Uav::unInitializeItem()
{
#ifdef WITH_MATLABENGINE
  { unInitializeMatlabEngine(); }
#endif // WITH_MATLABENGINE  

GustUav::unInitializeItem();
}

void Uav::timerEvent ( QTimerEvent* event )
{
  GustUav::timerEvent(event);
  
  const int timerId = event->timerId();

  if( timerId == metronome(1000) )
  { // 1 Hz stuff
#ifdef WITH_MATLABENGINE
    { useMatlabEngine(); }
#endif // WITH_MATLABENGINE
  }
}


#ifdef WITH_MATLABENGINE

void Uav::initializeMatlabEngine()
{
  matlabEngine = engOpen(NULL);
    
  if( !matlabEngine )
  { qCritical("%s : Cannot open the MATLAB engine.",
                objectName().toStdString().c_str() ); }; 
                
 
  // prepare an empty matrix to hold the ownship's track
  engEvalString(matlabEngine, "ownshipTrack = [];");
  
  // prepare a plot figure to show that track
  engEvalString(matlabEngine, "figure('Name','UAV Track');");
  engEvalString(matlabEngine, "axis equal;");
}


void Uav::useMatlabEngine()
{
  Q_ASSERT(matlabEngine);
  
  /* 
   * Create a variable for our data
   */
  mxArray* pos2d = mxCreateDoubleMatrix(1, 2, mxREAL);
  
  
  //NOTE: the UAV's (x,y,z) coordinates are given wrt. to the 
  // North/x-East/y-Down/y frame of reference.
  // Matlab's plot function assume the standart mathematical 
  // X/East-Y/North-Z/Up frame of reference...
  
  memcpy(static_cast<void*>(mxGetPr(pos2d)),
         static_cast<const void*>( &(ownship().position2D().y()) ),
         sizeof(ownship().position2D().x())) ;
         
  memcpy(static_cast<void*>(mxGetPr(pos2d)+1 ),
         static_cast<const void*>( &(ownship().position2D().x()) ),
         sizeof(ownship().position2D().y())) ;         
        
  /*
   * Place the variable "pos2d" into the MATLAB workspace
   */
  engPutVariable(matlabEngine, "pos2d", pos2d);
  
  // append the current position to the track
  engEvalString(matlabEngine, "ownshipTrack = [ownshipTrack; pos2d];");
  
  /*
   * Plot the result
   */
  engEvalString(matlabEngine, "plot(ownshipTrack(:,1),ownshipTrack(:,2),'-b',pos2d(1),pos2d(2),'+r');");
  
  /*
   * Clean up
   */
  mxDestroyArray(pos2d);
}

void Uav::unInitializeMatlabEngine()
{
  engClose(matlabEngine);
  matlabEngine = nullptr;
}

#endif // WITH_MATLABENGINE


