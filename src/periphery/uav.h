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

#ifndef UAV_H
#define UAV_H

#include "config.h"
#include <Nodes/gustuav.h>

#ifdef WITH_MATLABENGINE
  #include "engine.h"
#endif // WITH_MATLABENGINE

/** \brief This is the class representing a UAV. */
class Uav : public GustUav {

public: 
  using Settings  = XML::SAI::UavNode_t ;

public:
  
  /** \brief Default constructor.
   * \note  A UAV is a standalone SimItem and as such doesn't have a 
   * (SimItem) parent.
   * \param[in] settings The defining settings for this UAV.   */
  Uav(Settings* const settings /*, SimItem* parent */);
  
  /** \brief Destructor. */
  virtual ~Uav();
   
protected:
  Q_DISABLE_COPY(Uav)
  
  virtual void initializeItem(); 
  virtual void startItem();
  virtual void stopItem();
  virtual void unInitializeItem();
  virtual void timerEvent ( QTimerEvent* event );
  
private:
  
#ifdef WITH_MATLABENGINE
  Engine* matlabEngine = nullptr;
  
  void initializeMatlabEngine();
  void useMatlabEngine();
  void unInitializeMatlabEngine();
  
#endif // WITH_MATLABENGINE
};

#endif // UAV_H
