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

#include "activity.h"
#include <QtCore/QDebug>
#include <QtCore/QStateMachine>

Activity::Activity(const State::StateId stateId,
                   State* parent)
  :State(stateId,parent)
  ,done(new QFinalState(this))
{
  done->setObjectName(tr("Done"));
  setInitialState(done);
  
  connect(this,&Activity::finished,
    [this]()
    {
      qDebug() << machine()->parent()->objectName() 
        << ": Activity" << nestedStateName(this) << "has finished.";
      
    }
  );
}

