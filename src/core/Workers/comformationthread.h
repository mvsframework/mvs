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
//  -----
//  
//  This class is based upon a Qt tutorial available at
//  http://qt-project.org/doc/qt-5/qtcore-mandelbrot-example.html
//  
   

#ifndef COMFORMATIONTHREAD_H
#define COMFORMATIONTHREAD_H

#include <QtCore/QThread>
#include <QtCore/QWaitCondition>

#include "Nodes/node.h"


class ComFormationThread : public QThread
{
Q_OBJECT  

Q_SIGNALS: 
  void comFormationProposed();
  
public:
  /** \brief Constructor. */
  ComFormationThread(Node* parent=nullptr);
  
  /** \brief Destructor. */
  ~ComFormationThread();
  
  /** \brief Propose a formation to connect a source Node to a target Element.
   * \details This method triggeres the computations to propose a new COM 
   * formation. When the computation is finished, the comFormationProposed 
   * signal is emitted.
   * \param[in] target The Id of the target obstacle
   * \param[in] source The Node that should be the source for the proposed 
   * formation.
   */
  void proposeComFormation(const ElementId& target, const NodeId& source);

protected:
  
  virtual void run();  
  
private:
  /** \brief A mutex to protect \a all private data members. */
  QMutex mutex;
  QWaitCondition condition;
  bool abort = false;
  bool restart = false;

  ElementId target;
  NodeId source;
    
};

#endif // COMFORMATIONTHREAD_H
