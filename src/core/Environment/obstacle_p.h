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

#ifndef OBSTACLEPRIVATE_H
#define OBSTACLEPRIVATE_H

#include "Environment/element_p.h"

#include "Graph/obstaclegraph.h"

class Obstacle; // forward declaration

class ObstaclePrivate : public ElementPrivate {

public:
  ObstaclePrivate ( Obstacle* q );
  virtual ~ObstaclePrivate();

  /** \brief The vertex descriptor for this blip in the COM graph. */
  ObstacleGraph::Vertex_Descriptor obstacleGraphVertex;
  
private:
  Q_DECLARE_PUBLIC(Obstacle);
  Obstacle* const q_ptr;
};

#endif // OBSTACLEPRIVATE_H
