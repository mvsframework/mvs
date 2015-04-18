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

#include "obstacle.h"
#include "obstacle_p.h"


//############################################################################//
//                                                                            //
//  ObstaclePrivate                                                           //
//                                                                            //
//############################################################################//

ObstaclePrivate::ObstaclePrivate ( Obstacle* q )
  :ElementPrivate(q)
  ,q_ptr ( q )
  ,obstacleGraphVertex(ObstacleGraph::Traits::null_vertex())
{//NOTE: Do _NOT_ use q_ptr or q in here, it _WILL_ break things!
 
}


ObstaclePrivate::~ObstaclePrivate()
{

  
}



//############################################################################//
//                                                                            //
//  Obstacle                                                                  //
//                                                                            //
//############################################################################//

/** \note This constructor does NOT do any work!
 * All Obstacle level construction needs to happen in 
 * Obstacle(ObstaclePrivate& dd, ...)!
 */
Obstacle::Obstacle(QObject* parent)
  :Obstacle(*new ObstaclePrivate( this ), parent )
{}

Obstacle::Obstacle(ObstaclePrivate& dd, QObject* parent)
  : Element(dd, parent)
{

}



Obstacle::~Obstacle()
{

}

ObstacleGraph::Vertex_Descriptor Obstacle::obstacleGraphVertex() const
{
  Q_D(const Obstacle); Q_ASSERT(d);
  return d->obstacleGraphVertex;
}

void Obstacle::setObstacleGraphVertex(ObstacleGraph::Vertex_Descriptor const & descriptor)
{
  Q_D(Obstacle); Q_ASSERT(d);
  d->obstacleGraphVertex = descriptor;
}





#include "obstacle.moc"