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

#ifndef OBSTACLE_H
#define OBSTACLE_H

#include "Environment/element.h"

#include "Graph/obstaclegraph.h"

class ObstaclePrivate; // forward declaration

class Obstacle : public Element {
Q_OBJECT
    
public:
  /** \brief Default constructor. */
  Obstacle(QObject* parent = nullptr);
  
  /** \brief Destructor. */
  virtual ~Obstacle();

  ObstacleGraph::Vertex_Descriptor obstacleGraphVertex() const;
  virtual void               setObstacleGraphVertex(ObstacleGraph::Vertex_Descriptor const & descriptor);
  
protected:
  /** \brief Constructor optimized for d-pointer inheritance.
   * See http://qt-project.org/wiki/Dpointer for more explanations.
   */
  explicit Obstacle( ObstaclePrivate& dd, QObject* parent = nullptr );
    
private:
//   Q_DISABLE_COPY(Obstacle)
  Q_DECLARE_PRIVATE(Obstacle)
};


//############################################################################//
//                                                                            //
//  I/O Support                                                               //
//                                                                            //
//############################################################################//

inline QDebug operator<<(QDebug dbg, const Obstacle& obstacle )
{
  dbg.nospace() << "\nOBJECT Obstacle::" << obstacle.objectName();
  
  { // dump properties
    dbg.nospace() << "\n  PROPERTIES";
    dbg.nospace() << "\n    id: " << obstacle.id();
  }
  
#ifdef WITH_GRAPHICS
  { // dump graphics
    dbg.nospace() << "\n  PROPERTIES (GRAPHICS)";
    dbg.nospace() << "\n    shapeVisibility: " << obstacle.isShapeVisible();      
    dbg.nospace() << "\n    convexHullVisibility: "<< obstacle.isConvexHullVisible();
  }
#endif // WITH_GRAPHICS

  { // dump other
    dbg.nospace() << "\n  OTHER";
    dbg.nospace() << "\n    obstacle graph vertex: " << obstacle.obstacleGraphVertex();
  }
  
  return dbg.space();
};




#endif // OBSTACLE_H
