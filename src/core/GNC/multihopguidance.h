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

#ifndef MULTI_HOP_GUIDANCE_H
#define MULTI_HOP_GUIDANCE_H

#include <unordered_map>

#include "config.h"
#include "nodeid.h"
#include "Geometry/geometry.h"
#include "GNC/waypoint.h"

#ifdef WITH_GRAPHICS
  #include "Graphics/geometryitem.h"
#endif // WITH_GRAPHICS

/** \brief A POD structure holding data related to the Dual Hop Guidance.
 * \details This POD struct holds data that a 
 */
struct DualHopGuidanceData
{
  DualHopGuidanceData()
    :centroid(0.0,0.0)
    ,target(0.0,0.0)
  {
    Geometry::Point_2D center(0,0);
    auto circle = Geometry::makeCircle<Geometry::Polygon_2D::ring_type,100>(center,10); //FIXME: Magig Numbers
    boost::geometry::assign(targetArea,circle );
    

#ifdef WITH_GRAPHICS
    targetAreaItem = new Graphics_2D::Polygon(targetArea);
    centroidItem = new Graphics_2D::Point(centroid);
    targetItem = new Graphics_2D::Point(target);
#endif //WITH_GRAPHICS
  };
  
  ~DualHopGuidanceData()
  {
#ifdef WITH_GRAPHICS
    delete targetAreaItem;
    targetAreaItem = nullptr;
    
    delete centroidItem;
    centroidItem = nullptr;
    
    delete targetItem;
    targetItem = nullptr;
#endif // WITH_GRAPHICS
  };
  
    
  /** \brief A potential set making up the intersection of the R/F polygons. */
  std::deque<Geometry::Polygon_2D> rfIntersection;
  Geometry::Polygon_2D targetArea;
  Geometry::Point_2D centroid;
  Geometry::Point_2D target;
  
#ifdef WITH_GRAPHICS
  Graphics_2D::Polygon* targetAreaItem = nullptr;
  Graphics_2D::Point* centroidItem = nullptr;
  Graphics_2D::Point* targetItem = nullptr;
#endif //WITH_GRAPHICS
  
  QMetaObject::Connection tracking_head; // the connection to track position updates of the head Node
  QMetaObject::Connection tracking_tail; // the connection to track position updates of the tail Node
};



#endif // MULTI_HOP_GUIDANCE_H
