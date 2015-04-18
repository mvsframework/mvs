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

#include "Graphics/palette.h"
#include <boost/concept_check.hpp>
#include <random>

namespace Graphics_2D { 
  
// QPen   const    Pens::rfPolygon(Outline(Qt::darkGreen));
// QBrush const Brushes::rfPolygon(Qt::NoBrush);

namespace Colors {

QColor const comGraph(Qt::darkGreen);  
QColor const routeGraph(Qt::lightGray);
QColor const obstacleGraph(Qt::blue);
QColor const activeFlightplan(Qt::magenta);
QColor const inactiveFlightplan(Qt::darkMagenta);
QColor const targetElement(Qt::darkBlue);
QColor const nonTargetElement(Qt::darkRed);

} // end namespace Colors

} // end namespace Graphics_2D