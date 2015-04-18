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

#ifndef PATH_PRIVATE_H
#define PATH_PRIVATE_H

#include <QtCore/qglobal.h>

#include "config.h"

#ifdef WITH_GRAPHICS
  #include <QtWidgets/QGraphicsItem>
  #include <QtWidgets/QMenu>
#endif // WITH_GRAPHICS

class Path; // Forward declaration.
class PathPrivate
{
private:
  Path* const q_ptr;
  Q_DECLARE_PUBLIC(Path)

public:
  PathPrivate(Path* q);
  virtual ~PathPrivate();

#ifdef WITH_GRAPHICS  
  QGraphicsItem* graphicsItem = nullptr;
#endif WITH_GRAPHICS
  
};


#endif // PATH_PRIVATE_H