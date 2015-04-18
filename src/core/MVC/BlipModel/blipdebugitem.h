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

#ifndef BLIPDEBUGITEM_H
#define BLIPDEBUGITEM_H

#include "MVC/treeitem.h"

class BlipPrivate; // forward declaration

class BlipDebugItem : public TreeItem
{
public:
  BlipDebugItem(BlipPrivate const * blip, TreeItem* parent = nullptr);
  virtual ~BlipDebugItem();
  virtual QVariant data(int column, int role) const;

private:
  BlipPrivate const* d;
  
  QVariant name;
  QVariant comment;
  
//   TreeItem* fovPolyVisible = nullptr;
//   TreeItem* rfPolyVisible  = nullptr;
  TreeItem* comConnectedToHost = nullptr; ///< \brief Child #0 
  TreeItem* rfRelayFlag = nullptr;        ///< \brief Child #1 
  TreeItem* closestObstacle = nullptr;    ///< \brief Child #2 
  TreeItem* targetElement = nullptr;      ///< \brief Child #3 

};

#endif // BLIPDEBUGITEM_H
