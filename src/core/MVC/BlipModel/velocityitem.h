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

#ifndef VELOCITYITEM_H
#define VELOCITYITEM_H

#include "MVC/treeitem.h"
#include <amg.hpp>

class VelocityItem : public TreeItem {
public:
  VelocityItem(AMG::Vector const & velocity, QString const & name = "Orientation"
              , QString const & comment = "", TreeItem* parent = nullptr);
  virtual ~VelocityItem();

  virtual QVariant data ( int column, int role = Qt::DisplayRole ) const;
  
private:
  AMG::Vector const & velocity;
  QVariant name;
  QVariant comment;
  TreeItem* xDot = nullptr;
  TreeItem* yDot = nullptr;
  TreeItem* zDot = nullptr;
  
  QString speedString() const;
};

#endif // VELOCITYITEM_H
