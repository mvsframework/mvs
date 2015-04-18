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

#ifndef POSITIONITEM_H
#define POSITIONITEM_H

#include "MVC/treeitem.h"
#include <amg.hpp>

class PositionItem : public TreeItem {
public:
  PositionItem( AMG::Vector const & vector, QString const & name = "Position"
              , QString const & comment = "", TreeItem* parent = nullptr);
  virtual ~PositionItem();
  
  virtual QVariant data ( int column, int role = Qt::DisplayRole ) const;
private:
  AMG::Vector const & position;
  QVariant name;
  QVariant comment;
  QVariant type;
  TreeItem* x = nullptr;
  TreeItem* y = nullptr;
  TreeItem* z = nullptr;
  
  QString geodeticPositionString() const;
  QString relativePositionString() const;
};

#endif // POSITIONITEM_H
