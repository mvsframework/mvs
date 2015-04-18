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

#ifndef ATTITUDEITEM_H
#define ATTITUDEITEM_H

#include <amg.hpp>
#include "MVC/treeitem.h"


class AttitudeItem : public TreeItem {
public:
  AttitudeItem(AMG::EulerAngleTupel const & eulerAngles, QString const & name = "Orientation"
              , QString const & comment = "", TreeItem* parent = nullptr);
  virtual ~AttitudeItem();
  
  virtual QVariant data ( int column, int role = Qt::DisplayRole ) const;
  
private:
  AMG::EulerAngleTupel const & attitude;
  QVariant name;
  QVariant comment;
  QVariant type;
  TreeItem* phi = nullptr;
  TreeItem* theta = nullptr;
  TreeItem* psi = nullptr;
  
  QString lookingAtString() const;
};

#endif // ATTITUDEITEM_H
