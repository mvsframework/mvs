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

#ifndef BLIPCONTAINER_H
#define BLIPCONTAINER_H

#include "MVC/treeitem.h"
#include <unordered_map>
#include "nodeid.h"
#include "blip.h"

class BlipModel; //forward declaration

class BlipContainer : public TreeItem {
public:
 
  using DataContainer = std::unordered_map<NodeId,Blip*>;

  BlipContainer(DataContainer * const container , BlipModel* const model);
  virtual ~BlipContainer();
                            
  virtual QVariant data ( int column, int role = Qt::DisplayRole ) const;

  TreeItem* child(NodeId const & id);
  virtual int childCount() const;
  
  bool insertBlip( NodeId const & id, int position = 0);
  bool removeBlip( NodeId const & id);
  
  virtual QAbstractItemModel* model() const;
  void setModel(QAbstractItemModel* model);
  
private:
  DataContainer*  const blipData;
  BlipModel* const parentModel = nullptr;
  QVector<QVariant> headers;  
};

#endif // BLIPCONTAINER_H
