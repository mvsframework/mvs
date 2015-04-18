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

#ifndef BLIPMODEL_H
#define BLIPMODEL_H

#include "MVC/treemodel.h"
#include <unordered_map>
#include "nodeid.h"

class Blip; // forward declaration

class BlipModel : public TreeModel {
  
public:
  /** \brief The underlying (STL) container holding the set of identical ContainerItems. */
  using DataContainer = std::unordered_map<NodeId,Blip*>;
  
public Q_SLOTS:
  /** \brief Add item to the model representation.*/
  void addBlipToModel(NodeId const & id);
  
  /** \brief Remove item from the model representation. */
  void removeBlipFromModel(NodeId const & id);

public:
  BlipModel(DataContainer * const data, QObject* parent = nullptr);
  virtual ~BlipModel();
  
  virtual Qt::DropActions supportedDragActions() const;
  virtual Qt::ItemFlags flags(QModelIndex const& index);
  virtual QMimeData* mimeData(const QModelIndexList &indexes) const;
  virtual QStringList mimeTypes() const;
};

#endif // BLIPMODEL_H
