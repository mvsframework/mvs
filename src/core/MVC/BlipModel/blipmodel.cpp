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

#include "MVC/BlipModel/blipmodel.h"
#include "MVC/BlipModel/blipcontainer.h"
#include "blip.h"

#include <QtCore/QMimeData>

BlipModel::BlipModel(DataContainer * const data, QObject* parent)
  :TreeModel(new BlipContainer(data, this))
{
  
}

BlipModel::~BlipModel()
{
  delete rootItem;
}

void BlipModel::addBlipToModel ( const NodeId& id )
{
  auto blipContainer = static_cast<BlipContainer*>(rootItem);
  
  int position = 0;
  
  beginInsertRows(QModelIndex(),position,position);
  blipContainer->insertBlip(id,position);// insert at the top
  endInsertRows();
}

void BlipModel::removeBlipFromModel ( const NodeId& id )
{
  auto blipContainer = static_cast<BlipContainer*>(rootItem);
  int position =blipContainer->child(id)->childNumber();
  beginRemoveRows(QModelIndex(),position,position);
  blipContainer->removeBlip(id);
  endRemoveRows();
}

Qt::DropActions BlipModel::supportedDragActions() const
{
  return supportedDropActions();
}

Qt::ItemFlags BlipModel::flags(const QModelIndex& index)
{
  Qt::ItemFlags defaultFlags = TreeModel::flags(index);
  
  if( index.isValid() /*and !index.parent().isValid()*/ )
  {
    return Qt::ItemIsDragEnabled | defaultFlags;
  }
  else
  { return defaultFlags; }
}

QStringList BlipModel::mimeTypes() const
{
  QStringList types = QAbstractItemModel::mimeTypes();
  types << "NodeId";
  return types;
}


QMimeData* BlipModel::mimeData(const QModelIndexList& indexes) const
{
  if( indexes.length() != 1 )
  { return QAbstractItemModel::mimeData(indexes); }
  else
  {
    QMimeData *mimeData = new QMimeData();
    
    QModelIndex index = indexes.first();
    
    if( index.isValid() and !index.parent().isValid() )
    {      
      QByteArray byteArray = data(index, Qt::UserRole).toByteArray();
      mimeData->setData("NodeId", byteArray);
    }
    return mimeData;
  }
}


