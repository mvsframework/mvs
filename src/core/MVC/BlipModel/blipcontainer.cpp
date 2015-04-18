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

#include "blipcontainer.h"

#include <QtCore/QObject>
#include <QtCore/QDebug>
#include "blip.h"
#include "IDL/idlsupport.h"

#include "MVC/BlipModel/blipmodel.h"

BlipContainer::BlipContainer( BlipContainer::DataContainer*const container, BlipModel* const model )
  :TreeItem()
  ,blipData(container)
  ,parentModel(model)
{
  using ColumnMap = TreeItem::DataAspect;
  
  headers.resize(numberOfColumns);
  headers[static_cast<int>(ColumnMap::name)]     = enumToQString(ColumnMap::name);
  headers[static_cast<int>(ColumnMap::value)]    = enumToQString(ColumnMap::value);
  headers[static_cast<int>(ColumnMap::comment)]  = enumToQString(ColumnMap::comment);
  headers[static_cast<int>(ColumnMap::type)]     = enumToQString(ColumnMap::type);
  
  
  for( auto& pair : *blipData )
  {
    TreeItem* blipItem = pair.second;
    insertChild(blipItem,0); //insert at the top
  }
}

BlipContainer::~BlipContainer()
{
}


QVariant BlipContainer::data ( int column, int role) const
{
  return headers.value(column);
}


TreeItem* BlipContainer::child ( const NodeId& id )
{
  
  return blipData->at(id); //TODO: catch the case when the blip with that ID doesn't exist.
}


int BlipContainer::childCount() const
{
  return blipData->size();
}


bool BlipContainer::insertBlip ( const NodeId& id, int position)
{
  // get the blip(Item)
  TreeItem* blipItem = child(id);
  
  // check wether an item for the blip with the given id already exists
  if( blipItem->parent() == this )
  { return false; }
  
  // if the blipItem is registered with a different parent, remove it from there
  if( blipItem->parent() != nullptr )
  { blipItem->parent()->removeChildren(blipItem->childNumber(),1); }
  
  // attempt the insertion
  return insertChild(blipItem, position) ;
}

bool BlipContainer::removeBlip (const NodeId& id )
{
  TreeItem* blipItem = child(id);
  TreeItem* parentItem = blipItem->parent();
  if( parentItem )
  { return parentItem->removeChildren(blipItem->childNumber(),1); }
  //else
  return false;
}


QAbstractItemModel* BlipContainer::model() const
{
  if( parentModel  )
  { return parentModel; }
  //else
  return TreeItem::model();
}


