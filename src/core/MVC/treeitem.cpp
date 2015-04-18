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

#include "MVC/treeitem.h"
#include <boost/graph/graph_concepts.hpp>
#include <assert.h>
#include <QtCore/QDebug>

TreeItem::TreeItem( TreeItem* parent )
  // :parentItem(parent) // this doesn't really do anything... calling appendChild below instead
{
  if(parent != nullptr )
  { parent->appendChild(this); }
}

TreeItem::~TreeItem()
{
  // detach all children from this as their parent.
  removeChildren(0,childCount());
}

TreeItem* TreeItem::parent()
{
  return parentItem;
}

TreeItem* TreeItem::child ( int number )
{
  Q_ASSERT(number < childCount());
  return childItems.value(number);
}

int TreeItem::childCount() const
{
  return childItems.count();
}

int TreeItem::columnCount() const
{
  return numberOfColumns;
}


int TreeItem::childNumber() const
{
  if( parentItem != nullptr )
  { return parentItem->childItems.indexOf(const_cast<TreeItem*>(this)); }
  
  // else // this TreeItem is a root item
  return 0;
}


bool TreeItem::insertChild( TreeItem*const item, int position )
{
  if (position < 0 || position > childItems.size())
  { return false; }
  
  // remove item from a potential other parent
  if( item->parentItem != nullptr )
  { item->parentItem->removeChildren(item->childNumber()); }
  
  // add it to this item as a child.
  childItems.insert(position, item);
  item->parentItem = this;

  return true;
}

bool TreeItem::appendChild ( TreeItem*const item )
{
  return insertChild(item, childCount());
}


bool TreeItem::removeChildren ( int position, int count )
{
  if (position < 0 || position + count > childItems.size())
  { return false; }

  for (int i = 0; i < count; ++i)
  { 
    // remove the entry from the childItems list
    //NOTE: DO NOT DELETE the treeItem pointer as the TreeItem does _not_ have 
    // ownership of the memory!
    TreeItem* item = childItems.takeAt(position); 
    item->parentItem = nullptr;
  }

  return true;
}

QAbstractItemModel* TreeItem::model() const
{
  if (parentItem != nullptr)
  { return parentItem->model(); }
  //else
  return nullptr ;
}

QModelIndex TreeItem::index() const
{
  QModelIndex myIndex;
  
  if( parentItem == nullptr ) // this is a root item
  { myIndex = QModelIndex(); }
  else
  { 
    QModelIndex parentIndex = parentItem->index();
    myIndex = model()->index(childNumber(),0,parentIndex);
  }
  return myIndex;
}


QString TreeItem::itemName() const
{
  return data(static_cast<int>(DataAspect::name)).toString();
}




QString enumToQString ( const TreeItem::DataAspect& aspect )
{
  using da=TreeItem::DataAspect;
  switch( aspect )
  {
    case da::name :     return QObject::tr("Name");
    case da::value :    return QObject::tr("Value");
    case da::comment :  return QObject::tr("Comment");
    case da::type :     return QObject::tr("Type");
    default : return QObject::tr("(Unknown TreeItem::DataAspect = %1)")
                    .arg(static_cast<int>(aspect));
  }
}




