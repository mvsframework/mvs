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

#include "MVC/treemodel.h"
#include "MVC/treeitem.h"

TreeModel::TreeModel( TreeItem* const rootItem, QObject* parent )
  :QAbstractItemModel(parent)
  ,rootItem(rootItem)
{

}

TreeModel::~TreeModel()
{

}

Qt::ItemFlags TreeModel::flags ( const QModelIndex& index ) const
{
  if( !index.isValid() )
  { return 0; }
  //else
  return QAbstractItemModel::flags ( index );
}


QVariant TreeModel::data ( const QModelIndex& index, int role ) const
{
  if (!index.isValid())
  { return QVariant();}

//   if (role != Qt::DisplayRole /*&& role != Qt::EditRole*/)
//   { return QVariant(); }

  TreeItem *item = getItem(index);

  return item->data(index.column(),role);
}

int TreeModel::columnCount ( const QModelIndex& parent ) const
{
  return rootItem->columnCount();
}

int TreeModel::rowCount ( const QModelIndex& parent ) const
{
  TreeItem* parentItem = getItem(parent);
  return parentItem->childCount();
}

QModelIndex TreeModel::parent ( const QModelIndex& child ) const
{
  if (!child.isValid())
  { return QModelIndex(); }

  TreeItem *childItem = getItem(child);
  TreeItem *parentItem = childItem->parent();

  if (parentItem == rootItem)
  { return QModelIndex(); }
  // else
  return createIndex(parentItem->childNumber(), 0, parentItem);
}

QModelIndex TreeModel::index ( int row, int column, const QModelIndex& parent ) const
{
  if (parent.isValid() && parent.column() != 0)
  { return QModelIndex(); }
  
  TreeItem *parentItem = getItem(parent); //NOTE: this returns the rootItem if parent is QModelIndex()!

  TreeItem *childItem = parentItem->child(row);
  if (childItem)
  { return createIndex(row, column, childItem); }
  //else
  return QModelIndex();
}



TreeItem* TreeModel::getItem ( const QModelIndex& index ) const
{
  if( index.isValid() )
  {
    TreeItem *item = static_cast<TreeItem*>(index.internalPointer());
    if (item)
    { return item; }
  }
  //else
  return rootItem;
}

QVariant TreeModel::headerData ( int section, Qt::Orientation orientation, int role ) const
{
  if( orientation == Qt::Horizontal && role == Qt::DisplayRole )
  { return rootItem->data(section, role); }
  
  //else
  return QVariant();
}


