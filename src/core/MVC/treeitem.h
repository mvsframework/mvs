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

#ifndef TREEITEM_H
#define TREEITEM_H


#include <QtCore/QList>
#include <QtCore/QVariant>
#include <QtCore/QString>
#include <QtCore/QModelIndex>

class QAbstractItemModel; // forward declaration

/** \brief An abstract base class for tree views in a Model-View-Container setup. */
class TreeItem {
public:
  
  /** \brief The aspects for which an item should return data.
   */
//   enum DataAspect {
  enum class DataAspect : int { // This doesn't work as QModelIndex::index expects an int
    name,    ///< \brief The name of the variable the item represents.
    value,   ///< \brief The value of the variable the item represents.
    type,    ///< \brief The type or unit of the variable the item represents.
    comment  ///< \brief A comment for the variable the item represent.
  };
  
  TreeItem(TreeItem* parent = nullptr);
  virtual ~TreeItem();
     
  TreeItem* child(int number);
  TreeItem* parent();
  int childNumber() const;
  
  virtual int childCount() const;
  virtual int columnCount() const;
  
  /** \brief Insert an item into the list of children.
   * \note The TreeItem does \b not take ownership of the memory!
   */
  bool insertChild(TreeItem* const item, int position = 0 );
//   bool insertColumns(int position, int columns);
  
  /** \brief Append an item to the list of children. */
  bool appendChild(TreeItem* const item);
  
  /** \brief Remove an item from the list of children.
   * \note This does not free the memroy associated with the removed item!
   */
  bool removeChildren(int position, int count = 1);
//   bool removeColumns(int position, int columns);
  
  virtual QVariant data(int column, int role = Qt::DisplayRole) const = 0;
//   bool setData(int column, const QVariant &value);
  
  /** \brief Get the index for this item. */
  QModelIndex index() const;

  
protected:
  int numberOfColumns = 4;

  virtual QAbstractItemModel* model() const;

  
private:
  Q_DISABLE_COPY(TreeItem)
  QList<TreeItem*> childItems;
  TreeItem* parentItem = nullptr;
  
  QString itemName() const;
    
};


QString enumToQString(TreeItem::DataAspect const & aspect);

#endif // TREEITEM_H
