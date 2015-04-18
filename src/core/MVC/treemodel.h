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

#ifndef TREEMODEL_H
#define TREEMODEL_H

#include <QtCore/QAbstractItemModel>

class TreeItem; // forward declaration

/** \brief A model/view-model base class for tree models.
 * 
 * Related Qt documentation:
 * - https://qt-project.org/doc/qt-5.0/qtwidgets/model-view-programming.html#model-subclassing-reference
 * - https://qt-project.org/doc/qt-5.0/qtwidgets/itemviews-editabletreemodel.html
 * 
 * 
 * \internal How I think a model is parsed by views:
 * - Get the "root index" of the root element by calling QModelIndex();
 * - Get the rows and columns underneath root by using
 * -- model::rowCount(QModelIndex())
 * -- model::columnCount(QModelIndex())
 * - Build the indices of the children of the root element
 * -- model::index( row, column, QModelIndex() )
 * - Query the children (via their indices) for their rows and columns
 * -- model::rowCount(rootChildIndex)
 * -- model::columnCount(rootChildIndex)
 * - etc.
 * 
 * Once all the indeces and have been found, the data is queried via:
 * -model::data(childIndex, role)
 */
class TreeModel : public QAbstractItemModel {
  Q_OBJECT
public:
  TreeModel(TreeItem* const rootItem, QObject* parent = nullptr);
  virtual ~TreeModel();
  
  virtual QVariant data ( const QModelIndex& index, int role ) const;
  virtual int columnCount ( const QModelIndex& parent ) const;
  
  virtual Qt::ItemFlags flags ( const QModelIndex& index ) const;
  
  /** \brief The number of children (i.e. rows beneath) of an item
   * \param[in] parent The model index of the parent item.
   * \return The number of children (i.e. rows) the parent has.
   */
  virtual int rowCount ( const QModelIndex& parent ) const;
  
  /** \brief Obtain the model index for the parent of the given child.
   * 
   * Provides a model index corresponding to the parent of any given child item.
   * If the model index specified corresponds to a top-level item in the model, 
   * or if there is no valid parent item in the model, the function must return 
   * an invalid model index, created with the empty QModelIndex() constructor.
   */
  virtual QModelIndex parent ( const QModelIndex& child ) const;
  
  /** \brief  Obtain model indeces corresponding to children of a given parent.
   * 
   * Given a model index for a parent item, this function allows views and 
   * delegates to access children of that item. If no valid child item - 
   * corresponding to the specified row, column, and parent model index, can be 
   * found, the function must return QModelIndex(), which is an invalid model 
   * index.
   */
  virtual QModelIndex index ( int row, int column, const QModelIndex& parent ) const;
  virtual QVariant headerData ( int section, Qt::Orientation orientation, int role = Qt::DisplayRole ) const;
  
protected:
  /** \brief The root item, which is also holding the header information. */
  TreeItem* rootItem;
  TreeItem* getItem(QModelIndex const& index) const;
  
};

#endif // TREEMODEL_H
