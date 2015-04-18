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

#ifndef PODITEM_H
#define PODITEM_H

#include "MVC/treeitem.h"

template<typename T>
class PodItem : public TreeItem {
public:
  PodItem( T & data, QString const & name, QString const & unit
          ,QString const & comment = "", TreeItem* parent = nullptr )
    :TreeItem(parent)
    ,pod(data)
    ,name(name)
    ,unit(unit)
    ,comment(comment)
  {};
  virtual ~PodItem(){};
  
  virtual QVariant data ( int column, int role = Qt::DisplayRole ) const
  {
    if( role == Qt::DisplayRole )
    {
      switch( column )
      {
        case static_cast<int>(DataAspect::name) : return name;
        case static_cast<int>(DataAspect::value) : return QVariant(pod);
        case static_cast<int>(DataAspect::comment) : return comment;
        case static_cast<int>(DataAspect::type) : return unit;
        default: return QVariant("(default)");
      }
    }
    //else
    return QVariant();
  };
  
    
private:
  T &  pod;
  QVariant name;
  QVariant unit;
  QVariant comment;

};

#endif // PODITEM_H
