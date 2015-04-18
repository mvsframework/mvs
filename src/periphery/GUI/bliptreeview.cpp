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

#include "bliptreeview.h"

#include <assert.h>
#include "MVC/treeitem.h"
#include "blip.h"

#include <QtGui/QDrag>

BlipTreeView::BlipTreeView()
{
  setContextMenuPolicy(Qt::CustomContextMenu);
  connect(this, &BlipTreeView::customContextMenuRequested,
          [=](QPoint const & localPoint)
          { 
            NodeId id;
            if( mapToBlip(localPoint,id) )
            { emit blipContextMenuRequested(mapToGlobal(localPoint),id); }
          });
  
  setDragEnabled(true);
}

BlipTreeView::~BlipTreeView()
{

}

bool BlipTreeView::mapToBlip ( const QPoint& point, NodeId& id )
{
  // get the item that has been clicked on. This could be a blip or any of its
  // subitems
  QModelIndex itemIndex = indexAt(point);
  
  if( !itemIndex.isValid() )
  { return false; }
  
  TreeItem* item = static_cast<TreeItem*>(itemIndex.internalPointer());
  
  int childNumber = -1;
  // iterate all the way up to the rootItem, the blipContainer,...
  while( item->parent() != nullptr )
  {
   childNumber = item->childNumber(); // ...but keep the last childNumber so that...
   item = item->parent();
  }
  Q_ASSERT(childNumber != -1);
  
  // .. the blip can be recovered
  if( Blip* blip = dynamic_cast<Blip*>( item->child(childNumber) ) )
  { // cast was sucessfull (which should be the norm)
    id =  blip->id();
    return true;
  }
  // else
  return false;
}


void BlipTreeView::mouseMoveEvent(QMouseEvent* event)
{
//     QTreeView::mouseMoveEvent(event);
  
  // get the item that has been clicked on. This could be a blip or any of its
  // subitems
  QModelIndex itemIndex = indexAt(event->pos());
   
  QList<QModelIndex> indices;
  indices << itemIndex;
  
  QMimeData* mime = model()->mimeData(indices);

  QDrag* drag = new QDrag(this);
  drag->setMimeData(mime);
  drag->exec();
}


#include "bliptreeview.moc"