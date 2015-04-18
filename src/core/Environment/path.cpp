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

#include "path_p.h"
#include "path.h"

#include <assert.h>

#ifdef WITH_GRAPHICS
  #include <QtWidgets/QGraphicsSceneEvent>
#endif // WITH_GRAPHICS

//############################################################################//
//                                                                            //
//  PathPrivate                                                               //
//                                                                            //
//############################################################################//

PathPrivate::PathPrivate(Path* q)
:q_ptr(q)
{//NOTE: Do _NOT_ use q_ptr or q in here, it _WILL_ break things!
  
}


PathPrivate::~PathPrivate()
{
  
}



//############################################################################//
//                                                                            //
//  Path                                                                      //
//                                                                            //
//############################################################################//

/** \note This constructor does NOT do any work!
 * All Path level construction needs to happen in 
 * Path(PathPrivate& dd, ...)!
 */
Path::Path(QObject* parent)
  :Path(*new PathPrivate( this ), parent)
{}

Path::Path(PathPrivate& dd, QObject* parent)
  :QObject(parent)
  ,d_ptr(&dd)
{
  Q_D( Path ); Q_ASSERT(d);  
}


Path::~Path()
{
}

#ifdef WITH_GRAPHICS
void Path::setGraphicsItem(QGraphicsItem* item)
{
  Q_D(Path); Q_ASSERT(d);
  d->graphicsItem = item;
}

QGraphicsItem* Path::graphicsItem() const
{
  Q_D(const Path); Q_ASSERT(d);
  return d->graphicsItem;
}

void Path::contextMenuEvent(QGraphicsSceneContextMenuEvent* event)
{

}
#endif // WITH_GRAPHICS


#include "path.moc"