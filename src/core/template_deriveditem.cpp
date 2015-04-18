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

#include "template_deriveditem.h"
#include "template_deriveditem_p.h"

//############################################################################//
//                                                                            //
//  DerivedItemPrivate                                                        //
//                                                                            //
//############################################################################//


DerivedItemPrivate::DerivedItemPrivate ( DerivedItem* q
                                        ,DerivedItem::Settings* const settings
                                        ,SimItem* parent)
  :SimItemPrivate(q,settings)
  ,q_ptr ( q )
  ,xmlSettings(settings)
{ //NOTE: Do _NOT_ use q_ptr or q in here, it _WILL_ break things!
}

DerivedItemPrivate::~DerivedItemPrivate()
{
}

void DerivedItemPrivate::unInitializeDerivedItem()
{
  // undo whatever DerivedItem::initializeItem() did...
}


//############################################################################//
//                                                                            //
//  DerivedItem                                                               //
//                                                                            //
//############################################################################//

/** \note This constructor does NOT do any work!
 * All DerivedItem level construction needs to happen in 
 * DerivedItem(DerivedItemPrivate & dd, ...)!
 */
DerivedItem::DerivedItem( Settings* const settings
                         ,SimItem* parent)
  :DerivedItem(*new DerivedItemPrivate(this,settings,parent),parent)
{}

DerivedItem::DerivedItem( DerivedItemPrivate& dd
                         ,SimItem* parent )
  : SimItem(dd, parent) // DerivedItems have SimItems as parents
{
  Q_D(DerivedItem); Q_ASSERT(d);
  /** \internal 
   * This will only indicate the status of the DerivedItem, any derived classes
   * will not be constructed at this point!
   */
  d->setStatus(Status::Constructed); 
}


DerivedItem::~DerivedItem()
{
  Q_D(DerivedItem); Q_ASSERT(d);
  d->unInitializeDerivedItem();
}




void DerivedItem::initializeItem()
{
  SimItem::initializeItem();
  // do local work below ...
}

void DerivedItem::startItem()
{
  SimItem::startItem();
  // do local work below ...
}

void DerivedItem::stopItem()
{
  // do local work above...
  SimItem::stopItem();
}

void DerivedItem::unInitializeItem()
{
  Q_D(DerivedItem); Q_ASSERT(d);
  d->unInitializeDerivedItem();
  
  // do local work above...
  SimItem::unInitializeItem();
}

#include "template_deriveditem.moc"
