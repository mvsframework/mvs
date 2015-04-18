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

#ifndef TEMPLATE_DERIVEDITEM_H
#define TEMPLATE_DERIVEDITEM_H


#include "simitem.h"

class DerivedItemPrivate; // forward declaration

class DerivedItem : public SimItem{
  Q_OBJECT
public:
  using Settings = XML::SAI::SimItem_t; //NOTE: Change to something like XML::SAI:DerivedItem_t
  
  explicit DerivedItem( Settings* const settings
                       ,SimItem* parent);
  virtual ~DerivedItem();

protected:
  explicit DerivedItem ( DerivedItemPrivate& dd, SimItem* parent = nullptr );
  
  virtual void initializeItem();
  virtual void startItem();
  virtual void stopItem();
  virtual void unInitializeItem();

private:
  Q_DISABLE_COPY(DerivedItem)
  Q_DECLARE_PRIVATE(DerivedItem);

};

#endif // TEMPLATE_DERIVEDITEM_H
