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

#ifndef TEMPLATE_DERIVEDITEM_PRIVATE_H
#define TEMPLATE_DERIVEDITEM_PRIVATE_H

#include "simitem_p.h"

class DerivedItem;

class DerivedItemPrivate : public SimItemPrivate {
private:
  Q_DECLARE_PUBLIC(DerivedItem);
  DerivedItem* const q_ptr;

public:
  DerivedItemPrivate( DerivedItem* q
                     ,DerivedItem::Settings* const settings
                     ,SimItem* parent  );
  virtual ~DerivedItemPrivate();
     
  DerivedItem::Settings* const xmlSettings;
     
  void unInitializeDerivedItem();


};

#endif // TEMPLATE_DERIVEDITEM_PRIVATE_H
