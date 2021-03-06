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

#ifndef LOCATIONID_H
#define LOCATIONID_H

#include "IDL/data.h"
#include <limits>

/** \brief A class representing an identifyer for locations. 
 * \note An ElementId is not necessarily unique.
 * \internal This type should be compatible to the Qt qHash() function return so 
 * that Location can be used in QHash containers. */
using LocationId = IDL::Data::LocationId;

/** \brief A value representing an invalid LocationId. */
const LocationId NULL_LOCATION_ID = std::numeric_limits<LocationId>::max(); // FIXME: Magic Numbers


#endif // LOCATIONID_H