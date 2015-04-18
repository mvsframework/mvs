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

#ifndef NODEID_H
#define NODEID_H

#include "nodeid_128.h"
using NodeId=NodeId_128;

const NodeId NULL_NODE_ID = NULL_NODE_128_ID;

// #include "DDS/ddssupport.h"
// using NodeId=DDS_GUID_t;

#endif // NODEID_H
