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

#ifndef TOPICOPTIONS_H
#define TOPICOPTIONS_H

#include <QtCore/QFlags>


/** \brief A enum containing option flags for Topics. 
 * 
 * This is a bitmask options enum like [QFlags](https://qt-project.org/doc/qt-5.0/qtcore/qflags.html).
 * 
 * hex  | decimal | binary
 * ----:|--------:|---------:
 * 0x00 |       0 | 00000000
 * 0x01 |       1 | 00000001
 * 0x02 |       2 | 00000010
 * 0x04 |       4 | 00000100
 * 0x08 |       8 | 00001000
 * 0x10 |      16 | 00010000
 * 0x20 |      32 | 00100000
 * 0x40 |      64 | 01000000
 * 0x80 |     128 | 10000000
 * 
 * 
 *\internal I would have liked to have this be a protected member of Topic, but
 * that doesn't seem to work, see http://stackoverflow.com/q/14761672/1984137.
 */
enum class TopicOptions {
   NoOptions    = 0x00 ///< \brief (This is a defunct case.)
   
  // mutually exclusive options, i.e. only using the numerals 1,2,4,8
  ,Send         = 0x01 ///< \brief The topic can send.
  ,Receive      = 0x02 ///< \brief The topic can receive.
  // combined options
  ,SendReceive  = 0x03 ///< \brief The topic can send and receive.
                       ///<
                       ///< This is 0x01 | 0x02 = 0x03 (bitwise OR).
};  

#endif // TOPICOPTIONS_H